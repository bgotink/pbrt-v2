//
//  falsecolor.cpp
//  pbrt
//
//  Created by Bram Gotink on 06/05/13.
//
//


/*
 pbrt source code Copyright(c) 1998-2012 Matt Pharr and Greg Humphreys.
 
 This file is part of pbrt.
 
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are
 met:
 
 - Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 
 - Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 
 */


// film/image.cpp*
#include "stdafx.h"
#include "film/falsecolor.h"
#include "spectrum.h"
#include "parallel.h"
#include "imageio.h"

// ImageFilm Method Definitions
FalseColorFilm::FalseColorFilm(int xres, int yres, Filter *filt, const float crop[4],
                     const string &fn, bool openWindow)
: Film(xres, yres) {
    filter = filt;
    memcpy(cropWindow, crop, 4 * sizeof(float));
    filename = fn;
    // Compute film image extent
    xPixelStart = Ceil2Int(xResolution * cropWindow[0]);
    xPixelCount = max(1, Ceil2Int(xResolution * cropWindow[1]) - xPixelStart);
    yPixelStart = Ceil2Int(yResolution * cropWindow[2]);
    yPixelCount = max(1, Ceil2Int(yResolution * cropWindow[3]) - yPixelStart);
    
    // Allocate film image storage
    pixels = new BlockedArray<Pixel>(xPixelCount, yPixelCount);
    
    // Possibly open window for image display
    if (openWindow || PbrtOptions.openWindow) {
        Warning("Support for opening image display window not available in this build.");
    }
}


void FalseColorFilm::AddSample(const CameraSample &sample,
                          const Spectrum &L) {
    // NOP
}


void FalseColorFilm::Splat(const CameraSample &sample, const Spectrum &L) {
    // NOP
}

void FalseColorFilm::Add(const CameraSample &sample, uint64_t value) {
    // Compute sample's raster extent
    float dimageX = sample.imageX - 0.5f;
    float dimageY = sample.imageY - 0.5f;
    int x0 = Ceil2Int (dimageX - filter->xWidth);
    int x1 = Floor2Int(dimageX + filter->xWidth);
    int y0 = Ceil2Int (dimageY - filter->yWidth);
    int y1 = Floor2Int(dimageY + filter->yWidth);
    x0 = max(x0, xPixelStart);
    x1 = min(x1, xPixelStart + xPixelCount - 1);
    y0 = max(y0, yPixelStart);
    y1 = min(y1, yPixelStart + yPixelCount - 1);
    if ((x1-x0) < 0 || (y1-y0) < 0)
    {
        PBRT_SAMPLE_OUTSIDE_IMAGE_EXTENT(const_cast<CameraSample *>(&sample));
        return;
    }
    
    for (int y = y0; y <= y1; ++y) {
        for (int x = x0; x <= x1; ++x) {
            (*pixels)(x - xPixelStart, y - yPixelStart).count += value;
        }
    }
}

void FalseColorFilm::Set(const CameraSample &sample, uint64_t value) {
    // Compute sample's raster extent
    float dimageX = sample.imageX - 0.5f;
    float dimageY = sample.imageY - 0.5f;
    int x0 = Ceil2Int (dimageX - filter->xWidth);
    int x1 = Floor2Int(dimageX + filter->xWidth);
    int y0 = Ceil2Int (dimageY - filter->yWidth);
    int y1 = Floor2Int(dimageY + filter->yWidth);
    x0 = max(x0, xPixelStart);
    x1 = min(x1, xPixelStart + xPixelCount - 1);
    y0 = max(y0, yPixelStart);
    y1 = min(y1, yPixelStart + yPixelCount - 1);
    if ((x1-x0) < 0 || (y1-y0) < 0)
    {
        PBRT_SAMPLE_OUTSIDE_IMAGE_EXTENT(const_cast<CameraSample *>(&sample));
        return;
    }
    
    for (int y = y0; y <= y1; ++y) {
        for (int x = x0; x <= x1; ++x) {
            (*pixels)(x - xPixelStart, y - yPixelStart).count = value;
        }
    }
}


void FalseColorFilm::GetSampleExtent(int *xstart, int *xend,
                                int *ystart, int *yend) const {
    *xstart = Floor2Int(xPixelStart + 0.5f - filter->xWidth);
    *xend   = Floor2Int(xPixelStart + 0.5f + xPixelCount  +
                        filter->xWidth);
    
    *ystart = Floor2Int(yPixelStart + 0.5f - filter->yWidth);
    *yend   = Floor2Int(yPixelStart + 0.5f + yPixelCount +
                        filter->yWidth);
}


void FalseColorFilm::GetPixelExtent(int *xstart, int *xend,
                               int *ystart, int *yend) const {
    *xstart = xPixelStart;
    *xend   = xPixelStart + xPixelCount;
    *ystart = yPixelStart;
    *yend   = yPixelStart + yPixelCount;
}

void GetRGB(float *rgb, uint64_t count, uint64_t max) {
    uint64_t half = max / 2;
    
    if (count < half) {
        // blue -> green
        float green = static_cast<float>(count) / static_cast<float>(half);
        float blue = 1.f - green;
        
        rgb[0] = 0.f;
        rgb[1] = green;
        rgb[2] = blue;
    } else {
        // green -> red
        float red = static_cast<float>(count - half) / static_cast<float>(half);
        float green = 1.f - red;
        
        rgb[0] = red;
        rgb[1] = green;
        rgb[2] = 0.f;
    }
}


void FalseColorFilm::WriteImage(float splatScale) {
    // Convert image to RGB and compute final pixel values
    int nPix = xPixelCount * yPixelCount;
    float *rgb = new float[3*nPix];
    int offset = 0;
    uint64_t maxCount = GetMax();
    
    for (int y = 0; y < yPixelCount; ++y) {
        for (int x = 0; x < xPixelCount; ++x) {
            GetRGB(&rgb[3*offset], (*pixels)(x, y).count, maxCount);
            ++offset;
        }
    }
    
    // Write RGB image
    ::WriteImage(filename, rgb, NULL, xPixelCount, yPixelCount,
                 xResolution, yResolution, xPixelStart, yPixelStart);
    
    // Release temporary image memory
    delete[] rgb;
}


void FalseColorFilm::UpdateDisplay(int x0, int y0, int x1, int y1,
                              float splatScale) {
}

uint64_t FalseColorFilm::GetMax() const {
    uint64_t maxCount = 0;
    
    for (int y = 0; y < yPixelCount; ++y) {
        for (int x = 0; x < xPixelCount; ++x) {
            maxCount = max(maxCount, (*pixels)(x, y).count);
        }
    }
    
    return maxCount;
}


FalseColorFilm *CreateFalseColorFilm(string filePart, const ParamSet &params, Filter *filter) {
    string filename = params.FindOneString("filename", PbrtOptions.imageFile);
    if (filename == "")
#ifdef PBRT_HAS_OPENEXR
        filename = "pbrt.exr";
#else
    filename = "pbrt.tga";
#endif
    
    char realFileName[ filename.size() + filePart.size() + 1 + 1 ];
    {
        const char *filename_str = filename.c_str();
        uint64_t extensionIdx = filename.size() - 4; // exr\0 or tga\0
        
        memcpy(realFileName, filename_str, extensionIdx);
        *(realFileName + extensionIdx) = '.';
        memcpy(realFileName + extensionIdx + 1, filePart.c_str(), filePart.size());
        memcpy(realFileName + extensionIdx + filePart.size() + 1, filename_str + extensionIdx, 4);
        realFileName[filename.size() + filePart.size() + 1] = '\0';
    }
    
    int xres = params.FindOneInt("xresolution", 640);
    int yres = params.FindOneInt("yresolution", 480);
    if (PbrtOptions.quickRender) xres = max(1, xres / 4);
    if (PbrtOptions.quickRender) yres = max(1, yres / 4);
    bool openwin = params.FindOneBool("display", false);
    float crop[4] = { 0, 1, 0, 1 };
    int cwi;
    const float *cr = params.FindFloat("cropwindow", &cwi);
    if (cr && cwi == 4) {
        crop[0] = Clamp(min(cr[0], cr[1]), 0., 1.);
        crop[1] = Clamp(max(cr[0], cr[1]), 0., 1.);
        crop[2] = Clamp(min(cr[2], cr[3]), 0., 1.);
        crop[3] = Clamp(max(cr[2], cr[3]), 0., 1.);
    }
    
    return new FalseColorFilm(xres, yres, filter, crop, realFileName, openwin);
}


