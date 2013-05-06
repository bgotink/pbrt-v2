//
//  falsecolor.h
//  pbrt
//
//  Created by Bram Gotink on 06/05/13.
//
//

#if defined(_MSC_VER)
#pragma once
#endif

#ifndef PBRT_FILM_FALSECOLOR_H
#define PBRT_FILM_FALSECOLOR_H

// film/image.h*
#include "pbrt.h"
#include "film.h"
#include "sampler.h"
#include "filter.h"
#include "paramset.h"

// ImageFilm Declarations
class FalseColorFilm : public Film {
public:
    // ImageFilm Public Methods
    FalseColorFilm(int xres, int yres, Filter *filt, const float crop[4],
              const string &filename, bool openWindow);
    void AddSample(const CameraSample &sample, const Spectrum &L);
    void Splat(const CameraSample &sample, const Spectrum &L);
    void GetSampleExtent(int *xstart, int *xend, int *ystart, int *yend) const;
    void GetPixelExtent(int *xstart, int *xend, int *ystart, int *yend) const;
    void WriteImage(float splatScale);
    void UpdateDisplay(int x0, int y0, int x1, int y1, float splatScale);
    
    void Add(const CameraSample &sample, uint64_t value = 1);
    void Set(const CameraSample &sample, uint64_t value = 0);
private:
    // ImageFilm Private Data
    Filter *filter;
    float cropWindow[4];
    string filename;
    int xPixelStart, yPixelStart, xPixelCount, yPixelCount;
    struct Pixel {
        Pixel() {
            count = 0;
        }
        uint64_t count;
    };
    BlockedArray<Pixel> *pixels;
    float *filterTable;
};


FalseColorFilm *CreateFalseColorFilm(const ParamSet &params, Filter *filter);

#endif // PBRT_FILM_FALSECOLOR_H
