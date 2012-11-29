
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


// core/geometry.cpp*
#include "stdafx.h"
#include "geometry.h"

// BBox Method Definitions
BBox Union(const BBox &b, const Point &p) {
    BBox ret = b;
    ret.pMin.x = min(b.pMin.x, p.x);
    ret.pMin.y = min(b.pMin.y, p.y);
    ret.pMin.z = min(b.pMin.z, p.z);
    ret.pMax.x = max(b.pMax.x, p.x);
    ret.pMax.y = max(b.pMax.y, p.y);
    ret.pMax.z = max(b.pMax.z, p.z);
    return ret;
}


BBox Union(const BBox &b, const BBox &b2) {
    BBox ret;
    ret.pMin.x = min(b.pMin.x, b2.pMin.x);
    ret.pMin.y = min(b.pMin.y, b2.pMin.y);
    ret.pMin.z = min(b.pMin.z, b2.pMin.z);
    ret.pMax.x = max(b.pMax.x, b2.pMax.x);
    ret.pMax.y = max(b.pMax.y, b2.pMax.y);
    ret.pMax.z = max(b.pMax.z, b2.pMax.z);
    return ret;
}

BBox BBox::Union(const Point &p) const {
    BBox b = *this;
    b.pMin.x = min(pMin.x, p.x);
    b.pMin.y = min(pMin.y, p.y);
    b.pMin.z = min(pMin.z, p.z);
    b.pMax.x = max(pMax.x, p.x);
    b.pMax.y = max(pMax.y, p.y);
    b.pMax.z = max(pMax.z, p.z);
    return b;
}

BBox BBox::Union(const BBox &b) const {
    BBox res = *this;
    res.pMin.x = min(pMin.x, b.pMin.x);
    res.pMin.y = min(pMin.y, b.pMin.y);
    res.pMin.z = min(pMin.z, b.pMin.z);
    res.pMax.x = max(pMax.x, b.pMax.x);
    res.pMax.y = max(pMax.y, b.pMax.y);
    res.pMax.z = max(pMax.z, b.pMax.z);
    return res;
}

BBox &BBox::Insert(const BBox &b) {
    
    pMin.x = min(pMin.x, b.pMin.x);
    pMin.y = min(pMin.y, b.pMin.y);
    pMin.z = min(pMin.z, b.pMin.z);
    
    pMax.x = max(pMax.x, b.pMax.x);
    pMax.y = max(pMax.y, b.pMax.y);
    pMax.z = max(pMax.z, b.pMax.z);
    
    return *this;
}

BBox &BBox::Insert(const Point &p) {
    
    pMin.x = min(pMin.x, p.x);
    pMin.y = min(pMin.y, p.y);
    pMin.z = min(pMin.z, p.z);
    
    pMax.x = max(pMax.x, p.x);
    pMax.y = max(pMax.y, p.y);
    pMax.z = max(pMax.z, p.z);
    
    return *this;
}


BBox BBox::Union(const BBox &b, const BBox &b2) {
    BBox ret;
    ret.pMin.x = min(b.pMin.x, b2.pMin.x);
    ret.pMin.y = min(b.pMin.y, b2.pMin.y);
    ret.pMin.z = min(b.pMin.z, b2.pMin.z);
    ret.pMax.x = max(b.pMax.x, b2.pMax.x);
    ret.pMax.y = max(b.pMax.y, b2.pMax.y);
    ret.pMax.z = max(b.pMax.z, b2.pMax.z);
    return ret;
}


void BBox::BoundingSphere(Point *c, float *rad) const {
    *c = .5f * pMin + .5f * pMax;
    *rad = Inside(*c) ? Distance(*c, pMax) : 0.f;
}


bool BBox::IntersectP(const Ray &ray, float *hitt0,
                      float *hitt1) const {
    float t0 = ray.mint, t1 = ray.maxt;
    for (int i = 0; i < 3; ++i) {
        // Update interval for _i_th bounding box slab
        float invRayDir = 1.f / ray.d[i];
        float tNear = (pMin[i] - ray.o[i]) * invRayDir;
        float tFar  = (pMax[i] - ray.o[i]) * invRayDir;

        // Update parametric interval from slab intersection $t$s
        if (tNear > tFar) swap(tNear, tFar);
        t0 = tNear > t0 ? tNear : t0;
        t1 = tFar  < t1 ? tFar  : t1;
        if (t0 > t1) return false;
    }
    if (hitt0) *hitt0 = t0;
    if (hitt1) *hitt1 = t1;
    return true;
}


