//
//  vector.cpp
//  pbrt
//
//  Created by Bram Gotink on 31/05/13.
//
//

#include "vector.h"

namespace shaft { namespace surface {
    
    PlaneBoxResult operator*(const plane_t &plane, const bbox_t &bbox) {
//        const point_t &
        return BOTH;
    }
    
    plane_t CreatePlane(const point_t &a, const point_t &b, const point_t&c) {
        ::Vector n = (b - a) ^ (c - a);
        float d = -n.x * a.x - n.y * a.y - n.z * a.z;
        
        if (n.x + n.y + n.z == 0) {
            Severe("Creating plane between (%f,%f,%f), (%f,%f,%f), (%f,%f,%f) yields (0, 0, 0, %f)",
                   a.x, a.y, a.z,
                   b.x, b.y, b.z,
                   c.x, c.y, c.z,
                   d);
        }
        
        return plane_t(n.x, n.y, n.z, d);
    }
    
}}