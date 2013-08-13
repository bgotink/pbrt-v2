//
//  visibility.cpp
//  pbrt
//
//  Created by Bram Gotink on 24/12/12.
//
//

#include "visibility.h"

namespace shaft {
namespace vis {
    VisibilityCalculator::~VisibilityCalculator() {
    }
    
    VisibilityCalculator::trislist VisibilityCalculator::getTriangles(const Mesh &mesh, const nbllist &tidx, const Triangle *const mostBlockingOccluder) {
        trislist result;
        
        const nblciter end = tidx.end();
        Reference<Triangle> cur;
        for (nblciter iter = tidx.begin(); iter != end; iter++) {
            cur = mesh.getTriangle(*iter);
            if (&*cur != mostBlockingOccluder) // also valid if mostBlockingOccluder is NULL
                result.push_back(mesh.getTriangle(*iter));
        }
        
        return result;
    }
}
}