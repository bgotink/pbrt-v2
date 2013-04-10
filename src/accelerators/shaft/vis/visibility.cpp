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
    
    VisibilityCalculator::trislist VisibilityCalculator::getTriangles(const Mesh &mesh, const nbllist &tidx) {
        trislist result;
        
        const nblciter end = tidx.end();
        for (nblciter iter = tidx.begin(); iter != end; iter++)
            result.push_back(mesh.getTriangle(*iter));
        
        return result;
    }
}
}