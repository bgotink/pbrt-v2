//
//  exact.cpp
//  pbrt
//
//  Created by Bram Gotink on 24/12/12.
//
//

#include "visibility.h"
#include "../log.h"

using shaft::Mesh;
using shaft::ElementTreeNode;

namespace shaft { namespace vis {
  
    ExactVisibilityCalculator::ExactVisibilityCalculator(const Mesh &mesh, const nbllist &triangles,
                                                         const Reference<ElementTreeNode> &receiver,
                                                         const Reference<ElementTreeNode> &light)
    : mesh(mesh), triangles(VisibilityCalculator::getTriangles(mesh, triangles)),
    receiver_node(*receiver), light_node(*light)
    {}
    
    float ExactVisibilityCalculator::Visibility(const Ray &ray) const {
        ShaftStartIntersectP();
        
        // check all triangles
        for (trisciter t = triangles.begin(); t != triangles.end(); t++) {
            ShaftIntersectTest();
            if (IntersectsTriangle(*t, mesh, ray))
                return 0.f;
        }
        ShaftNotIntersected();
        
        return (receiver_node.IntersectP(ray) || light_node.IntersectP(ray)) ? 0.f : 1.f;
    }
    
}}
