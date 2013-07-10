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
    
    std::list<const ::Triangle *> getTrisPtrs(const std::list<unsigned int> &idx, const Mesh &mesh) {
        std::list<const ::Triangle *> result;
        
        std::list<unsigned int>::const_iterator i, end;
        end = idx.end();
        for (i = idx.begin(); i != end; i++) {
            result.push_back(&*(mesh.getTriangle(*i)));
        }
        
        return result;
    }
  
    ExactVisibilityCalculator::ExactVisibilityCalculator(const Mesh &mesh, const nbllist &triangles,
                                                         const Reference<ElementTreeNode> &receiver,
                                                         const Reference<ElementTreeNode> &light)
    : mesh(mesh), _triangles(VisibilityCalculator::getTriangles(mesh, triangles)),
    receiver_node(*receiver), light_node(*light), triangles(getTrisPtrs(triangles, mesh))
    {
    }
    
    float ExactVisibilityCalculator::Visibility(const Ray &ray) const {
        log::ShaftStartIntersectP();
        
        // check all triangles
        trisptrciter end = triangles.end();
        for (trisptrciter t = triangles.begin(); t != end; t++) {
            log::ShaftIntersectTest();
            log::ShaftAddIntersect();
            if ((*t)->IntersectP(ray))
                return 0.f;
        }
        log::ShaftNotIntersected();
        
        return (receiver_node.IntersectP(ray) || light_node.IntersectP(ray)) ? 0.f : 1.f;
    }
    
}}
