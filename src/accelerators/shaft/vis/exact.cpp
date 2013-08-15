//
//  exact.cpp
//  pbrt
//
//  Created by Bram Gotink on 24/12/12.
//
//

#include "exact.h"
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

    uint64_t ExactVisibilityCalculator::memsize() const {
        return static_cast<uint64_t>(sizeof(ExactVisibilityCalculator))
                + triangles.size() * sizeof(trisptrlist::value_type)
                + _triangles.size() + sizeof(trislist::value_type);
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

    VisibilityCalculator *createExactVisibilityCalculator(const shaft::Mesh &mesh,
                                                          const VisibilityCalculator::nbllist &triangles,
                                                          const Reference<ElementTreeNode> &receiver_node,
                                                          const Reference<ElementTreeNode> &light_node) {
        return new ExactVisibilityCalculator(mesh, triangles, receiver_node, light_node);
    }
    
}}
