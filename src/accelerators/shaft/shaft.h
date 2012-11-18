//
//  shaft.h
//  pbrt
//
//  Created by Bram Gotink on 14/11/12.
//
//

#if defined(_MSC_VER)
#pragma once
#endif

#ifndef __pbrt__shaft__
#define __pbrt__shaft__

#include "pbrt.h"
#include "primitive.h"
#include "mesh.h"
#include "tree.h"
#include "surface.h"

namespace shaft {
    
    struct ShaftGeometry {
        typedef std::vector<Vector4f> plane_list;
        typedef plane_list::iterator plane_iter;
        typedef plane_list::const_iterator plane_citer;
        
        plane_list planes;
        int main_axis;
        
        const BBox &receiver_bbox, &light_bbox;
        const BBox bbox;
        
        Vector4f testplane_1, testplane_2;
        
        bool blockedBy(const Reference<Surface> &surface) const;
        bool intersects(const Reference<Triangle> &triangle, const Mesh &mesh) const;
        
        ShaftGeometry(Reference<ElementTreeNode> &receiver_node, Reference<ElementTreeNode> &light_node);
        ~ShaftGeometry() {}
        
    private:
        bool canBeBlockedBy(const BBox &bounding_box) const;
        std::list<Point> clampAndGetVertices(const Edge &edge) const;
    };
    
    class Shaft : public ReferenceCounted {
        typedef std::vector<Reference<Surface> > surface_list;
        typedef surface_list::iterator surface_iter;
        
        Reference<ElementTreeNode> receiverNode;
        Reference<ElementTreeNode> lightNode;
        
        ShaftGeometry geometry;
        surface_list surfaces;
        
        Shaft(Reference<ElementTreeNode> &receiver, Reference<ElementTreeNode> &light, Reference<ElementTreeNode> &split, Shaft &parent);
        
        friend class Surface;
        
    public:
        const Reference<Triangle> &getTriangle(int idx) {
            return receiverNode->tree->mesh.getTriangle(idx);
        }
    };

    class ShaftAccel : public Aggregate {
    public:
        bool canIntersect() { return true; }
        
        ~ShaftAccel();
    private:
        ElementTree *tree;
    };
    
}

#endif /* defined(__pbrt__shaft__) */
