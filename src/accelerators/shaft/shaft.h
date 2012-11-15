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
        std::vector<Vector4f> planes;
        int main_axis;
        
        Vector4f testplane_1, testplane_2;
        
        bool blockedBy(const Reference<Surface> &surface) const;
        
        ShaftGeometry(ElementTreeNode &receiver_node, ElementTreeNode &light_node);
        
        ~ShaftGeometry() {}
        
    private:
        bool canBeBlockedBy(const BBox &bounding_box) const;
    };
    
    class Shaft : public ReferenceCounted {
        typedef std::vector<Reference<Surface> > surface_list;
        typedef surface_list::iterator surface_iter;
        
        ElementTreeNode *receiverNode;
        ElementTreeNode *lightNode;
        
        ShaftGeometry geometry;
        surface_list surfaces;
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
