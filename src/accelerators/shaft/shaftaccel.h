//
//  shaftaccel.h
//  pbrt
//
//  Created by Bram Gotink on 29/11/12.
//
//

#ifndef __pbrt__shaftaccel__
#define __pbrt__shaftaccel__

#include "primitive.h"
#include "tree.h"

#include <vector>

namespace shaft {
    
    struct ShaftTreeNode;
    
    class ShaftAccel : public Aggregate {
        typedef std::vector<Reference<Primitive> > prim_list;
        typedef prim_list::iterator prim_iter;
        
        typedef std::vector<Reference<Shape> > shape_list;
        
    public:
        ShaftAccel(const prim_list &primitives, const prim_list &light_sources);
        ShaftAccel(const prim_list &primitives, const shape_list &light_sources);
        ~ShaftAccel();
        
        BBox WorldBound() const { return bounding_box; }
        
        // Primitive::CanIntersect()
        bool CanIntersect() const { return true; }
        
        // find intersection point
        inline bool Intersect(const Ray &ray, Intersection *isect) const {
            return fallback_accel->Intersect(ray, isect);
        }
        
        // check if shadow
        bool IntersectP(const Ray &ray) const;
        
    private:
        Reference<ElementTree> receiver_tree;
        Reference<ElementTree> light_tree;
        
        BBox bounding_box;
        
        mutable ShaftTreeNode *shaft_tree;
        
        Aggregate *fallback_accel;
    }; // class ShaftAccel
    
} // namespace shaft

#endif /* defined(__pbrt__shaftaccel__) */
