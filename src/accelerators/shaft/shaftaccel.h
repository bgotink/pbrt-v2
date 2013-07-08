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
    
    class ShaftTreeNode;
    
    class ShaftAccel : public Aggregate {
        typedef std::vector<Reference<Primitive> > prim_list;
        typedef prim_list::iterator prim_iter;
        
        typedef std::vector<Reference<Shape> > shape_list;
        
    public:
        ShaftAccel(const prim_list &primitives, const prim_list &light_sources, uint32_t nbPointsInReceiverLeaf, uint32_t nbPointsInLightLeaf, bool drawShafts = false, const Point &shaftPoint = Point(0, 0, 0), const string * const probVisType = NULL);
        ShaftAccel(const prim_list &primitives, const shape_list &light_sources, uint32_t nbPointsInReceiverLeaf, uint32_t nbPointsInLightLeaf, bool drawShafts = false, const Point &shaftPoint = Point(0, 0, 0), const string * const probVisType = NULL);
        ~ShaftAccel();
        
        BBox WorldBound() const { return bounding_box; }
        
        // Primitive::CanIntersect()
        bool CanIntersect() const { return true; }
        
        // find intersection point
        bool Intersect(const Ray &ray, Intersection *isect) const;
        // check if shadow
        virtual bool IntersectP(const Ray &ray) const;
        virtual float Visibility(const Ray &ray) const;
        
    private:
        Reference<ElementTree> receiver_tree;
        Reference<ElementTree> light_tree;
        
        BBox bounding_box;
        
        mutable ShaftTreeNode *shaft_tree;
        Aggregate *fallback_accel;
        
        const bool showShafts;
        const Point shaftPoint;
        Reference<Primitive> prim;
        
    }; // class ShaftAccel
    
    ShaftAccel *createShaftAccel(const std::vector<Reference<Primitive> > &receivers,
                                 const std::vector<Reference<Shape> > &lights,
                                 const ParamSet &ps);
    
} // namespace shaft

#endif /* defined(__pbrt__shaftaccel__) */
