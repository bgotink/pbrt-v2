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
#include "log.h"
#include "rng.h"
#include "vis/visibility.h"

#include <set>

namespace shaft {
    
    struct ShaftGeometry {
        typedef std::list<Vector4f> plane_list;
        typedef plane_list::iterator plane_iter;
        typedef plane_list::const_iterator plane_citer;
        
        plane_list planes;
        int main_axis;
        
        const BBox receiver_bbox, light_bbox;
        const BBox bbox;
        
        bool intersects(const Reference<Triangle> &triangle, const Mesh &mesh) const;

        bool Intersect(const Ray &ray, Intersection *isect = NULL) const;
        bool IntersectP(const Ray &ray) const;
        
        ShaftGeometry(Reference<ElementTreeNode> &receiver_node, Reference<ElementTreeNode> &light_node);
        inline ~ShaftGeometry() {}
        
    private:
        friend class Shaft;
    };
    
    class Shaft : public ReferenceCounted {
        typedef std::vector<unsigned int> nblist;
        typedef nblist::iterator nbiter;
        typedef nblist::const_iterator nbciter;

        typedef std::list<unsigned int> nbllist;
        typedef nbllist::iterator nbliter;
        typedef nbllist::const_iterator nblciter;
        
        typedef std::set<unsigned int> nbset;
        
#if defined(SHAFT_LOG) && defined(SHAFT_SHOW_LEAFS)
        static uint32_t NEXT_UID;
        const uint32_t uid;
#endif
        
        Reference<ElementTreeNode> receiverNode;
        Reference<ElementTreeNode> lightNode;
        
        ShaftGeometry geometry;
        
        nbllist triangles;
        nbllist filtered_triangles;
        
#if defined(SHAFT_LOG) && defined(SHAFT_SHOW_DEPTHS)
        uint32_t depth;
#endif
        
        const ::shaft::vis::VisibilityCalculator *vis;
        
        Shaft(Reference<ElementTreeNode> &receiver, Reference<ElementTreeNode> &light, Reference<ElementTreeNode> &split, Shaft &parent);
        Shaft(Reference<ElementTreeNode> &receiver, Reference<ElementTreeNode> &light);

        friend class ShaftTreeNode;
        
        inline Mesh &getMesh() { return receiverNode->tree->mesh; }
        inline const Mesh &getMesh() const { return receiverNode->tree->mesh; }
        
        void filterTriangles();
        
    public:
        ~Shaft();
        
        inline const Reference<Triangle> &getTriangle(int idx) const {
            return getMesh().getTriangle(idx);
        }
        
        inline const Point &getPoint(unsigned int idx) const {
            return getMesh().getPoint(idx);
        }

        inline bool intersects(const Reference<Triangle> &triangle) {
            return geometry.intersects(triangle, getMesh());
        }
        
        inline bool empty() const {
            return triangles.empty();
        }
        
        inline bool Intersect(const Ray &ray, Intersection *isect) const {
            return geometry.Intersect(ray, isect);
        }
        bool IntersectP(const Ray &ray) const;
        float Visibility(const Ray &ray) const;
        
        inline bool isLeaf() const {
            return (receiverNode->is_leaf && lightNode->is_leaf)
                        || (empty() && receiverNode->is_leaf);
        }
        
#if defined(SHAFT_LOG) && defined(SHAFT_SHOW_DEPTHS)
        inline uint32_t getDepth() const { return depth; }
#endif
        
        void initProbVis(bool useProbVis, RNG *rng = NULL, const string * const type = NULL,
                         uint32_t regular_size_prims = 8, uint32_t regular_size_lights = 4);
        
        inline static Reference<Shaft> constructSubShaft(Reference<ElementTreeNode> &receiver, Reference<ElementTreeNode> &light,
                                                    Reference<ElementTreeNode> &split, Shaft &parent) {
            return Reference<Shaft>(new Shaft(receiver, light, split, parent));
        }
        
        inline static Reference<Shaft> constructInitialShaft(Reference<ElementTreeNode> &receiver, Reference<ElementTreeNode> &light) {
            return Reference<Shaft>(new Shaft(receiver, light));
        }
    };    
}

#endif /* defined(__pbrt__shaft__) */
