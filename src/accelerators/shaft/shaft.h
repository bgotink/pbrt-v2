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
#include "log.h"

namespace shaft {
    
    struct ShaftGeometry {
        typedef std::list<Vector4f> plane_list;
        typedef plane_list::iterator plane_iter;
        typedef plane_list::const_iterator plane_citer;
        
        plane_list planes;
        int main_axis;
        
        const BBox receiver_bbox, light_bbox;
        const BBox bbox;
        
        Vector4f testplane_1, testplane_2;
        
        bool blockedBy(const Reference<Surface> &surface) const;
        
        bool intersects(const Reference<Triangle> &triangle, const Mesh &mesh) const;
        bool intersectsLine(Point one, Point two) const;
        
        bool Intersect(const Ray &ray, Intersection *isect = NULL) const;
        bool IntersectP(const Ray &ray) const;
        
        ShaftGeometry(Reference<ElementTreeNode> &receiver_node, Reference<ElementTreeNode> &light_node);
        inline ~ShaftGeometry() {}
        
    private:
        friend class Shaft;
        
        bool canBeBlockedBy(const BBox &bounding_box) const;
        
        std::list<Point> clampAndGetVertices(const Edge &edge) const;
    };
    
    class Shaft : public ReferenceCounted {
        typedef std::list<Reference<Surface> > surface_list;
        typedef surface_list::iterator surface_iter;
        typedef surface_list::const_iterator surface_citer;
        
        typedef std::vector<unsigned int> nblist;
        typedef nblist::iterator nbiter;
        typedef nblist::const_iterator nbciter;

        typedef std::list<unsigned int> nbllist;
        typedef nbllist::iterator nbliter;
        typedef nbllist::const_iterator nblciter;
        
        Reference<ElementTreeNode> receiverNode;
        Reference<ElementTreeNode> lightNode;
        
        ShaftGeometry geometry;
        surface_list surfaces;
        
        nbllist triangles;
        
#ifdef SHAFT_LOG
        uint32_t depth;
#endif
        
        Shaft(Reference<ElementTreeNode> &receiver, Reference<ElementTreeNode> &light, Reference<ElementTreeNode> &split, Shaft &parent);
        Shaft(Reference<ElementTreeNode> &receiver, Reference<ElementTreeNode> &light);
        
        Reference<Surface> constructTriangleSurface(nblist &triangles);
        Reference<Patch> createClippedPatch(const Reference<Triangle> &triangle) const;
        
        void classifyEdges(Reference<Surface> &surface) const;
        void updatePatchFacings(Reference<Surface> &surface) const;
        
        void computeLooseEdges(Reference<Surface> &surface);
        void combineSurfaces(surface_list &input_surfaces);
        
        friend class Surface;
        friend class ShaftTreeNode;
        
        inline Mesh &getMesh() { return receiverNode->tree->mesh; }
        inline const Mesh &getMesh() const { return receiverNode->tree->mesh; }
        
    public:
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
            return surfaces.empty() && receiverNode->empty();
        }
        
        inline bool Intersect(const Ray &ray, Intersection *isect) const {
            return geometry.Intersect(ray, isect);
        }
        bool IntersectP(const Ray &ray) const;
        bool GeomIntersectP(const Ray &ray) const;
        
        inline bool isLeaf() const {
            return receiverNode->is_leaf && lightNode->is_leaf;
        }
        
#ifdef SHAFT_LOG
        inline uint32_t getDepth() const { return depth; }
#endif
        
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
