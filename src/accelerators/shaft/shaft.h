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
        typedef std::list<Vector4f> plane_list;
        typedef plane_list::iterator plane_iter;
        typedef plane_list::const_iterator plane_citer;
        
        plane_list planes;
        int main_axis;
        
        const BBox &receiver_bbox, &light_bbox;
        const BBox bbox;
        
        Vector4f testplane_1, testplane_2;
        
        bool blockedBy(const Reference<Surface> &surface) const;
        
        bool intersects(const Reference<Triangle> &triangle, const Mesh &mesh) const;
        bool intersectsLine(Point one, Point two) const;
        
        ShaftGeometry(Reference<ElementTreeNode> &receiver_node, Reference<ElementTreeNode> &light_node);
        ~ShaftGeometry() {}
        
    private:
        friend class Shaft;
        
        bool canBeBlockedBy(const BBox &bounding_box) const;
        
        std::list<Point> clampAndGetVertices(const Edge &edge) const;
    };
    
    class Shaft : public ReferenceCounted {
        typedef std::list<Reference<Surface> > surface_list;
        typedef surface_list::iterator surface_iter;
        typedef surface_list::const_iterator surface_citer;
        
        typedef std::vector<uint32_t> nblist;
        typedef nblist::iterator nbiter;
        typedef nblist::const_iterator nbciter;
        
        Reference<ElementTreeNode> receiverNode;
        Reference<ElementTreeNode> lightNode;
        
        ShaftGeometry geometry;
        surface_list surfaces;
        
        Shaft(Reference<ElementTreeNode> &receiver, Reference<ElementTreeNode> &light, Reference<ElementTreeNode> &split, Shaft &parent);
        Shaft(Reference<ElementTreeNode> &receiver, Reference<ElementTreeNode> &light, nblist &triangles);
        
        Reference<Surface> constructTriangleSurface(nblist &triangles);
        Reference<Patch> createClippedPatch(const Reference<Triangle> &triangle) const;
        
        void classifyEdges(Reference<Surface> &surface) const;
        void updatePatchFacings(Reference<Surface> &surface) const;
        
        void computeLooseEdges(Reference<Surface> &surface);
        void combineSurfaces(surface_list &input_surfaces);
        
        friend class Surface;
        
        inline Mesh &getMesh() { return receiverNode->tree->mesh; }
        inline const Mesh &getMesh() const { return receiverNode->tree->mesh; }
        
    public:
        const Reference<Triangle> &getTriangle(int idx) const {
            return getMesh().getTriangle(idx);
        }
        
        const Point &getPoint(int idx) const {
            return getMesh().getPoint(idx);
        }

        bool intersects(const Reference<Triangle> &triangle) {
            return geometry.intersects(triangle, getMesh());
        }
        
        static Reference<Shaft> constructSubShaft(Reference<ElementTreeNode> &receiver, Reference<ElementTreeNode> &light,
                                                    Reference<ElementTreeNode> &split, Shaft &parent) {
            return Reference<Shaft>(new Shaft(receiver, light, split, parent));
        }
        
        static Reference<Shaft> constructInitialShaft(Reference<ElementTreeNode> &receiver, Reference<ElementTreeNode> &light,
                                                      nblist &triangles) {
            return Reference<Shaft>(new Shaft(receiver, light, triangles));
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
