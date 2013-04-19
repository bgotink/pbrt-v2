//
//  surface.h
//  pbrt
//
//  Created by Bram Gotink on 15/11/12.
//
//

#ifndef __pbrt__surface__
#define __pbrt__surface__

#include "vector.h"
#include "geometry.h"
#include "memory.h"
#include "Mesh.h"
#include <set>
#include <list>

namespace shaft {
    
    enum Facing {
        TOWARDS_RECEIVER, TOWARDS_LIGHT, INCONSISTENT
    };
    
    struct Vertex {
        Point point;
    };
    
    class Patch;
    class Shaft;
    
    struct RawEdge : public ReferenceCounted {
        typedef uint64_t idtype;
        
        Vertex vertices[2];
        Reference<Patch> neighbour[2];
        bool is_inside;
        idtype mesh_edge;
        
        RawEdge(uint32_t from, uint32_t to, const Mesh &mesh);
        
        inline const Point &getPoint(int i) {
            Assert(i == 0 || i == 1);
            return vertices[i].point;
        }
        
        Reference<RawEdge> clone() const;
        
        inline static idtype createId(uint32_t a, uint32_t b) {
            idtype result;
            if (a > b) {
                result = a;
                result = (result << 32) + b;
            } else {
                result = b;
                result = (result << 32) + a;
            }
            return result;
        }
    private:
        RawEdge() {}
    };
    
    class Edge : public ReferenceCounted {
        Reference<RawEdge> raw_edge;
        bool is_flipped;
        
        friend class Surface;
        friend class Shaft;
        
    public:
        
        Edge(uint32_t from, uint32_t to, const Mesh& mesh);
        Edge(const Reference<RawEdge> &raw, bool flipped);
        
        inline const Vertex &getVertex(int idx) const {
            Assert(idx == 0 || idx == 1);
            return is_flipped ? raw_edge->vertices[idx ^ 1]
                              : raw_edge->vertices[idx];
        }
        
        inline const Reference<Patch> &getNeighbour() const {
            return is_flipped ? raw_edge->neighbour[1]
                              : raw_edge->neighbour[0];
        }
        
        inline Reference<Patch> &getNeighbour() {
            return is_flipped ? raw_edge->neighbour[1]
                              : raw_edge->neighbour[0];
        }
        
        inline const Reference<Patch> &getOwner() const {
            return is_flipped ? raw_edge->neighbour[0]
                              : raw_edge->neighbour[1];
        }
        
        inline void setOwner(const Reference<Patch> &p) {
            if (is_flipped)
                raw_edge->neighbour[0] = p;
            else
                raw_edge->neighbour[1] = p;
        }
        
        inline void setNeighbour(const Reference<Patch> &p) {
            if (is_flipped)
                raw_edge->neighbour[1] = p;
            else
                raw_edge->neighbour[0] = p;
        }
        
        inline Reference<Edge> flip() {
            return Reference<Edge>(new Edge(raw_edge, !is_flipped));
        }
        
        inline RawEdge::idtype getRawEdgeLabel() const {
            return raw_edge->mesh_edge;
        }
        
        Reference<Edge> clone() const;
    };
    
    class Patch : public ReferenceCounted {
    public:
        typedef std::list<Reference<Edge> > edge_list;
        typedef edge_list::iterator edge_iter;
        typedef edge_list::const_iterator edge_citer;
        
        Reference<Patch> clone() const;
        
        BBox getBoundingBox() const;
        
    private:
        friend class Surface;
        friend class Shaft;
        
        edge_list edges;
        Facing facing;
        int mesh_triangle; // triangle ID if facing == INCONSISTENT
    };
    
    class Surface : public ReferenceCounted {
    public:
        typedef std::list<Reference<Patch> > patch_list;
        typedef patch_list::iterator patch_iter;
        typedef patch_list::const_iterator patch_citer;
        
    private:
        typedef std::vector<RawEdge::idtype> nblist;
        typedef nblist::iterator nbiter;
        
        friend class Shaft;
        
        patch_list patches;
        BBox bounding_box;
        nblist loose_edges;
        
        void mergePatches();
        void simplifyPatches();
        void splitSurface(std::list<Reference<Surface> > &target_surfaces);
        static Reference<Surface> constructCombinedSurface(std::set<Reference<Surface> > &surfaces);
        
    public:
        inline const BBox &getBoundingBox() const { return bounding_box; }
        void computeBoundingBox();
        
        const std::list<Reference<RawEdge> > getRawEdges() const;
        std::list<Reference<RawEdge> > getRawEdges();
        Reference<Surface> clone() const;
        inline bool operator==(const Patch &o) const { return false; }
    };
    
}

#endif /* defined(__pbrt__surface__) */
