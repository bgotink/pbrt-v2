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
#include <list>

namespace shaft {
    
    enum Facing {
        TOWARDS_RECEIVER, TOWARDS_LIGHT, INCONSISTENT
    };
    
    struct Vertex {
        Point point;
    };
    
    class Patch;
    
    struct RawEdge : public ReferenceCounted {
        Vertex vertices[2];
        Patch *neighbour[2];
        bool is_inside;
        int mesh_edge;
    };
    
    class Edge : public ReferenceCounted {
        Reference<RawEdge> raw_edge;
        bool is_flipped;
        
        friend class Surface;
        
    public:
        Edge(Reference<RawEdge> raw, bool flipped) : raw_edge(raw), is_flipped(flipped) {}
        ~Edge() {}
        
        Vertex &getVertex(int idx) {
            Assert(idx == 0 || idx == 1);
            return is_flipped ? raw_edge->vertices[idx ^ 1]
                              : raw_edge->vertices[idx];
        }
        
        Patch *getNeighbour() {
            return is_flipped ? raw_edge->neighbour[1]
                              : raw_edge->neighbour[0];
        }
        
        Patch *getOwner() {
            return is_flipped ? raw_edge->neighbour[0]
                              : raw_edge->neighbour[1];
        }
        
        Reference<Edge> flip() {
            return Reference<Edge>(new Edge(raw_edge, !is_flipped));
        }
    };
    
    class Patch : public ReferenceCounted {
    public:
        typedef std::vector<Edge> edge_list;
        typedef edge_list::iterator edge_iter;
        typedef edge_list::const_iterator edge_citer;
        
    private:
        friend class Surface;
        
        edge_list edges;
        Facing facing;
        int mesh_triangle; // triangle ID if facing == INCONSISTENT
    };
    
    class Surface : public ReferenceCounted {
    public:
        typedef std::vector<Reference<Patch> > patch_list;
        typedef patch_list::iterator patch_iter;
        typedef patch_list::const_iterator patch_citer;
        
    private:
        typedef std::vector<int> nblist;
        typedef nblist::iterator nbiter;
        
        patch_list patches;
        BBox bounding_box;
        nblist loose_edges;
        
    public:
        const BBox &getBoundingBox() const { return bounding_box; }
        
        const std::list<const Reference<RawEdge> > getRawEdges() const;
    };
    
}

#endif /* defined(__pbrt__surface__) */
