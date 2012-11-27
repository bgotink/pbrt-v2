//
//  surface.cpp
//  pbrt
//
//  Created by Bram Gotink on 15/11/12.
//
//

#include "surface.h"
#include "shaft.h"

#include <map>
#include <set>
#include <stack>

using std::list;
using std::map;
using std::set;
using std::stack;

namespace shaft {
    
    Reference<RawEdge> RawEdge::clone() const {
        RawEdge &clone = *new RawEdge;
        
        clone.is_inside = is_inside;
        clone.mesh_edge = mesh_edge;
        clone.vertices[0].point = vertices[0].point;
        clone.vertices[1].point = vertices[1].point;
        
        return Reference<RawEdge>(&clone);
    }
    
    Reference<Edge> Edge::clone() const {
        return Reference<Edge>(new Edge(raw_edge->clone(), is_flipped));
    }
    
    Reference<Patch> Patch::clone() const {
        Patch &clone = *new Patch;
        
        clone.facing = facing;
        clone.mesh_triangle = mesh_triangle;
        
        Reference<Edge> eclone;
        for (edge_citer edge = edges.begin(); edge != edges.end(); edge++) {
            eclone = (*edge)->clone();
            eclone->setOwner(&clone);
            
            clone.edges.push_back(eclone);
        }
        
        return Reference<Patch>(&clone);
    }
    
    const list<Reference<RawEdge> > Surface::getRawEdges() const {
        list<Reference<RawEdge> > retVal;
        
        for (patch_citer patch = patches.begin(); patch != patches.end(); patch++) {
            for (Patch::edge_citer edge = (*patch)->edges.begin();
                 edge != (*patch)->edges.end();
                 edge++) {
                retVal.push_back((*edge)->raw_edge);
            }
        }
        
        return retVal;
    }
    
    list<Reference<RawEdge> > Surface::getRawEdges() {
        list<Reference<RawEdge> > retVal;
        
        for (patch_citer patch = patches.begin(); patch != patches.end(); patch++) {
            for (Patch::edge_citer edge = (*patch)->edges.begin();
                 edge != (*patch)->edges.end();
                 edge++) {
                retVal.push_back((*edge)->raw_edge);
            }
        }
        
        return retVal;
    }
    
    Reference<Surface> Surface::clone() const {
        Reference<Surface> clone = Reference<Surface>(new Surface());
        
        map<RawEdge::idtype, Reference<Edge> > edges;
        
        Reference<Patch> pclone;
        for (patch_citer patch = patches.begin(); patch != patches.end(); patch++) {
            pclone = (*patch)->clone();
            clone->patches.push_back(pclone);
            
            for (Patch::edge_iter edge = pclone->edges.begin(); edge != pclone->edges.end(); edge++) {
                RawEdge::idtype label = (*edge)->getRawEdgeLabel();
                if (edges.count(label) == 0) {
                    edges.at(label) = *edge;
                } else {
                    Reference<Edge> newEdge = edges.at(label)->flip();
                    newEdge->setOwner(&*pclone);
                    
                    *edge = newEdge;
                }
            }
        }
        
        clone->loose_edges = loose_edges;
        clone->bounding_box = bounding_box;

        return clone;
    }
    
    // see [Laine, 06] fig 4.23
    void Surface::mergePatches() {
        typedef set<Reference<Patch> > patch_set;
        typedef patch_set::iterator patch_siter;
        
        typedef stack<Reference<Patch> > patch_stack;
        
        patch_list new_patches;
        patch_set patches_processed;
        
        map<Reference<Patch>, Reference<Patch> > patch_remap;
        
        for (patch_iter patch_seed = patches.begin(); patch_seed != patches.end(); patch_seed++) {
            if (patches_processed.count(*patch_seed) != 0)
                // already processed
                continue;
            
            patch_set merge_patches;
            patch_stack traversal_stack;
            
            traversal_stack.push(*patch_seed);
            
            while (!traversal_stack.empty()) {
                Reference<Patch> patch = traversal_stack.top();
                traversal_stack.pop();
                
                patches_processed.insert(patch);
                
                for (Patch::edge_iter e = patch->edges.begin(); e != patch->edges.end(); e++) {
                    Reference<Patch> neighbour = Reference<Patch>((*e)->getNeighbour());
                    if ((*e)->raw_edge->is_inside
                            && (neighbour.GetPtr() != NULL)
                            && (merge_patches.count(neighbour) == 0)
                            && (patch->facing != INCONSISTENT) && (neighbour->facing != INCONSISTENT)
                            && (patch->facing == neighbour->facing)) {
                        merge_patches.insert(neighbour);
                        traversal_stack.push(neighbour);
                    }
                }
            }
            
            Patch &new_patch = *new Patch;
            Reference<Patch> new_patch_ref = Reference<Patch>(&new_patch);
            new_patch.facing = (*patch_seed)->facing;
            new_patch.mesh_triangle = (*patch_seed)->mesh_triangle;
            
            for(patch_siter p = merge_patches.begin(); p != merge_patches.end(); p++) {
                Reference<Patch> patch = *p;
                patch_remap.at(patch) = new_patch_ref;
                for (Patch::edge_iter e = patch->edges.begin(); e != patch->edges.end(); e++) {
                    if (merge_patches.count(Reference<Patch>((*e)->getNeighbour())) == 0) {
                        Reference<Edge> clone = (*e)->clone();
                        clone->setOwner((*e)->getOwner());
                        clone->setNeighbour((*e)->getNeighbour());
                        
                        new_patch.edges.push_back(clone);
                    }
                }
            }
            
            new_patches.push_back(Reference<Patch>(&new_patch));
        }
        
        patches = new_patches;
        
        list<Reference<RawEdge> > raw_edges = getRawEdges();
        for (list<Reference<RawEdge> >::iterator re = raw_edges.begin(); re != raw_edges.end(); re++) {
            if ((*re)->neighbour[0] != NULL)
                (*re)->neighbour[0] = & *patch_remap.at((*re)->neighbour[0]);
            if ((*re)->neighbour[1] != NULL)
                (*re)->neighbour[1] = & *patch_remap.at((*re)->neighbour[1]);
        }
    }
}