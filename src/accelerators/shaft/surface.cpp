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
    
    // RawEdge
    
    RawEdge::RawEdge(uint32_t from, uint32_t to, const Mesh &mesh) {
        vertices[0].point = mesh.getPoint(from);
        vertices[1].point = mesh.getPoint(to);
        
        mesh_edge = createId(from, to);
        is_inside = true;
    }
    
    Reference<RawEdge> RawEdge::clone() const {
        RawEdge &clone = *new RawEdge;
        
        clone.is_inside = is_inside;
        clone.mesh_edge = mesh_edge;
        clone.vertices[0].point = vertices[0].point;
        clone.vertices[1].point = vertices[1].point;
        
        return Reference<RawEdge>(&clone);
    }
    
    // Edge
    
    Edge::Edge(uint32_t from, uint32_t to, const Mesh& mesh) : raw_edge(new RawEdge(from, to, mesh)), is_flipped(false) {
    }
    
    Edge::Edge(const Reference<RawEdge> &raw, bool flipped) : raw_edge(raw), is_flipped(flipped) {}
    
    Reference<Edge> Edge::clone() const {
        return Reference<Edge>(new Edge(raw_edge->clone(), is_flipped));
    }
    
    // Patch
    
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
    
    // Surface
    
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
                    edges[label] = *edge;
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
    
    typedef set<Patch *> patch_set;
    typedef patch_set::iterator patch_siter;
    
    typedef stack<Patch *> patch_stack;
    
    // see [Laine, 06] fig 4.23
    void Surface::mergePatches() {
        patch_list new_patches;
        patch_set patches_processed;
        
        map<Reference<Patch>, Reference<Patch> > patch_remap;
        
        for (patch_iter patch_seed = patches.begin(); patch_seed != patches.end(); patch_seed++) {
            if (patches_processed.count(&* *patch_seed) != 0) {
                // already processed
                continue;
            }
            
            patch_set merge_patches;
            patch_stack traversal_stack;
            
            merge_patches.insert(& **patch_seed);
            traversal_stack.push(& **patch_seed);
            
            while (!traversal_stack.empty()) {
                Patch &patch = *traversal_stack.top();
                traversal_stack.pop();
                
                patches_processed.insert(&patch);
                
                for (Patch::edge_citer e = patch.edges.begin(); e != patch.edges.end(); e++) {
                    Reference<Patch> neighbour = (*e)->getNeighbour();
                                            
                    if ((*e)->raw_edge->is_inside
                            && (neighbour)
                            && (merge_patches.count(&*neighbour) == 0)
                            && (patch.facing != INCONSISTENT) && (neighbour->facing != INCONSISTENT)
                            && (patch.facing == neighbour->facing)) {
                        merge_patches.insert(&*neighbour);
                        traversal_stack.push(&*neighbour);
                    }
                }
            }
            
            Patch &new_patch = *new Patch;
            Reference<Patch> new_patch_ref = Reference<Patch>(&new_patch);
            new_patch.facing = (*patch_seed)->facing;
            new_patch.mesh_triangle = (*patch_seed)->mesh_triangle;
            
            for(patch_siter p = merge_patches.begin(); p != merge_patches.end(); p++) {
                Reference<Patch> patch(*p);
                patch_remap[patch] = new_patch_ref;
                for (Patch::edge_iter e = patch->edges.begin(); e != patch->edges.end(); e++) {
                    Reference<Patch> neighbour = (*e)->getNeighbour();
                    if (merge_patches.count(&* neighbour) == 0) {
                        Reference<Edge> clone = (*e)->clone();
                        clone->setOwner((*e)->getOwner());
                        clone->setNeighbour(neighbour);
                        
                        new_patch.edges.push_back(clone);
                    }
                }
            }
            
            new_patches.push_back(new_patch_ref);
        }
        
        Info("Merging patches: %lu -> %lu", patches.size(), new_patches.size());
        
        patches = new_patches;
        
        list<Reference<RawEdge> > raw_edges = getRawEdges();
        for (list<Reference<RawEdge> >::iterator re = raw_edges.begin(); re != raw_edges.end(); re++) {
            if ((*re)->neighbour[0])
                (*re)->neighbour[0] = & *patch_remap.at((*re)->neighbour[0]);
            if ((*re)->neighbour[1])
                (*re)->neighbour[1] = & *patch_remap.at((*re)->neighbour[1]);
        }
    }
    
    // defined but not implemented in [Laine, 06] fig 4.19
    // removes all internal edges from the patches
    void Surface::simplifyPatches() {
        Reference<Patch> patch;
        Reference<Edge> edge;
        for (patch_iter p = patches.begin(); p != patches.end(); p++) {
            patch = *p;
            for (Patch::edge_iter e = patch->edges.begin(); e != patch->edges.end(); e++) {
                edge = *e;
                if (edge->getNeighbour() && edge->getOwner()
                        && (&*edge->getNeighbour() == &*patch) && (&*edge->getOwner() == &*patch)) {
                    e = patch->edges.erase(e);
                }
            }
        }
    }
    
    typedef list<Reference<Surface> > surf_list;
    typedef surf_list::iterator surf_iter;
    
    // cf [Laine, 06] fig 4.25
    void Surface::splitSurface(surf_list &target_surfaces) {
        int tmp = target_surfaces.size();
        patch_set processed;
        for (patch_iter sp = patches.begin(); sp != patches.end(); sp++) {
            Reference<Patch> seed_patch = *sp;
            
            if (processed.count(&*seed_patch) != 0)
                continue;
            
            patch_set component_patches;
            patch_stack traversal_stack;
            traversal_stack.push(&*seed_patch);
            
            while (!traversal_stack.empty()) {
                Reference<Patch> patch = traversal_stack.top(); traversal_stack.pop();
                processed.insert(&*patch);
                
                Reference<Patch> neighbour;
                for (Patch::edge_iter e = patch->edges.begin(); e != patch->edges.end(); e++) {
                    neighbour = (*e)->getNeighbour();
                    if ((*e)->raw_edge->is_inside && neighbour
                                    && component_patches.count(&*neighbour) == 0) {
                        component_patches.insert(&*neighbour);
                        traversal_stack.push(&*neighbour);
                    }
                }
            }
            
            Surface &component_surface = *new Surface;
            for (patch_siter p = component_patches.begin(); p != component_patches.end(); p++) {
                Reference<Patch> patch = *p;
                component_surface.patches.push_back(patch);
                for (Patch::edge_iter e = patch->edges.begin(); e != patch->edges.end(); e++) {
                    Reference<Edge> edge = *e;
                    Reference<Patch> &neighbour = edge->getNeighbour();
                    if (neighbour && component_patches.count(&*neighbour) == 0) {
                        Reference<Edge> ourEdge = edge->clone();
                        ourEdge->setOwner(edge->getOwner());
                        edge->setOwner(NULL);
                        
                        ourEdge->setNeighbour(NULL);
                        
                        *e = ourEdge;
                    }
                }
            }
            target_surfaces.push_back(Reference<Surface>(&component_surface));
        }
        Info("Splitting surface with %lu patches into %lu surfaces", patches.size(), target_surfaces.size() - tmp);
    }
    
    typedef std::set<Reference<Surface> > surf_set;
    typedef surf_set::iterator surf_siter;
    
    Reference<Surface> Surface::constructCombinedSurface(surf_set &surfaces) {
        Surface &new_surf = *new Surface;
        patch_list &patches = new_surf.patches;
        
        for (surf_siter surf = surfaces.begin(); surf != surfaces.end(); surf++) {
            patches.insert(patches.end(), (*surf)->patches.begin(), (*surf)->patches.end());
        }
        
        new_surf.computeBoundingBox();
        new_surf.mergePatches();
        new_surf.simplifyPatches();
        
        return Reference<Surface>(&new_surf);
    }
    
    BBox Patch::getBoundingBox() const {
        BBox res;
        
        for (edge_citer edge = edges.begin(); edge != edges.end(); edge++) {
            res.Insert((*edge)->getVertex(0).point);
            res.Insert((*edge)->getVertex(1).point);
        }
        
        return res;
    }
    
    void Surface::computeBoundingBox() {
        bounding_box = BBox();
        
        for (patch_citer patch = patches.begin(); patch != patches.end(); patch++) {
            bounding_box.Insert((*patch)->getBoundingBox());
        }
    }
}