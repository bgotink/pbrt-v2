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

using std::list;
using std::map;

namespace shaft {
    
    
    const list<Reference<RawEdge> > Surface::getRawEdges() const {
        list<Reference<RawEdge> > retVal;
        
        for (patch_citer patch = patches.begin(); patch != patches.end(); patch++) {
            for (Patch::edge_citer edge = (*patch)->edges.begin();
                                   edge != (*patch)->edges.end();
                                   edge++) {
                retVal.push_back(edge->raw_edge);
            }
        }
        
        return retVal;
    }
    
    // see [Laine, 06] fig 4.20
    Reference<Surface> Surface::constructTriangleSurface(nblist &triangles, Shaft &shaft) {
        Surface &new_surface = *(new Surface);
        
        ShaftGeometry &sgeom = shaft.geometry;
        
        map<int, Reference<Edge> > edge_map;
        
        for (nbiter tidx = triangles.begin(); tidx != triangles.end(); tidx++) {
            const Reference<Triangle> &t = shaft.getTriangle(*tidx);
            // TODO implement!
        }
        
        return Reference<Surface>(&new_surface);
    }
    
}