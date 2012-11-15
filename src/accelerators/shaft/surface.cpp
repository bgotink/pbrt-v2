//
//  surface.cpp
//  pbrt
//
//  Created by Bram Gotink on 15/11/12.
//
//

#include "surface.h"

using std::list;

namespace shaft {
    
    
    const list<const Reference<RawEdge> > Surface::getRawEdges() const {
        list<const Reference<RawEdge> > retVal;
        
        for (patch_citer patch = patches.begin(); patch != patches.end(); patch++) {
            for (Patch::edge_citer edge = (*patch)->edges.begin();
                                   edge != (*patch)->edges.end();
                                   edge++) {
                retVal.push_back(edge->raw_edge);
            }
        }
    }
    
}