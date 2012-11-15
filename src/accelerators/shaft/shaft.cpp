//
//  shaft.cpp
//  pbrt
//
//  Created by Bram Gotink on 14/11/12.
//
//

#include "shaft.h"

using namespace std;

namespace shaft {

    bool ShaftGeometry::canBeBlockedBy(const BBox &bounding_box) const {
        return false;
    }
    
    typedef list<const Reference<RawEdge> > rawedge_clist;
    typedef rawedge_clist::const_iterator rawedge_citer;
    
    bool ShaftGeometry::blockedBy(const Reference<shaft::Surface> &surface) const {
        // main_axis = -1 if the two endpoints overlap,
        // in which case we ALWAYS split the shaft
        if (main_axis == -1) return false;
        
        if (!canBeBlockedBy(surface->getBoundingBox()))
            return false;
        
        const rawedge_clist raw_edges = surface->getRawEdges();
        
        for (rawedge_citer re = raw_edges.begin(); re != raw_edges.end(); re++) {
            const RawEdge &edge = *re->GetPtr();
            
            if ((edge.neighbour[0] == NULL || edge.neighbour[1] == NULL) && edge.is_inside)
                return false; // single edge inside shaft
        }

        
        
        // TODO
        return false;
    }
    
    ShaftGeometry::ShaftGeometry(ElementTreeNode &receiver, ElementTreeNode &light) {
        const BBox &receiver_bbox = receiver.bounding_box, &light_bbox = light.bounding_box;
        const Point receiver_center = receiver_bbox.getCenter(),
                    light_center = light_bbox.getCenter();
        
        if (receiver_bbox.Overlaps(light_bbox)) {
            // special case, we won't really use this shaft
            main_axis = -1;
        } else {
            const Vector v = (receiver_center - light_center).abs();
            
            if (v.x > v.y && v.x > v.z)
                main_axis = 0;
            else {
                if (v.y > v.z)
                    main_axis = 1;
                else
                    main_axis = 2;
            }
        }
        
        const BBox *first, *last;
        switch (main_axis) {
            case 0: // X
                if (receiver_center.x < light_center.x) {
                    first = &receiver_bbox;
                    last = &light_bbox;
                } else {
                    first = &light_bbox;
                    last = &receiver_bbox;
                }
                
                const Point &first_min = first->pMin,
                &first_max = first->pMax,
                &last_min = last->pMin,
                &last_max = last->pMax;

                Point
                    first_XYZ = first_max,
                    first_XyZ = Point(first_max.x, first_min.y, first_max.z),
                    first_Xyz = Point(first_max.x, first_min.y, first_min.z),
                    first_XYz = Point(first_max.x, first_max.y, first_min.z),
                
                    last_xyz = last_min,
                    last_xyZ = Point(last_min.x, last_min.y, last_max.z),
                    last_xYZ = Point(last_min.x, last_max.y, last_max.z),
                    last_xYz = Point(last_min.x, last_max.y, last_min.z);
                
                planes.push_back(CreatePlane(first_XYZ, first_XYz, last_xYz));
                planes.push_back(CreatePlane(first_XYZ, first_XyZ, last_xYZ));
                planes.push_back(CreatePlane(first_XyZ, first_Xyz, last_xyZ));
                planes.push_back(CreatePlane(first_XYz, first_Xyz, last_xyz));
                
                Point tmp1 = (receiver_center + Vector(0, 0, 1)), tmp2 = (receiver_center + Vector(0, 1, 0));
                
                testplane_1 = CreatePlane(receiver_center, tmp1, light_center);
                testplane_2 = CreatePlane(receiver_center, tmp2, light_center);
                
                break;
            case 1: // Y
                break;
            case 2: // Z
                break;
            case -1:
                // how to handle this case????
                break;
        }
    }

}