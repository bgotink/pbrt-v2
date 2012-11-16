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
    
    typedef list<Reference<RawEdge> > rawedge_clist;
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

        Reference<RawEdge> raw_edge;
        for (rawedge_citer re = raw_edges.begin(); re != raw_edges.end(); re++) {
            raw_edge = *re;
            if (raw_edge->neighbour[0] != NULL && raw_edge->neighbour[1] != NULL)
                // double edge -> not @ side of the surface
                continue;
            
            Edge edge(raw_edge, raw_edge->neighbour[0] != NULL);
            //            list<Vector3i> vertices = clampAndGetVertices(edge);
        }
        
        // TODO
        return false;
    }
    
    /*
     * The planes are oriented so that a point in the shaft lies on the positive
     * side of each of the planes.
     */
    
    #define CREATE_PLANES_X(first, last) \
        if (first.pMax.y > last.pMax.y) \
            planes.push_back(CreatePlane(first.getXYZ(), first.getXYz(), last.getXYZ())); \
        else \
            planes.push_back(CreatePlane(first.getxYZ(), first.getxYz(), last.getxYZ())); \
        if (first.pMax.z > last.pMax.z) \
            planes.push_back(CreatePlane(first.getXyZ(), first.getXYZ(), last.getXYZ())); \
        else \
            planes.push_back(CreatePlane(first.getxyZ(), first.getxYZ(), last.getxYZ())); \
        if (first.pMin.y < last.pMin.y) \
            planes.push_back(CreatePlane(first.getXyz(), first.getXyZ(), last.getXyz())); \
        else \
            planes.push_back(CreatePlane(first.getxyz(), first.getxyZ(), last.getxyz())); \
        if (first.pMin.z < last.pMin.z) \
            planes.push_back(CreatePlane(first.getXYz(), first.getXyz(), last.getXyz())); \
        else \
            planes.push_back(CreatePlane(first.getxYz(), first.getxyz(), last.getxyz()));

    #define CREATE_PLANES_Y(first, last) \
        if (first.pMax.z > last.pMax.z) \
            planes.push_back(CreatePlane(first.getXYZ(), first.getxYZ(), last.getXYZ())); \
        else \
            planes.push_back(CreatePlane(first.getXyZ(), first.getxyZ(), last.getXyZ())); \
        if (first.pMax.x > last.pMax.x) \
            planes.push_back(CreatePlane(first.getXYz(), first.getXYZ(), last.getXYZ())); \
        else \
            planes.push_back(CreatePlane(first.getXyz(), first.getXyZ(), last.getXyZ())); \
        if (first.pMin.z < last.pMin.z) \
            planes.push_back(CreatePlane(first.getxYz(), first.getXYz(), last.getxYz())); \
        else \
            planes.push_back(CreatePlane(first.getxyz(), first.getXyz(), last.getxyz())); \
        if (first.pMin.x < last.pMin.x) \
            planes.push_back(CreatePlane(first.getxYZ(), first.getxYz(), last.getxYz())); \
        else \
            planes.push_back(CreatePlane(first.getxyZ(), first.getxyz(), last.getxyz()));

    #define CREATE_PLANES_Z(first, last) \
        if (first.pMax.x > last.pMax.x) \
            planes.push_back(CreatePlane(first.getXyZ(), first.getXYZ(), last.getXYZ())); \
        else \
            planes.push_back(CreatePlane(first.getXyz(), first.getXYz(), last.getXYz())); \
        if (first.pMax.y > last.pMax.y) \
            planes.push_back(CreatePlane(first.getXYZ(), first.getxYZ(), last.getXYZ())); \
        else \
            planes.push_back(CreatePlane(first.getXYz(), first.getxYz(), last.getXYz())); \
        if (first.pMin.x < last.pMin.x) \
            planes.push_back(CreatePlane(first.getxYZ(), first.getxyZ(), last.getxyZ())); \
        else \
            planes.push_back(CreatePlane(first.getxYz(), first.getxyZ(), last.getxyz())); \
        if (first.pMin.y < last.pMin.y) \
            planes.push_back(CreatePlane(first.getxyZ(), first.getXyZ(), last.getxyZ())); \
        else \
            planes.push_back(CreatePlane(first.getxyz(), first.getXyz(), last.getxyz()));

    
    ShaftGeometry::ShaftGeometry(ElementTreeNode &receiver, ElementTreeNode &light) {
        const BBox &receiver_bbox = receiver.bounding_box, &light_bbox = light.bounding_box;
        const Point receiver_center = receiver_bbox.getCenter(),
                    light_center = light_bbox.getCenter();
        
        if (receiver_bbox.Overlaps(light_bbox)) {
            // special case, we won't really use this shaft
            main_axis = -1;
        } else {
            const ::Vector v = (receiver_center - light_center).abs();
            
            if (v.x > v.y && v.x > v.z)
                main_axis = 0;
            else {
                if (v.y > v.z)
                    main_axis = 1;
                else
                    main_axis = 2;
            }
        }
        
        switch (main_axis) {
            case 0: // X
            {
                if (receiver_center.x < light_center.x) {
                    CREATE_PLANES_X(receiver_bbox, light_bbox);
                } else {
                    CREATE_PLANES_X(light_bbox, receiver_bbox);
                }
                
                Point tmp1 = (receiver_center + Vector(0, 0, 1)), tmp2 = (receiver_center + Vector(0, 1, 0));
                
                testplane_1 = CreatePlane(receiver_center, tmp1, light_center);
                testplane_2 = CreatePlane(receiver_center, tmp2, light_center);
            }
                break;
            case 1: // Y
            {
                if (receiver_center.y < light_center.y) {
                    CREATE_PLANES_Y(receiver_bbox, light_bbox);
                } else {
                    CREATE_PLANES_Y(light_bbox, receiver_bbox);
                }
                
                Point tmp1 = (receiver_center + Vector(1, 0, 0)), tmp2 = (receiver_center + Vector(0, 0, 1));
                
                testplane_1 = CreatePlane(receiver_center, tmp1, light_center);
                testplane_2 = CreatePlane(receiver_center, tmp2, light_center);
            }
                break;
            case 2: // Z
            {
                if (receiver_center.z < light_center.z) {
                    CREATE_PLANES_Z(receiver_bbox, light_bbox);
                } else {
                    CREATE_PLANES_Z(light_bbox, receiver_bbox);
                }
                
                Point tmp1 = (receiver_center + Vector(1, 0, 0)), tmp2 = (receiver_center + Vector(0, 1, 0));
                
                testplane_1 = CreatePlane(receiver_center, tmp1, light_center);
                testplane_2 = CreatePlane(receiver_center, tmp2, light_center);
            }
                break;
            case -1:
                // how to handle this case? <-- FIXME
                                 
                // by not settings test_plane{1,2}, we assure that all tests fail
                // (these tests shouldn't be run in the first place
                break;
        }
    }

}