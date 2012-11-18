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
    
    // see [Laine, 06] fig 4.18
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

        float winding_counter = 0;
        for (rawedge_citer re = raw_edges.begin(); re != raw_edges.end(); re++) {
            Reference<RawEdge> raw_edge = *re;
            if (raw_edge->neighbour[0] != NULL && raw_edge->neighbour[1] != NULL)
                // double edge -> not @ side of the surface
                continue;
            
            Edge edge(raw_edge, raw_edge->neighbour[0] != NULL);
            list<Point> vertices = clampAndGetVertices(edge);
            
            list<Point>::iterator vertices_end = --vertices.end();
            for (list<Point>::iterator vertex = vertices.begin(); vertex != vertices_end;) {
                Point current = *vertex;
                vertex++;
                Point next = *vertex;

                float d0 = current * testplane_1, d1 = next * testplane_1;
                
                if (sign(d0) == sign(d1)) continue;
                
                float f0 = current * testplane_2, f1 = next * testplane_2;
                
                if ((f0 < 0) && (f1 < 0)) continue;
                if ((d0 < d1) == (d0 * f1 > d1 * f0)) continue;
                
                float adjust = 1.f;
                if ((d0 == 0) || (d1 == 0)) adjust = .5f;
                if (d0 > d1) adjust = -adjust;
                winding_counter += adjust;
            }
        }
        
        return winding_counter != 0;
    }
    
    bool ShaftGeometry::intersects(const Reference<Triangle> &triangle, const Mesh &mesh) const {
        const Point *point[3];
        
        for (int i = 0; i < 3; i++)
            point[i] = &mesh.getPoint(triangle->operator[](i));
        
        // if neither of the point is inside the bounding box of the shaft, the triangle cannot intersect
        if (!bbox.Inside(*point[0]) && !bbox.Inside(*point[1]) && !bbox.Inside(*point[2]))
            return false;
        
        // if every point lies to the wrong side of one of the planes, the triangle cannot intersect
        for (plane_citer plane = planes.begin(); plane != planes.end(); plane++) {
            if (((*point[0] * *plane) < 0) && ((*point[1] * *plane) < 0) && ((*point[2] * *plane) < 0))
                return false;
        }
        
        {
            // check if any of the triangle points lies on the right side over all the planes,
            // if so: the triangle intersects
            bool check[3] = { true, true, true };
            for (plane_citer plane = planes.begin(); plane != planes.end(); plane++) {
                for (int i = 0; i < 3; i++) {
                    check[i] &= (*point[i] * *plane) > 0;
                }
            }
            if (check[0] || check[1] || check[2])
                return true;
        }
        
        // TODO handle the case where the triangle overlaps the shaft, but no points lie in the shaft
        Warning("TODO implement -- triangle may overlap with shaft, but no points lie inside");
        // how?
        // clip away parts of the triangle that lie outside of a plane, then check if anything is left
        
        return true;
    }
    
    list<Point> ShaftGeometry::clampAndGetVertices(const Edge & edge) const {
        float min, max;
        list<Point> result;
        
        switch (main_axis) {
            case -1: // overlap, yay us
                result.push_back(Point(edge.getVertex(0).point));
                result.push_back(Point(edge.getVertex(1).point));
                
                break;
            case 0: // X
            case 1: // Y
            case 2: // Z
                if (receiver_bbox.getCenter()[main_axis] < light_bbox.getCenter()[main_axis]) {
                    min = receiver_bbox.getxyz()[main_axis];
                    max = light_bbox.getXYZ()[main_axis];
                } else {
                    min = light_bbox.getxyz()[main_axis];
                    max = receiver_bbox.getXYZ()[main_axis];
                }
                
                const Point *p[2];
                p[0] = &edge.getVertex(0).point;
                p[1] = &edge.getVertex(1).point;
                for (int i = 0; i < 2; i++) {
                    const Point &point = *p[i];
                    const Point &other = *p[1-i];
                    
                    const Vector diff = (other - point);
                    
                    if (point[main_axis] < min) {
                        Point new_point = point;
                        new_point += diff * (min - point[main_axis]) / diff[main_axis];
                        result.push_back(new_point);
                    } else if (point[main_axis] > max) {
                        Point new_point = point;
                        new_point += diff * (max - point[main_axis]) / diff[main_axis];
                        result.push_back(new_point);
                    } else {
                        result.push_back(point);
                    }
                }
        }
        
        // TODO see fig 4.11?
        
        return result;
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

    
    ShaftGeometry::ShaftGeometry(Reference<ElementTreeNode> &receiver, Reference<ElementTreeNode> &light)
    : receiver_bbox(receiver->bounding_box), light_bbox(light->bounding_box), bbox(BBox::Union(receiver_bbox, light_bbox)) {
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
                Warning("TODO implement -- how to create the bounding planes for a shaft between overlapping nodes?");
                                 
                // by not settings test_plane{1,2}, we assure that all tests fail
                // (these tests shouldn't be run in the first place
                break;
        }
    }
    
    // see [Laine, 06] fig 4.19
    Shaft::Shaft(Reference<ElementTreeNode> &receiver, Reference<ElementTreeNode> &light, Reference<ElementTreeNode> &split, Shaft &parent)
    : receiverNode(receiver), lightNode(light), geometry(receiver, light) {
        // copy main_axis from the parent?
        
        for (surface_iter surf = parent.surfaces.begin(); surf != parent.surfaces.end(); surf++) {
            
        }
    }

}