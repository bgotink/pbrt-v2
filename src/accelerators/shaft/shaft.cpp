//
//  shaft.cpp
//  pbrt
//
//  Created by Bram Gotink on 14/11/12.
//
//

#include "shaft.h"
#include <float.h>

#include "pbrt.h"
#include "log.h"
#include "intersection.h"

#include <set>
#include <map>

using namespace std;

namespace shaft {

    bool ShaftGeometry::canBeBlockedBy(const BBox &bounding_box) const {
        return false;
    }
    
    typedef list<Reference<RawEdge> > rawedge_list;
    typedef rawedge_list::iterator rawedge_iter;
    typedef rawedge_list::const_iterator rawedge_citer;
    
    // see [Laine, 06] fig 4.18
    bool ShaftGeometry::blockedBy(const Reference<shaft::Surface> &surface) const {
        // main_axis = -1 if the two endpoints overlap,
        // in which case we ALWAYS split the shaft
        if (main_axis == -1) return false;
        
        if (!canBeBlockedBy(surface->getBoundingBox()))
            return false;
        
        const rawedge_list raw_edges = surface->getRawEdges();
        
        for (rawedge_citer re = raw_edges.begin(); re != raw_edges.end(); re++) {
            const RawEdge &edge = **re;
            
            if ((!edge.neighbour[0] || !edge.neighbour[1]) && edge.is_inside)
                return false; // single edge inside shaft
        }

        float winding_counter = 0.f;
        for (rawedge_citer re = raw_edges.begin(); re != raw_edges.end(); re++) {
            Reference<RawEdge> raw_edge = *re;
            if (raw_edge->neighbour[0]&& raw_edge->neighbour[1])
                // double edge -> not @ side of the surface
                continue;
            
            Edge edge(raw_edge, !!raw_edge->neighbour[0]);
            list<Point> vertices = clampAndGetVertices(edge);
            
            list<Point>::iterator vertices_end = --vertices.end();
            for (list<Point>::iterator vertex = vertices.begin(); vertex != vertices_end;) {
                Point current = *vertex;
                vertex++;
                Point next = *vertex;

                float d0 = current * testplane_1, d1 = next * testplane_1;
                
                if (sign(d0) == sign(d1)) continue;
                
                float f0 = current * testplane_2, f1 = next * testplane_2;
                
                if ((f0 < 0.f) && (f1 < 0.f)) continue;
                if ((d0 < d1) == (d0 * f1 > d1 * f0)) continue;
                
                float adjust = 1.f;
                if ((d0 == 0.f) || (d1 == 0.f)) adjust = .5f;
                if (d0 > d1) adjust = -adjust;
                winding_counter += adjust;
            }
        }
        
        return winding_counter != 0.f;
    }
    
    bool ShaftGeometry::intersects(const Reference<Triangle> &triangle, const Mesh &mesh) const {
        const Point *point[3];
        
        for (unsigned int i = 0; i < 3; i++)
            point[i] = &mesh.getPoint((*triangle)[i]);
        
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
                    check[i] &= (*point[i] * *plane) >= 0;
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
    
    bool ShaftGeometry::intersectsLine(Point one, Point two) const {        
        float f, f2;

        for (plane_citer plane = planes.begin(); plane != planes.end(); plane++) {
            f = one * *plane;
            f2 = two * *plane;
            
            if (sign(f) == sign(f2)) {
                if (f < 0)
                    // both points are outside of the plane
                    return false;
                else
                    continue;
            }
            
            // using http://www.softsurfer.com/Archive/algorithm_0104/algorithm_0104B.htm#intersect3D_SegPlane()
            
            Vector u = two - one;
            Vector w = one - getPointOnPlane(*plane);
            Vector normal = getNormal(*plane);
            
            float d = normal * u;
            float n = -normal * w;
            
            if (fabs(d) < 16 * FLT_EPSILON)
                // line segment || plane
                // -> skip this plane
                continue;
            
            Point interstection = one + (n / d) * u;
            
            // replace the outside point with the intersection
            if (f < 0)
                one = interstection;
            else
                two = interstection;
        }
        
        return !one.epsilonEquals(two);
    }
    
    bool ShaftGeometry::Intersect(const Ray &ray, Intersection *isect) const {
        Intersection intersect;
        float distance = INFINITY;
        
        for (plane_citer plane = planes.begin(); plane != planes.end(); plane++) {
            float nominator = ray.d * *plane;
            if (nominator == 0.f) {
                // plane || ray
                continue;
            }
            
            float t = - (ray.o * *plane) / nominator;
            if (t < 0 || t > distance)
                continue;
            
            Point intersectPoint(ray(t));
            
            {
                bool exit = false;
                for (plane_citer p = planes.begin(); p != planes.end(); p++) {
                    if (p == plane) continue;
                    if (*p * intersectPoint < 0) {
                        exit = true;
                        break;
                    }
                }
                if (exit) continue;
            }
            
            intersect.dg.p = intersectPoint;
            intersect.dg.nn = Normal(plane->x, plane->y, plane->z);
            if (intersect.dg.nn * ray.d < 0.f)
                intersect.dg.nn = -intersect.dg.nn;
            intersect.rayEpsilon = 1e-3 * t;
            distance = t;
        }
        
        if (!isinf(distance)) {
            if (isect) *isect = intersect;
            return true;
        }
        return false;
    }
    
    bool ShaftGeometry::IntersectP(const Ray &ray) const {
        return Intersect(ray);
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
                for (unsigned int i = 0; i < 2; i++) {
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
                Warning("TODO stub: using a bounding volume instead of shaft");
                
                BBox shaft_bbox = receiver_bbox.Union(light_bbox);
                
                planes.push_back(Vector4f(1, 0, 0, -shaft_bbox.pMin.x));
                planes.push_back(Vector4f(0, 1, 0, -shaft_bbox.pMin.y));
                planes.push_back(Vector4f(0, 0, 1, -shaft_bbox.pMin.z));
                
                planes.push_back(Vector4f(-1, 0, 0, shaft_bbox.pMax.x));
                planes.push_back(Vector4f(0, -1, 0, shaft_bbox.pMax.y));
                planes.push_back(Vector4f(0, 0, -1, shaft_bbox.pMax.z));
                                 
                // by not settings test_plane{1,2}, we assure that all tests fail
                // (these tests shouldn't be run in the first place
                break;
        }
    }
    
    Reference<Patch> Shaft::createClippedPatch(const Reference<Triangle> &triangle) const {
        Patch &result = *(new Patch);
        Patch::edge_list &edges = result.edges;
        
        uint32_t pidx[3] = { triangle->getPoint(0), triangle->getPoint(1), triangle->getPoint(2) };
        const Mesh &mesh = getMesh();
        
        for (unsigned int i = 0; i < 3; i++) {
            int next = (i == 2) ? 0 : i+1;
            Edge &edge = *new Edge(pidx[i], pidx[next], mesh);
            
            edge.setOwner(&result);
            
            RawEdge::idtype edge_id = pidx[i];
            if (pidx[i] > pidx[next]) {
                edge_id = (edge_id << 32) + pidx[next];
            } else {
                edge_id = edge_id + (uint64_t(pidx[next]) << 32);
            }
            
            edges.push_back(Reference<Edge>(&edge));
        }
        // FIXME implement real clipping here
        
        return Reference<Patch>(&result);
    }
    
    // See [Laine, 06] fig 4.21
    void Shaft::classifyEdges(Reference<Surface> &surface) const {
        rawedge_list rawEdges = surface->getRawEdges();
        for (rawedge_iter re = rawEdges.begin(); re != rawEdges.end(); re++) {
            if (!(*re)->is_inside) continue;
            (*re)->is_inside = geometry.intersectsLine((*re)->getPoint(0), (*re)->getPoint(1));
        }
    }
    
    // See [Laine, 06] fig 4.22
    void Shaft::updatePatchFacings(Reference<Surface> &surface) const {
        const BBox &receiverBox = receiverNode->bounding_box;
        const BBox &lightBox = lightNode->bounding_box;
        
        for (Surface::patch_iter patch = surface->patches.begin(); patch != surface->patches.end(); patch++) {
            if ((*patch)->facing != INCONSISTENT)
                continue;
            
            const Reference<Triangle> &t = getTriangle((*patch)->mesh_triangle);
            Vector4f pl = CreatePlane(getPoint(t->getPoint(0)),
                                      getPoint(t->getPoint(1)),
                                      getPoint(t->getPoint(2)));
            
            BBoxPlaneResult receiver_result = pl * receiverBox;
            BBoxPlaneResult light_result = pl * lightBox;
            
            if (receiver_result == FRONT)
                (*patch)->facing = TOWARDS_RECEIVER;
            else if (receiver_result == BACK)
                (*patch)->facing = TOWARDS_LIGHT;
            else if (light_result == FRONT)
                (*patch)->facing = TOWARDS_LIGHT;
            else if (light_result == BACK)
                (*patch)->facing = TOWARDS_RECEIVER;
        }
    }
    
    // see [Laine, 06] fig 4.20
    Reference<Surface> Shaft::constructTriangleSurface(nblist &triangles) {
        Surface &new_surface = *(new Surface);
        
        map<RawEdge::idtype, Reference<Edge> > edge_map;
        Mesh &mesh = getMesh();
        
        for (nbiter tidx = triangles.begin(); tidx != triangles.end(); tidx++) {
            const Reference<Triangle> &t = getTriangle(*tidx);
            
            // Is the triangle in the shaft?
            if (!intersects(t)) continue;
            
            // If the triangle is in any of the nodes the shaft connects,
            // don't take it into account (cf [Laine, 06] section 4.2.2)
            if (Intersects(receiverNode->bounding_box, t, mesh)) continue;
            if (Intersects(lightNode->bounding_box, t, mesh)) continue;
            
            Reference<Patch> p = createClippedPatch(t);
            p->facing = INCONSISTENT;
            p->mesh_triangle = *tidx;
            
            for (Patch::edge_iter e = p->edges.begin(); e != p->edges.end(); e++) {
                RawEdge::idtype elabel = (*e)->getRawEdgeLabel();
                
                if (edge_map.count(elabel) == 0) {
                    edge_map[elabel] = *e;
                } else {
                    Reference<Edge> &other = edge_map[elabel];
                    *e = other->flip();
                    
                    (*e)->setOwner(&*p);
                }
            }
            
            new_surface.patches.push_back(p);
        }
        
        return Reference<Surface>(&new_surface);
    }
    
    // see [Laine, 06] fig 4.19
    Shaft::Shaft(Reference<ElementTreeNode> &receiver, Reference<ElementTreeNode> &light, Reference<ElementTreeNode> &split, Shaft &parent)
    : receiverNode(receiver), lightNode(light), geometry(receiver, light) {
        // copy main_axis from the parent?
        
        for (surface_iter surf = parent.surfaces.begin(); surf != parent.surfaces.end(); surf++) {
            surfaces.push_back((*surf)->clone());
        }
        
        surfaces.push_back(constructTriangleSurface(split->gone_triangles));
        surface_list tmp_surfaces;

        Reference<Surface> surf;
        for (surface_iter s = surfaces.begin(); s != surfaces.end(); s++) {
            surf = *s;
            classifyEdges(surf);
            updatePatchFacings(surf);
            surf->mergePatches();
            surf->simplifyPatches();
            surf->splitSurface(tmp_surfaces);
        }
        
        surfaces.clear();
        
        for (surface_iter surf = tmp_surfaces.begin(); surf != tmp_surfaces.end(); surf++) {
            computeLooseEdges(*surf);
        }
        
        combineSurfaces(tmp_surfaces);
        
        for (surface_iter surf = surfaces.begin(); surf != surfaces.end(); surf++) {
            (*surf)->computeBoundingBox();
        }
        
        Mesh &mesh = getMesh();
        for (nblciter tris = parent.triangles.begin(); tris != parent.triangles.end(); tris++) {
            if (geometry.intersects(mesh.getTriangle(*tris), mesh))
                triangles.push_back(*tris);
        }
        for (nbciter tris = split->gone_triangles.begin(); tris != split->gone_triangles.end(); tris++) {
            if (geometry.intersects(mesh.getTriangle(*tris), mesh))
                triangles.push_back(*tris);
        }
            
#ifdef SHAFT_LOG
        depth = parent.depth + 1;
#endif
    }
    
    // cf. [Laine, 06] fig 4.31
    Shaft::Shaft(Reference<ElementTreeNode> &receiver, Reference<ElementTreeNode> &light)
    : receiverNode(receiver), lightNode(light), geometry(receiver, light) {
        Mesh &mesh = getMesh();
        int nbTris = mesh.triangles.size();
        for (int i = 0; i < nbTris; i++) {
            triangles.push_back(i);
        }
        
        nblist triangles_vector(nbTris);
        triangles_vector.insert(triangles_vector.begin(), triangles.begin(), triangles.end());
        
        Reference<Surface> surf = constructTriangleSurface(triangles_vector);
        
        classifyEdges(surf);
        updatePatchFacings(surf);
        surf->mergePatches();
        surf->simplifyPatches();
        surf->splitSurface(surfaces);
        
        for (surface_iter s = surfaces.begin(); s != surfaces.end(); s++) {
            (*s)->computeBoundingBox();
        }
        
#ifdef SHAFT_LOG
        depth = 0;
#endif
    }
    
    // cf [Laine, 06] fig 4.26
    void Shaft::computeLooseEdges(Reference<Surface> &surface) {
        Mesh &mesh = getMesh();
        surface->loose_edges.clear();
        
        for (Surface::patch_iter patch = surface->patches.begin(); patch != surface->patches.end(); patch++) {
            Reference<Patch> p = *patch;
            for (Patch::edge_iter e = p->edges.begin(); e != p->edges.end(); e++) {
                Reference<Edge> edge = *e;
                RawEdge::idtype edge_label = edge->raw_edge->mesh_edge;
                if (mesh.is_double_edge[edge_label] == false) continue;
                if (edge->raw_edge->is_inside && !edge->getNeighbour()) {
                    surface->loose_edges.push_back(edge_label);
                }
            }
        }
    }
    
    typedef set<Reference<Surface> > surf_set;
    typedef surf_set::iterator surf_siter;
    
    typedef map<RawEdge::idtype, surf_set> union_find;
    typedef union_find::iterator uf_iter;
    
    // cf [Laine, 06] fig 4.27
    void Shaft::combineSurfaces(surface_list &input_list) {
        union_find combine_union;
        
        for (surface_iter s = input_list.begin(); s != input_list.end(); s++) {
            Reference<Surface> surf = *s;
            for (Surface::nbiter el = surf->loose_edges.begin(); el != surf->loose_edges.end(); el++) {
                RawEdge::idtype elabel = *el;
                if (combine_union.count(elabel)) {
                    // label seen already?
                    combine_union[elabel].insert(surf);
                } else {
                    combine_union[elabel] = set<Reference<Surface> >();
                    combine_union[elabel].insert(surf);
                }
            }
        }
        
        for (uf_iter s = combine_union.begin(); s != combine_union.end(); s++) {
            surf_set &set_surfaces = s->second;
            if (set_surfaces.size() == 1) {
                surfaces.push_back(* set_surfaces.begin());
            } else {
                surfaces.push_back(Surface::constructCombinedSurface(set_surfaces));
            }
        }
    }
    
    bool Shaft::GeomIntersectP(const Ray &ray) const {
        ShaftStartIntersectP();
        return geometry.IntersectP(ray);
    }
    
    bool Shaft::IntersectP(const Ray &ray) const {
        const Mesh &mesh = getMesh();

        ShaftStartIntersectP();
        
        // check all triangles
        for (nblciter t = triangles.begin(); t != triangles.end(); t++) {
            ShaftIntersectTest();
            if (IntersectsTriangle(mesh.getTriangle(*t), mesh, ray))
                return true;
        }
        ShaftNotIntersected();
        
        return receiverNode->IntersectP(ray);
    }

}