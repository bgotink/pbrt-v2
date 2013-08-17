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
#include "intersection.h"
#include "geometry.h"

#include "log.h"
#include "break.h"

#include <set>
#include <map>

#include <sstream>

//#define SHAFT_DO_CLIPPING

using namespace std;
using namespace shaft::vis;

namespace shaft {

    bool ShaftGeometry::intersects(const Reference<Triangle> &triangle, const Mesh &mesh) const {
        const Point *point[3];
        
        for (unsigned int i = 0; i < 3; i++)
            point[i] = &triangle->getPoint(i);
        
        // if every point lies to the wrong side of one of the planes, the triangle cannot intersect
        for (plane_citer plane = planes.begin(), end = planes.end(); plane != end; plane++) {
            if (((*point[0] * *plane) < 0) && ((*point[1] * *plane) < 0) && ((*point[2] * *plane) < 0))
                return false;
        }
        
        {
            // check if any of the triangle points lies on the right side off all the planes,
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
        // Warning("TODO implement -- triangle may overlap with shaft, but no points lie inside");
        // how?
        // clip away parts of the triangle that lie outside of a plane, then check if anything is left

#if !defined(SHAFT_DO_CLIPPING)
        return true;
    }
#else // defined(SHAFT_DO_CLIPPING)
        {
        typedef std::list<Point> plist;
        typedef plist::const_iterator pciter;

        plist vertices;
        vertices.push_back(*point[0]);
        vertices.push_back(*point[1]);
        vertices.push_back(*point[2]);

        // Sutherland-Hodgman
        // https://en.wikipedia.org/wiki/Sutherland%E2%80%93Hodgman_algorithm
        plane_citer plane_end = planes.end();
        Vector4f p;
        for (plane_citer plane = planes.begin(); plane != plane_end; plane++) {
            plist tmpPoints;
            tmpPoints.swap(vertices);

            p = *plane;

            pciter point_end = tmpPoints.end();
            Point s = tmpPoints.back();
            for (pciter point = tmpPoints.begin(); point != point_end; point++) {
                Point e = *point;

                if ((e * p) >= 0) {
                    if ((s * p) < 0) {
                        vertices.push_back(getIntersection(p, e, s));
                    }
                    vertices.push_back(e);
                } else if ((s * p) >= 0) {
                    vertices.push_back(getIntersection(p, e, s));
                }

                s = e;
            }
        }

        // check if surface of the "polygon" is empty
        // see http://geomalgorithms.com/a01-_area.html
        // 2*area = n*sum(V_i ^ V_i+1)
        //  n = normal, V_i = vertex i
        // -> area != 0 <=> n * sum(V_i ^ V_i+1) != 0

        pciter point_end = vertices.end();
        Vector A;
        Vector n = (*point[1] - *point[0]) ^ (*point[2] - *point[0]);

        Vector previous = Vector(vertices.back()), current;
        for (pciter point = vertices.begin(); point != point_end; point++) {
            current = Vector(*point);
            A += (current ^ previous);
            previous = current;
        }

        return fabs(A * n) > 0;
        }
    }
#endif
        
    bool ShaftGeometry::Intersect(const Ray &ray, Intersection *isect) const {
        Intersection intersect;
        float distance = INFINITY;
        
        for (plane_citer plane = planes.begin(), end = planes.end(); plane != end; plane++) {
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
                for (plane_citer p = planes.begin(), pend = planes.end(); p != pend; p++) {
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
            planes.push_back(CreatePlane(first.getxYz(), first.getxyz(), last.getxyz())); \
        planes.push_back(Vector4f(1, 0, 0, -first.pMin.x)); \
        planes.push_back(Vector4f(-1, 0, 0, last.pMax.x));

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
            planes.push_back(CreatePlane(first.getxyZ(), first.getxyz(), last.getxyz())); \
        planes.push_back(Vector4f(0, 1, 0, -first.pMin.y)); \
        planes.push_back(Vector4f(0, -1, 0, last.pMax.y));


    #define CREATE_PLANES_Z(first, last) \
        if (first.pMax.x > last.pMax.x) \
            planes.push_back(CreatePlane(first.getXYZ(), first.getXyZ(), last.getXYZ())); \
        else \
            planes.push_back(CreatePlane(first.getXYz(), first.getXyz(), last.getXYz())); \
        if (first.pMax.y > last.pMax.y) \
            planes.push_back(CreatePlane(first.getxYZ(), first.getXYZ(), last.getXYZ())); \
        else \
            planes.push_back(CreatePlane(first.getxYz(), first.getXYz(), last.getXYz())); \
        if (first.pMin.x < last.pMin.x) \
            planes.push_back(CreatePlane(first.getxyZ(), first.getxYZ(), last.getxyZ())); \
        else \
            planes.push_back(CreatePlane(first.getxyz(), first.getxYz(), last.getxyz())); \
        if (first.pMin.y < last.pMin.y) \
            planes.push_back(CreatePlane(first.getXyZ(), first.getxyZ(), last.getXyZ())); \
        else \
            planes.push_back(CreatePlane(first.getXyz(), first.getxyz(), last.getxyz())); \
        planes.push_back(Vector4f(0, 0, 1, -first.pMin.z)); \
        planes.push_back(Vector4f(0, 0, -1, last.pMax.z));



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
            }
                break;
            case 1: // Y
            {
                if (receiver_center.y < light_center.y) {
                    CREATE_PLANES_Y(receiver_bbox, light_bbox);
                } else {
                    CREATE_PLANES_Y(light_bbox, receiver_bbox);
                }
            }
                break;
            case 2: // Z
            {
                if (receiver_center.z < light_center.z) {
                    CREATE_PLANES_Z(receiver_bbox, light_bbox);
                } else {
                    CREATE_PLANES_Z(light_bbox, receiver_bbox);
                }
            }
                break;
            case -1:
                // Warning("TODO stub: using a bounding volume instead of shaft");
                
                BBox shaft_bbox = receiver_bbox.Union(light_bbox);
                
                planes.push_back(Vector4f(1, 0, 0, -shaft_bbox.pMin.x));
                planes.push_back(Vector4f(0, 1, 0, -shaft_bbox.pMin.y));
                planes.push_back(Vector4f(0, 0, 1, -shaft_bbox.pMin.z));
                
                planes.push_back(Vector4f(-1, 0, 0, shaft_bbox.pMax.x));
                planes.push_back(Vector4f(0, -1, 0, shaft_bbox.pMax.y));
                planes.push_back(Vector4f(0, 0, -1, shaft_bbox.pMax.z));
                                 
                break;
        }
    }
    
#if defined(SHAFT_LOG) && defined(SHAFT_SHOW_LEAFS)
    uint32_t Shaft::NEXT_UID = 0;
#endif
    
    // see [Laine, 06] fig 4.19
    Shaft::Shaft(Reference<ElementTreeNode> &receiver, Reference<ElementTreeNode> &light, Reference<ElementTreeNode> &split, Shaft &parent)
    :
#if defined(SHAFT_LOG) && defined(SHAFT_SHOW_LEAFS)
    uid(++NEXT_UID),
#endif
    receiverNode(receiver), lightNode(light), geometry(receiver, light), vis(NULL) {
        // copy main_axis from the parent?
        
        nbset new_triangles;
        
        Mesh &mesh = getMesh();
        for (nblciter tris = parent.triangles.begin(), trisend = parent.triangles.end();
                        tris != trisend; tris++) {
            if (geometry.intersects(mesh.getTriangle(*tris), mesh))
                new_triangles.insert(*tris);
        }
        for (nbciter tris = split->gone_triangles.begin(), trisend = split->gone_triangles.end();
                        tris != trisend; tris++) {
            if (geometry.intersects(mesh.getTriangle(*tris), mesh))
                new_triangles.insert(*tris);
        }
        
        triangles.insert(triangles.begin(), new_triangles.begin(), new_triangles.end());
        filterTriangles();
        
#if defined(SHAFT_LOG) && defined(SHAFT_SHOW_DEPTHS)
        depth = parent.depth + 1;
        Info("Shaft depth: %u", depth);
#endif
    }
    
    Shaft::~Shaft() {
        if (vis) delete vis;
    }
    
    void Shaft::filterTriangles() {
        BBox &receiverBox = receiverNode->bounding_box,
             &lightBox = lightNode->bounding_box;
        Mesh &mesh = getMesh();
        nbliter end = triangles.end();
        Reference<Triangle> triangle;
        for (nblciter tris = triangles.begin(); tris != end; tris++) {
            triangle = mesh.getTriangle(*tris);
            if (!Intersects(receiverBox, triangle)
                && !Intersects(lightBox, triangle))
                filtered_triangles.push_back(*tris);
        }
    }
    
    // cf. [Laine, 06] fig 4.31
    Shaft::Shaft(Reference<ElementTreeNode> &receiver, Reference<ElementTreeNode> &light)
    :
#if defined(SHAFT_LOG) && defined(SHAFT_SHOW_LEAFS)
    uid(++NEXT_UID),
#endif
    receiverNode(receiver), lightNode(light), geometry(receiver, light), vis(NULL) {
        Mesh &mesh = getMesh();
        int nbTris = mesh.triangles.size();
        for (int i = 0; i < nbTris; i++) {
            triangles.push_back(i);
        }
        
        filterTriangles();
        
#if defined(SHAFT_LOG) && defined(SHAFT_SHOW_DEPTHS)
        depth = 0;
#endif
    }
    
    bool Shaft::IntersectP(const Ray &ray) const {
        const Mesh &mesh = getMesh();

        // check all triangles
        for (nblciter t = triangles.begin(); t != triangles.end(); t++) {
            if (mesh.getTriangle(*t)->IntersectP(ray))
                return true;
        }
        
        return receiverNode->IntersectP(ray);
    }
    
    float Shaft::Visibility(const Ray &ray) const {
        Assert(vis);
#if defined(SHAFT_LOG)
#if defined(SHAFT_SHOW_DEPTHS)
        log::ShaftDepth(depth);
#endif // defined(SHAFT_SHOW_DEPTHS)
#if defined(SHAFT_SHOW_PRIMS)
        log::ShaftSetPrimCount(triangles.size());
#endif // defined(SHAFT_SHOW_PRIMS)
#if defined(SHAFT_SHOW_LEAFS)
        log::hitLeafId = uid;
#endif
#if defined(SHAFT_SHOW_EMPTY_LEAVES)
        log::ShaftSetLeafEmpty(receiverNode->empty());
#endif
#endif // defined(SHAFT_LOG)
#if defined(SHAFT_ENABLE_BREAK) && defined(SHAFT_BREAK_MANYPRIMS)
        if (triangles.size() > SHAFT_BREAK_MANYPRIMS_TRESHOLD) {
            breakp::manyprims();
        }
#endif // defined(SHAFT_ENABLE_BREAK) && defined(SHAFT_BREAK_MANYPRIMS)
        return vis->Visibility(ray);
    }
    
    Ray createMatterTestRay(const Ray &ray, const BBox &box) {
        Ray r = ray;
        float t = r.mint;
        float newT;
        
        if (ray.d.x < 0) {
            newT = (box.pMax.x - r.o.x) / r.d.x;
        } else {
            newT = (box.pMin.x - r.o.x) / r.d.x;
        }
        t = min(t, newT);
        
        if (ray.d.y < 0) {
            newT = (box.pMax.y - r.o.y) / r.d.y;
        } else {
            newT = (box.pMin.y - r.o.y) / r.d.y;
        }
        t = min(t, newT);
        
        if (ray.d.z < 0) {
            newT = (box.pMax.z - r.o.z) / r.d.z;
        } else {
            newT = (box.pMin.z - r.o.z) / r.d.z;
        }
        t = min(t, newT);
        
        r.o += t * r.d;
        r.mint = 0;
        r.maxt += t;
        
        return r;
    }
    
    void Shaft::initProbVis(bool useProbVis, RNG *rng, const string * const type,
                            uint32_t regular_size_prims, uint32_t regular_size_lights) {
        Assert(!useProbVis || rng != NULL);
        Assert(isLeaf());
        
        if (!useProbVis || filtered_triangles.empty()) {
            vis = createExactVisibilityCalculator(getMesh(), filtered_triangles, receiverNode, lightNode);
            return;
        }
        
        typedef std::map<uint32_t, unsigned int> countmap;
        
        VisibilityCalculator::nbllist &allTriangles = filtered_triangles;
        
        countmap counts;
        for (VisibilityCalculator::nblciter t = allTriangles.begin(); t != allTriangles.end(); t++) {
            counts[*t] = 0;
        }
        
        BBox &receiverBox = receiverNode->bounding_box;
        BBox &lightBox = lightNode->bounding_box;
        
        ElementTreeNode::nblist &receiverTris = receiverNode->inside_triangles;
        ElementTreeNode::nblist &lightTris = lightNode->inside_triangles;
        
        Point &receiverStart = receiverBox.pMin;
        Vector receiverStep = receiverBox.Extent() / (regular_size_prims - 1.);
        
        Point &lightStart = lightBox.pMin;
        Vector lightStep = lightBox.Extent() / (regular_size_lights - 1.);
        
        const nbllist &triangles = this->triangles;
        const Mesh &mesh = getMesh();
        uint countMatters = 0;
        
        for (uint32_t xr = 0; xr < regular_size_prims; xr++) {
        for (uint32_t yr = 0; yr < regular_size_prims; yr++) {
        for (uint32_t zr = 0; zr < regular_size_prims; zr++) {
        
            Point pr = receiverStart + Vector(receiverStep.x * xr, receiverStep.y * yr, receiverStep.z * zr);
            
            for (uint32_t xl = 0; xl < regular_size_lights; xl++) {
            for (uint32_t yl = 0; yl < regular_size_lights; yl++) {
            for (uint32_t zl = 0; zl < regular_size_lights; zl++) {

                Point pl = lightStart + Vector(lightStep.x * xl, lightStep.y * yl, lightStep.z * zl);
                
                Ray ray(pr, pl - pr, 0.f);
                log::addTestRay();
                
                unsigned int tris;
                bool rayMatters = false;
                
                // test whether the ray hits anything in the block
                Ray r = createMatterTestRay(ray, receiverBox);
                for (ElementTreeNode::nbciter t = receiverTris.begin(); !rayMatters && t != receiverTris.end(); t++) {
                    if (mesh.getTriangle(*t)->IntersectP(r))
                        rayMatters = true;
                }
                
                if (!rayMatters) continue;
                rayMatters = false;
                for (ElementTreeNode::nbciter t = lightTris.begin(); !rayMatters && t != lightTris.end(); t++) {
                    if (mesh.getTriangle(*t)->IntersectP(r))
                        rayMatters = true;
                }
                
                if (!rayMatters) continue;
                countMatters++;
                log::addUsefulTestRay();
                
                for (VisibilityCalculator::nblciter t = allTriangles.begin(); t != allTriangles.end(); t++) {
                    tris = *t;
                    if (mesh.getTriangle(*t)->IntersectP(ray)) {
                        counts[tris]++;
                    }
                }
            }}}
//            }
        }}}
//        }
        
        unsigned int max = 0, cur;
        Reference<Triangle> mostBlockingOccluder;
        for (nblciter t = triangles.begin(); t != triangles.end(); t++) {
            cur = *t;
            if (counts[cur] > max) {
                max = counts[cur];
                mostBlockingOccluder = mesh.getTriangle(cur);
            }
        }
        
        if (max == 0) {
//            Info("ProbVis initialisation found no blockers, falling back to exact visibility");
            vis = createExactVisibilityCalculator(mesh, filtered_triangles, receiverNode, lightNode);
            return;
        }
        
        float mostBlockingOccluderBlocking = static_cast<float>(max) / static_cast<float>(countMatters);
        
        {
            std::stringstream s;
            s << "Shaft: ";
            s << "[" << receiverStart << "->" << lightStart << "] -> ";
            s << "[" << receiverBox.pMax << "->" << lightBox.pMax << "]";
            string str = s.str();
            Info("%s", str.c_str());
        }
        Info("Most blocking occluder blocked %u / %d times", max, countMatters);
        Info("Most blocking occluder blocked %f %% times", 100. * mostBlockingOccluderBlocking);
//        Info("Most blocking occluder: (%d, %d, %d)", mostBlockingOccluder->getPoint(0), mostBlockingOccluder->getPoint(1), mostBlockingOccluder->getPoint(2));
        
        VisibilityCalculator::nbllist tmpTriangles;
        tmpTriangles.insert(tmpTriangles.begin(), triangles.begin(), triangles.end());
        tmpTriangles.insert(tmpTriangles.end(), receiverTris.begin(), receiverTris.end());
        tmpTriangles.insert(tmpTriangles.end(), lightTris.begin(), lightTris.end());
        
        vis = createProbabilisticVisibilityCalculator(*type, mesh, mostBlockingOccluder, tmpTriangles, *rng, mostBlockingOccluderBlocking);
    }

    uint64_t ShaftGeometry::memsize() const {
        return static_cast<uint64_t>(sizeof(ShaftGeometry))
                + planes.size() * sizeof(Vector4f);
    }

    uint64_t Shaft::memsize() const {
        return static_cast<uint64_t>(sizeof(Shaft))
                + geometry.memsize() - sizeof(ShaftGeometry) // sizeof(ShaftGeometry) gets counted double...
                + triangles.size() * sizeof(unsigned int)
                + filtered_triangles.size() * sizeof(unsigned int)
                + (vis ? vis->memsize() : 0);
    }

}
