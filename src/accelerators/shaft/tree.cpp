//
//  shaft.cpp
//  pbrt
//
//  Created by Bram Gotink on 14/11/12.
//
//

#include "tree.h"
#include "log.h"
#include "../../shapes/trianglemesh.h"
#include <sstream>

#define X 0
#define Y 1
#define Z 2

using namespace std;

namespace shaft {
    
/*======================== X-tests ========================*/
#define AXISTEST_X01(a, b, fa, fb)			   \
    p0 = a*v0[Y] - b*v0[Z];			       	   \
    p2 = a*v2[Y] - b*v2[Z];			       	   \
    if(p0<p2) {min=p0; max=p2;} else {min=p2; max=p0;} \
    rad = fa * boxhalfsize[Y] + fb * boxhalfsize[Z];   \
    if(min>rad || max<-rad) return 0;

#define AXISTEST_X2(a, b, fa, fb)			   \
    p0 = a*v0[Y] - b*v0[Z];			           \
    p1 = a*v1[Y] - b*v1[Z];			       	   \
    if(p0<p1) {min=p0; max=p1;} else {min=p1; max=p0;} \
    rad = fa * boxhalfsize[Y] + fb * boxhalfsize[Z];   \
    if(min>rad || max<-rad) return 0;

/*======================== Y-tests ========================*/
#define AXISTEST_Y02(a, b, fa, fb)			   \
    p0 = -a*v0[X] + b*v0[Z];		      	   \
    p2 = -a*v2[X] + b*v2[Z];	       	       	   \
    if(p0<p2) {min=p0; max=p2;} else {min=p2; max=p0;} \
    rad = fa * boxhalfsize[X] + fb * boxhalfsize[Z];   \
    if(min>rad || max<-rad) return 0;

#define AXISTEST_Y1(a, b, fa, fb)			   \
    p0 = -a*v0[X] + b*v0[Z];		      	   \
    p1 = -a*v1[X] + b*v1[Z];	     	       	   \
    if(p0<p1) {min=p0; max=p1;} else {min=p1; max=p0;} \
    rad = fa * boxhalfsize[X] + fb * boxhalfsize[Z];   \
    if(min>rad || max<-rad) return 0;

/*======================== Z-tests ========================*/
#define AXISTEST_Z12(a, b, fa, fb)			   \
    p1 = a*v1[X] - b*v1[Y];			           \
    p2 = a*v2[X] - b*v2[Y];			       	   \
    if(p2<p1) {min=p2; max=p1;} else {min=p1; max=p2;} \
    rad = fa * boxhalfsize[X] + fb * boxhalfsize[Y];   \
    if(min>rad || max<-rad) return 0;

#define AXISTEST_Z0(a, b, fa, fb)			   \
    p0 = a*v0[X] - b*v0[Y];				   \
    p1 = a*v1[X] - b*v1[Y];			           \
    if(p0<p1) {min=p0; max=p1;} else {min=p1; max=p0;} \
    rad = fa * boxhalfsize[X] + fb * boxhalfsize[Y];   \
    if(min>rad || max<-rad) return 0;

#define FINDMINMAX(a, b, c, min, max)           \
    min = ::min(a, ::min(b, c));                    \
    max = ::max(a, ::max(b, c));


static bool planeBoxOverlap(const BBox &box, const Point &point, const Vector &normal) {
    Vector vmin, vmax;
    float v;
    for(int q = 0; q < 3; q++) {
        v = point[q];
        
        if (normal[q] > 0.f) {
            vmin[q] = -box.pMax[q] - v;
            vmax[q] = box.pMax[q] - v;
        } else {
            vmin[q] = box.pMax[q] - v;
            vmax[q] = -box.pMax[q] - v;
        }
    }
    
    if (normal * vmin > 0.f)
        return false;
    return (normal * vmax >= 0.f);
}

// based on http://fileadmin.cs.lth.se/cs/Personal/Tomas_Akenine-Moller/code/tribox3.txt
bool Intersects(const BBox &box, const Reference<Triangle> &triangle, const Mesh &mesh) {
    Point boxcenter = box.getCenter();
    Vector boxhalfsize = box.getXYZ() - boxcenter;
    
    Vector v0, v1, v2;
    
    float min, max, p0, p1, p2, rad, fex, fey, fez;
    Vector normal, e0, e1, e2;
    
    v0 = mesh.getPoint((*triangle)[0]) - boxcenter;
    v1 = mesh.getPoint((*triangle)[1]) - boxcenter;
    v2 = mesh.getPoint((*triangle)[2]) - boxcenter;
    
    e0 = v1 - v0;
    e1 = v2 - v1;
    e2 = v0 - v2;
    
    fex = fabsf(e0[X]);
    fey = fabsf(e0[Y]);
    fez = fabsf(e0[Z]);
    AXISTEST_X01(e0[Z], e0[Y], fez, fey);
    AXISTEST_Y02(e0[Z], e0[X], fez, fex);
    AXISTEST_Z12(e0[Y], e0[X], fey, fex);
    
    fex = fabsf(e1[X]);
    fey = fabsf(e1[Y]);
    fez = fabsf(e1[Z]);
    AXISTEST_X01(e1[Z], e1[Y], fez, fey);
    AXISTEST_Y02(e1[Z], e1[X], fez, fex);
    AXISTEST_Z0(e1[Y], e1[X], fey, fex);
    
    fex = fabsf(e2[X]);
    fey = fabsf(e2[Y]);
    fez = fabsf(e2[Z]);
    AXISTEST_X2(e2[Z], e2[Y], fez, fey);
    AXISTEST_Y1(e2[Z], e2[X], fez, fex);
    AXISTEST_Z12(e2[Y], e2[X], fey, fex);
    
    FINDMINMAX(v0[X],v1[X],v2[X],min,max);
    if(min>boxhalfsize[X] || max<-boxhalfsize[X]) return false;
    
    FINDMINMAX(v0[Y],v1[Y],v2[Y],min,max);
    if(min>boxhalfsize[Y] || max<-boxhalfsize[Y]) return false;
    
    FINDMINMAX(v0[Z],v1[Z],v2[Z],min,max);
    if(min>boxhalfsize[Z] || max<-boxhalfsize[Z]) return false;
    
    normal = e0 ^ e1;
    if (!planeBoxOverlap(box, Point(v0), normal))
        return false;
    
    return true;
}

ElementTree::ElementTree(const prim_list &primitives, uint32_t nbPointsInLeaf) : max_points_in_leaf(nbPointsInLeaf), mesh(primitives) {
    root_node = new ElementTreeNode(this);
}
ElementTree::ElementTree(const vector<Reference<Shape> > &shapes, uint32_t nbPointsInLeaf) : max_points_in_leaf(nbPointsInLeaf), mesh(shapes) {
    root_node = new ElementTreeNode(this);
}

void ElementTreeNode::split(int split_axis) {
    Assert(!is_leaf);
    
    typedef vector<Point> pointlist;
    
    typedef nblist pidxlist;
    typedef pidxlist::iterator pidxiter;
    
    if (is_leaf) return;
    if (left || right) return;
    
    Mesh &mesh = tree->mesh;
    pointlist &points= mesh.vertex_pos;
    pidxlist &pidxs = this->points;
    
    if (split_axis == -1)
    	split_axis = bounding_box.MaximumExtent();
    float split_pos = (bounding_box.pMax[split_axis] + bounding_box.pMin[split_axis]) / 2.f;
    
    left = new ElementTreeNode(tree, this);
    right = new ElementTreeNode(tree, this);
    
    for (pidxiter point = pidxs.begin(); point != pidxs.end(); point++) {
        if(points[*point][split_axis] < split_pos) {
            left->points.push_back(*point);
        } else {
            right->points.push_back(*point);
        }
    }
    
    left->bounding_box = bounding_box;
    left->bounding_box.pMax[split_axis] = split_pos;
    
    right->bounding_box = bounding_box;
    right->bounding_box.pMin[split_axis] = split_pos;
    
    Info("bounding box: (%f,%f,%f) -> (%f,%f,%f)", bounding_box.pMin.x, bounding_box.pMin.y, bounding_box.pMin.z,
         bounding_box.pMax.x, bounding_box.pMax.y, bounding_box.pMax.z);
    Info("left bounding box: (%f,%f,%f) -> (%f,%f,%f)", left->bounding_box.pMin.x, left->bounding_box.pMin.y, left->bounding_box.pMin.z,
         left->bounding_box.pMax.x, left->bounding_box.pMax.y, left->bounding_box.pMax.z);
    Info("right bounding box: (%f,%f,%f) -> (%f,%f,%f)", right->bounding_box.pMin.x, right->bounding_box.pMin.y, right->bounding_box.pMin.z,
         right->bounding_box.pMax.x, right->bounding_box.pMax.y, right->bounding_box.pMax.z);    
    vector<Reference<Triangle> > &triangles = mesh.triangles;
    Reference<Triangle> triangle;
    for (nbiter t_idx = inside_triangles.begin(); t_idx != inside_triangles.end(); t_idx++) {
        triangle = triangles[*t_idx];

        float a = mesh.getPoint(triangle->getPoint(0))[split_axis],
            b = mesh.getPoint(triangle->getPoint(1))[split_axis],
            c = mesh.getPoint(triangle->getPoint(2))[split_axis];
        
        if (a <= split_pos || b <= split_pos || c <= split_pos) {
            left->inside_triangles.push_back(*t_idx);
            left->_inside_triangles.push_back(&* mesh.getTriangle(*t_idx)->getOriginal());
        } else {
            left->gone_triangles.push_back(*t_idx);
        }
        
        if (a >= split_pos || b >= split_pos || c >= split_pos) {
            right->inside_triangles.push_back(*t_idx);
            right->_inside_triangles.push_back(&* mesh.getTriangle(*t_idx)->getOriginal());
        } else {
            right->gone_triangles.push_back(*t_idx);
        }
        
        /*if (Intersects(left->bounding_box, triangle, mesh)) {
            left->inside_triangles.push_back(*t_idx);
        } else {
            left->gone_triangles.push_back(*t_idx);
        }

        if (Intersects(right->bounding_box, triangle, mesh)) {
            right->inside_triangles.push_back(*t_idx);
        } else {
            right->gone_triangles.push_back(*t_idx);
        }*/
    }
    
    left->setIsLeaf();
    if (left->is_leaf) {
        Info("Left child is leaf (points: %lu/%u)", left->points.size(), tree->max_points_in_leaf);
    } else {
        Info("Left child is not a leaf (points: %lu/%u)", left->points.size(), tree->max_points_in_leaf);
    }
    
    right->setIsLeaf();
    if (right->is_leaf) {
        Info("Right child is leaf (points: %lu/%u)", right->points.size(), tree->max_points_in_leaf);
    } else {
        Info("Right child is not a leaf (points: %lu/%u)", right->points.size(), tree->max_points_in_leaf);
    }
    
    Info("After split: %lu prims in current, %lu in left, %lu in right", inside_triangles.size(), left->inside_triangles.size(), right->inside_triangles.size());
    Info("Triangles gone: %lu in left, %lu in right", left->gone_triangles.size(), right->gone_triangles.size());
    
    points.clear();
    inside_triangles.clear();
}
    
ElementTreeNode::ElementTreeNode(ElementTree *tree) : parent(NULL), tree(tree) {
    const Mesh &mesh = tree->mesh;
    for (unsigned int i = 0; i < mesh.getNbVertices(); i++) {
        points.push_back(i);
    }
    for (unsigned int i = 0; i < mesh.getNbTriangles(); i++) {
        inside_triangles.push_back(i);
        _inside_triangles.push_back(&* mesh.getTriangle(i)->getOriginal());
    }
    setIsLeaf();
    createBoundingBox();
}

ElementTreeNode::ElementTreeNode(ElementTree *tree, ElementTreeNode *parent)
    : parent(parent), tree(tree) {
}

void ElementTreeNode::createBoundingBox() {    
    const vector<Point> &point_pos = tree->mesh.vertex_pos;
    
    Assert(!points.empty());
    // reset the bounding box
    bounding_box = BBox();
    
    nbiter end = points.end();
    nbiter point = points.begin();
    
    if (end == point) return;
    
    bounding_box.Insert(point_pos[*point]);
    bounding_box.Expand(.5f);
    
    for(; point != end; point++) {
        bounding_box.Insert(point_pos[*point]);
    }

    ::Vector extent = bounding_box.Extent();
    float maxSize = max(extent.x, max(extent.y, extent.z));
    bounding_box.Expand(maxSize * .01f);

    Info("Created BBox for ElementTreeNode");
}
    
bool ElementTreeNode::IntersectP(const Ray &ray) const {
    Assert(is_leaf);
    Assert(!empty());
    
    const trisciter tris_end = _inside_triangles.end();
    for (trisciter tris = _inside_triangles.begin(); tris != tris_end; tris++) {
        log::ShaftNodeIntersectionTest();
        if ((*tris)->IntersectP(ray))
            return true;
    }
    
    return false;
}
    
void ElementTreeNode::setIsLeaf() {
    if (empty()) {is_leaf = true; return;}

    if (points.size() > tree->max_points_in_leaf) {
        is_leaf = false;
        return;
    }
    
    is_leaf = true;
}
    
RNG ElementTreeNode::rng;

ElementTreeNode::pointlist ElementTreeNode::sample(uint count) const {
    pointlist result;
    uint nbTris = inside_triangles.size();
    
    uint size= count * count;
    result.reserve(size);
    
    const Mesh &mesh = tree->mesh;
    ::Normal n;
    
    for (uint cur = 0; cur < size; cur++) {
        float u = rng.RandomFloat() * nbTris, v;
        
        int i = static_cast<int>(u);
        u -= i;
        
        const Reference< ::Triangle> triangle = mesh.getTriangle(inside_triangles[i])->getOriginal();
        Point p;
        do {
            u = rng.RandomFloat();
            v = rng.RandomFloat();
            p = triangle->Sample(u, v, &n);

            {
                std::stringstream ss;
                ss << "Point " << p << " inside bounding box " << bounding_box << " ?";
                Warning("%s", ss.str().c_str());
            }
        } while (!bounding_box.Inside(p));
        result[cur] = p;
    }
    
    return result;
}

}
