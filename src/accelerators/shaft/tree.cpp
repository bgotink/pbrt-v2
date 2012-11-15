//
//  shaft.cpp
//  pbrt
//
//  Created by Bram Gotink on 14/11/12.
//
//

#include "tree.h"

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
bool Intersects(const BBox &box, const Reference<Triangle> &triangle, const vector<Point> &points) {
    Point boxcenter = (box.pMax + box.pMin) / 2;
    Vector boxhalfsize = box.pMax - boxcenter;
    
    Vector v0, v1, v2;
    
    float min, max, p0, p1, p2, rad, fex, fey, fez;
    Vector normal, e0, e1, e2;
    
    v0 = points[(*triangle)[0]] - boxcenter;
    v1 = points[(*triangle)[1]] - boxcenter;
    v2 = points[(*triangle)[2]] - boxcenter;
    
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
    if(min>boxhalfsize[X] || max<-boxhalfsize[X]) return 0;
    
    FINDMINMAX(v0[Y],v1[Y],v2[Y],min,max);
    if(min>boxhalfsize[Y] || max<-boxhalfsize[Y]) return 0;
    
    FINDMINMAX(v0[Z],v1[Z],v2[Z],min,max);
    if(min>boxhalfsize[Z] || max<-boxhalfsize[Z]) return 0;
    
    normal = e0 ^ e1;
    if (!planeBoxOverlap(box, Point(v0), normal))
        return false;
    
    return true;
}

struct SplitPlane {
    int split_axis;
    float split_pos;
};

SplitPlane findSplitPlane(const ElementTreeNode &node) {
    SplitPlane result;
    
    /*
     * This is a really naive way of choosing which plane to split at.
     * TODO refine!
     */
    
    const BBox &box = node.bounding_box;
    
    float diffx = box.pMax.x - box.pMin.x,
          diffy = box.pMax.y - box.pMin.y,
          diffz = box.pMax.z - box.pMin.z;
    
    if (diffx > diffy && diffx > diffz) {
        result.split_axis = X;
        result.split_pos = (box.pMax.x + box.pMin.x) / 2;
    } else if (diffy > diffz) {
        result.split_axis = Y;
        result.split_pos = (box.pMax.y + box.pMin.y) / 2;
    } else {
        result.split_axis = Z;
        result.split_pos = (box.pMax.z + box.pMin.z) / 2;
    }
    
    return result;
}
    
ElementTree::ElementTree(TriangleMesh &mesh) : mesh(mesh) {}

void ElementTreeNode::split() {
    typedef vector<Point> pointlist;
    typedef pointlist::iterator pointiter;
    
    typedef vector<int> pidxlist;
    typedef pidxlist::iterator pidxiter;
    
    if (is_leaf) return;
    if (left || right) return;
    
    Mesh &mesh = tree->mesh;
    pointlist &points= mesh.vertex_pos;
    pidxlist &pidxs = this->points;
    
    int split_axis; float split_pos;
    {
        SplitPlane split = findSplitPlane(*this);
        split_axis = split.split_axis;
        split_pos = split.split_pos;
    }
    
    left = new ElementTreeNode(tree, this);
    right = new ElementTreeNode(tree, this);
    
    for (pidxiter point = pidxs.begin(); point != pidxs.end(); point++) {
        if(points[*point][split_axis] < split_pos) {
            left->points.push_back(*point);
        } else {
            right->points.push_back(*point);
        }
    }
    
    left->createBoundingBox();
    right->createBoundingBox();
    
    if (left->points.size() < tree->max_points_in_leaf)
        left->is_leaf = true;
    if (right->points.size() < tree->max_points_in_leaf)
        right->is_leaf = true;
    
    vector<Reference<Triangle> > &triangles = mesh.triangles;
    Reference<Triangle> triangle;
    for (vector<int>::iterator t_idx = inside_triangles.begin(); t_idx != inside_triangles.end(); t_idx++) {
        triangle = triangles[*t_idx];
        
        if (Intersects(left->bounding_box, triangle)) {
            left->inside_triangles.push_back(*t_idx);
        } else {
            left->gone_triangles.push_back(*t_idx);
        }
        
        if (Intersects(right->bounding_box, triangle)) {
            right->inside_triangles.push_back(*t_idx);
        } else {
            right->gone_triangles.push_back(*t_idx);
        }
    }
    
    points.clear();
    inside_triangles.clear();
}

ElementTreeNode::ElementTreeNode(ElementTree *tree, ElementTreeNode *parent)
    : parent(parent), tree(tree) {
}

void ElementTreeNode::createBoundingBox() {
    const vector<Point> &point_pos = tree->mesh.vertex_pos;
    for (vector<int>::const_iterator point = points.begin(); point != points.end(); point++) {
        bounding_box.Union(point_pos[*point]);
    }
}

}