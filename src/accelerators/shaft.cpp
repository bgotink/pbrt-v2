//
//  shaft.cpp
//  pbrt
//
//  Created by Bram Gotink on 14/11/12.
//
//

#include "shaft.h"

#define X 0
#define Y 1
#define Z 2

using namespace std;

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
bool Intersects(const BBox &box, const Reference<Triangle> &triangle) {
    Point boxcenter = (box.pMax + box.pMin) / 2;
    Vector boxhalfsize = box.pMax - boxcenter;
    
    Vector v0, v1, v2;
    
    float min, max, p0, p1, p2, rad, fex, fey, fez;
    Vector normal, e0, e1, e2;
    
    v0 = triangle->getPoint(0) - boxcenter;
    v1 = triangle->getPoint(1) - boxcenter;
    v2 = triangle->getPoint(2) - boxcenter;
    
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

SplitPlane findSplitPlane(const Point * const points, const vector<int> &indices) {
    SplitPlane result;
    
    // TODO!!!
    
    return result;
}

void ShaftTreeNode::split() {
    if (is_leaf) return;
    if (left || right) return;
    
    TriangleMesh *mesh = tree->mesh;
    Point *mesh_points = mesh->p;
    int *vertexIndex = mesh->vertexIndex;
    
    int split_axis; float split_pos;
    {
        SplitPlane split = findSplitPlane(mesh_points, points);
        split_axis = split.split_axis;
        split_pos = split.split_pos;
    }
    
    left = new ShaftTreeNode(tree, this);
    right = new ShaftTreeNode(tree, this);
    
    for (vector<int>::iterator point = points.begin(); point != points.end(); point++) {
        if(mesh_points[*point][split_axis] < split_pos) {
            left->points.push_back(*point);
        } else {
            right->points.push_back(*point);
        }
    }
    
    left->createBoundingBox(mesh_points);
    right->createBoundingBox(mesh_points);
    
    if (left->points.size() < tree->max_points_in_leaf)
        left->is_leaf = true;
    if (right->points.size() < tree->max_points_in_leaf)
        right->is_leaf = true;
    
    for (vector<int>::iterator t_idx = inside_primitives.begin(); t_idx != inside_primitives.end(); t_idx++) {
        Reference<Triangle> t = mesh->getTriangle(*t_idx);
        
        if (Intersects(left->bounding_box, t)) {
            left->inside_primitives.push_back(*t_idx);
        } else {
            left->gone_primitives.push_back(*t_idx);
        }
        
        if (Intersects(right->bounding_box, t)) {
            right->inside_primitives.push_back(*t_idx);
        } else {
            right->gone_primitives.push_back(*t_idx);
        }
    }
    
    points.clear();
    inside_primitives.clear();
}

ShaftTreeNode::ShaftTreeNode(ShaftTree *tree, ShaftTreeNode *parent)
    : parent(parent), tree(tree) {
}

void ShaftTreeNode::createBoundingBox(const Point * const mesh_points) {
    for (vector<int>::iterator point = points.begin(); point != points.end(); point++) {
        bounding_box.Union(mesh_points[*point]);
    }
}