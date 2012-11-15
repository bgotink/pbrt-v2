//
//  shaft.h
//  pbrt
//
//  Created by Bram Gotink on 14/11/12.
//
//

#if defined(_MSC_VER)
#pragma once
#endif

#ifndef __pbrt__shaft__
#define __pbrt__shaft__

#include "pbrt.h"
#include "primitive.h"
#include "../shapes/trianglemesh.h"

struct ShaftTreeNode;
struct ShaftTree;

bool Intersects(const BBox &box, const Reference<Triangle> &triangle);

class ShaftAccel : public Aggregate {
public:
    typedef std::vector<Point> PointArray;
    
    bool canIntersect() { return true; }
    
    ~ShaftAccel();
    
private:
    ShaftTree *tree;
};

struct ShaftTree {
    ShaftTreeNode *root_node;
    std::vector<Point> point_pos;
    uint32_t max_points_in_leaf;
    TriangleMesh *mesh;
};

struct ShaftTreeNode {
    ShaftTreeNode *left, *right;
    ShaftTreeNode *parent;
    ShaftTree *tree;
    
    BBox bounding_box;
    
    std::vector<int> points;
    std::vector<int> gone_primitives;
    std::vector<int> inside_primitives;
    
    bool is_leaf;
    
private:
    ShaftTreeNode(ShaftTree *tree, ShaftTreeNode *parent);
    
    void split();
    void createBoundingBox(const Point * const mesh_points);
};

#endif /* defined(__pbrt__shaft__) */
