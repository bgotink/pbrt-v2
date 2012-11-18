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

#ifndef __pbrt__tree__
#define __pbrt__tree__

#include "pbrt.h"
#include "mesh.h"
#include "memory.h"

namespace shaft {

struct ElementTreeNode;
struct ElementTree;

bool Intersects(const BBox &box, const Reference<Triangle> &triangle);

struct ElementTree {
    friend class ElementTreeNode;
    
    Reference<ElementTreeNode> root_node;
    uint32_t max_points_in_leaf;
    Mesh mesh;
    
    std::vector<Point> &getPointPos() { return mesh.vertex_pos; }
public:
    ElementTree(TriangleMesh &mesh);
};

struct ElementTreeNode : public ReferenceCounted {
    friend class ElementTree;
    
    Reference<ElementTreeNode> left, right;
    Reference<ElementTreeNode> parent;
    ElementTree *tree;
    
    BBox bounding_box;
    
    std::vector<int> points;
    std::vector<int> gone_triangles;
    std::vector<int> inside_triangles;
    
    bool is_leaf;
    
private:
    ElementTreeNode(ElementTree *tree, ElementTreeNode *parent);
    
    void split();
    void createBoundingBox();
};
    
}

#endif /* defined(__pbrt__tree__) */
