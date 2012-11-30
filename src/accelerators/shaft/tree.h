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

#include <vector>

namespace shaft {

struct ElementTreeNode;
struct ElementTree;

bool Intersects(const BBox &box, const Reference<Triangle> &triangle, const Mesh &mesh);

    struct ElementTree: public ReferenceCounted {
private:
    typedef std::vector<Reference<Primitive> > prim_list;
    typedef std::vector<Reference<Shape> > shape_list;
    
public:
    friend class ElementTreeNode;
    
    Reference<ElementTreeNode> root_node;
    uint32_t max_points_in_leaf;
    Mesh mesh;
    
    inline std::vector<Point> &getPointPos() { return mesh.vertex_pos; }
        
    ElementTree(const prim_list &primitives);
    ElementTree(const shape_list &shapes);
};

struct ElementTreeNode : public ReferenceCounted {
    
    ElementTreeNode(ElementTree *tree);
    
    friend class ElementTree;
    friend class ShaftTreeNode;
    
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
