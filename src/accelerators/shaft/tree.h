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
#include "rng.h"

#include <vector>

namespace shaft {

struct ElementTreeNode;
struct ElementTree;

bool Intersects(const BBox &box, const Reference<Triangle> &triangle);

typedef enum ElementTreeNodeSplitType {
    CENTER, MEDIAN, MEAN
} ElementTreeNodeSplitType;

struct ElementTree: public ReferenceCounted {
private:
    typedef std::vector<Reference<Primitive> > prim_list;
    typedef std::vector<Reference<Shape> > shape_list;
    typedef ElementTreeNodeSplitType split_type_t;
    
public:
    friend struct ElementTreeNode;
    
    Reference<ElementTreeNode> root_node;
    const uint32_t max_points_in_leaf;
    Mesh mesh;
    const split_type_t node_split_type;
    
    inline std::vector<Point> &getPointPos() { return mesh.vertex_pos; }
    uint64_t memsize() const;
        
    ElementTree(const prim_list &primitives, uint32_t nbPoinsInLeaf = 15, split_type_t split_type = MEAN);
    ElementTree(const shape_list &shapes, uint32_t nbPointsInLeaf = 15, split_type_t split_type = MEAN);
};

struct ElementTreeNode : public ReferenceCounted {
public:
    typedef std::vector<unsigned int> nblist;
    typedef nblist::iterator nbiter;
    typedef nblist::const_iterator nbciter;
    
    typedef std::vector< ::Point> pointlist;
    typedef pointlist::iterator pointiter;
    typedef pointlist::const_iterator pointciter;
    
    ElementTreeNode(ElementTree *tree);
    bool IntersectP(const Ray &ray) const;
    inline bool empty() const { return inside_triangles.empty();}

    uint64_t memsize() const;
    
    pointlist sample(uint count) const;
    
    friend struct ElementTree;
    friend class ShaftTreeNode;
    
    Reference<ElementTreeNode> left, right;
    Reference<ElementTreeNode> parent;
    ElementTree *tree;
    
    BBox bounding_box;
    
    nblist points;
    nblist gone_triangles;
    nblist inside_triangles;
    
    bool is_leaf;

    typedef std::vector<const ::Triangle *> trislist;
    typedef trislist::const_iterator trisciter;
    
    trislist _inside_triangles;
    
private:
    ElementTreeNode(ElementTree *tree, ElementTreeNode *parent);
    static RNG rng;
    
    void split(int axis = -1);
    void createBoundingBox();
    void setIsLeaf();
};
    
}

#endif /* defined(__pbrt__tree__) */
