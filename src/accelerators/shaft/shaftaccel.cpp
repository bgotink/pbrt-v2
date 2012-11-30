//
//  shaftaccel.cpp
//  pbrt
//
//  Created by Bram Gotink on 29/11/12.
//
//

#include "shaftaccel.h"
#include "../bvh.h"
#include "shaft.h"

namespace shaft {
    
    enum ShaftState {
        SHAFT_BLOCKED, SHAFT_EMPTY, SHAFT_UNDECIDED, SHAFT_UNSET
    };
    
    struct ShaftTreeNode {
        ShaftTreeNode(Reference<Shaft> shaft) : shaft(shaft), left(NULL), right(NULL), state(getState()) {}
        ~ShaftTreeNode() { if (left) delete left; if (right) delete right; }
        
        Reference<Shaft> shaft;
        
        ShaftTreeNode *left, *right;
        
        ShaftState state;
        
        bool is_leaf;
        
        bool RayInShaft(const Ray &ray) {
            return shaft->receiverNode->bounding_box.IntersectP(ray)
                && shaft->lightNode->bounding_box.IntersectP(ray);
        }
        
        bool IntersectP(const Ray &ray) {
            if (!RayInShaft(ray))
                return false;
            
            if (state == SHAFT_UNSET) {
                split();
            }
            
            switch (state) {
                case SHAFT_BLOCKED:
                    return true;
                case SHAFT_EMPTY:
                    return false;
                case SHAFT_UNDECIDED:
                default:
                    if (is_leaf) {
                        return shaft->IntersectsP(ray);
                    } else {
                        return left->IntersectP(ray) || right->IntersectP(ray);
                    }
            }
        }
        
    private:
        void split() {
            // we only get here if we should be in UNDECIDED but the subshafts aren't constructed yet
            state = SHAFT_UNDECIDED;
            
            Shaft &shaft = *this->shaft;
            Reference<ElementTreeNode> &receiver = shaft.receiverNode,
                                    &light = shaft.lightNode;
            
            if (receiver->is_leaf && light->is_leaf) {
                is_leaf = true;
                return;
            } else {
                is_leaf = false;
            }
            
            bool split_light;
            
            if (receiver->is_leaf)
                split_light = true;
            else if (light->is_leaf)
                split_light = false;
            else {
                // use a heuristic (todo: use the clean heuristic provided by Laine
                int axis = shaft.geometry.main_axis;
                split_light = (receiver->bounding_box[1][axis] - receiver->bounding_box[0][axis])
                            < (light->bounding_box[1][axis] - light->bounding_box[0][axis]);
            }
            
            if (split_light) {
                light->split();
                
                left = new ShaftTreeNode(Shaft::constructSubShaft(receiver, light->left, light, shaft));
                right = new ShaftTreeNode(Shaft::constructSubShaft(receiver, light->right, light, shaft));
            } else {
                receiver->split();
                
                left = new ShaftTreeNode(Shaft::constructSubShaft(receiver->left, light, receiver, shaft));
                right = new ShaftTreeNode(Shaft::constructSubShaft(receiver->right, light, receiver, shaft));
            }
        }
        
        ShaftState getState() const {
            if (shaft->empty()) {
                return SHAFT_EMPTY;
            }
            
            const ShaftGeometry &geom = shaft->geometry;
            const Shaft::surface_list surfaces = shaft->surfaces;
            
            for (Shaft::surface_citer surf = surfaces.begin(); surf != surfaces.end(); surf++) {
                if (geom.blockedBy(*surf))
                    return SHAFT_BLOCKED;
            }
            
            return SHAFT_UNSET;
        }
    };
    
    ShaftAccel::ShaftAccel(const prim_list &primitives, const prim_list &lights)
    : receiver_tree(new ElementTree(primitives)), light_tree(new ElementTree(lights)),
    fallback_accel(new BVHAccel(primitives))
    {
        bounding_box = Union(receiver_tree->root_node->bounding_box, light_tree->root_node->bounding_box);
        shaft_tree = new ShaftTreeNode(Shaft::constructInitialShaft(receiver_tree->root_node, light_tree->root_node));
    }
    
    ShaftAccel::ShaftAccel(const prim_list &primitives, const shape_list &lights)
    : receiver_tree(new ElementTree(primitives)), light_tree(new ElementTree(lights)),
    fallback_accel(new BVHAccel(primitives))
    {
        bounding_box = Union(receiver_tree->root_node->bounding_box, light_tree->root_node->bounding_box);
        shaft_tree = new ShaftTreeNode(Shaft::constructInitialShaft(receiver_tree->root_node, light_tree->root_node));
    }
    
    ShaftAccel::~ShaftAccel() {
        delete shaft_tree;
        delete fallback_accel;
    }
    
    bool ShaftAccel::IntersectP(const Ray &ray) const {
        return false;
    }
    
} // namespace shaft