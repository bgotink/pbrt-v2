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
#include <iostream>
#include <pthread.h>
#include "log.h"
#include "paramset.h"

namespace shaft {
    
#ifdef SHAFT_LOG
    AtomicInt64 nb_intersect_operations = 0;
    AtomicInt64 nb_intersect_done = 0;
    AtomicInt64 nb_no_intersected_shaft = 0;
    AtomicInt64 nb_node_intersect_done = 0;
    AtomicInt64 nb_shaft_blocked = 0;
    AtomicInt64 nb_shaft_empty = 0;
    AtomicInt64 nb_shaftaccel_intersectp = 0;
#endif
    
    enum ShaftState {
        SHAFT_BLOCKED, SHAFT_EMPTY, SHAFT_UNDECIDED, SHAFT_UNSET
    };
    
    bool RayBBoxIntersect(const BBox &box, const Ray &ray) {
        float t0 = ray.mint, t1 = ray.maxt;
        for (int i = 0; i < 3; ++i) {
            // Update interval for _i_th bounding box slab
            float invRayDir = 1.f / ray.d[i];
            float tNear = (box.pMin[i] - ray.o[i]) * invRayDir;
            float tFar  = (box.pMax[i] - ray.o[i]) * invRayDir;
                
            // Update parametric interval from slab intersection $t$s
            if (tNear > tFar) swap(tNear, tFar);
            t0 = tNear > t0 ? tNear : t0;
            t1 = tFar  < t1 ? tFar  : t1;
            if (t0 > t1) {
                return false;
            }
        }
        return true;
    }
    
    struct ShaftTreeNode {
        ShaftTreeNode(const Reference<Shaft> &shaft) : shaft(shaft), left(NULL), right(NULL), state(getState()) {
            pthread_mutex_init(&mutex, NULL);
            Assert(shaft);
        }
        ~ShaftTreeNode() { if (left) delete left; if (right) delete right; }
        
        Reference<Shaft> shaft;
        
        ShaftTreeNode *left, *right;
        
        ShaftState state;
        
        bool is_leaf;
        
        bool RayInShaft(const Ray &ray) {
            if (!RayBBoxIntersect(shaft->receiverNode->bounding_box, ray))
                return false;
            
            // lenghten the ray by 1, because the algorithm takes the
            // length to be E smaller than the distance to the light
            // source
            Ray r = Ray(ray.o, ray.d, ray.mint, ray.maxt + 1, ray.time, ray.depth);
            
            return RayBBoxIntersect(shaft->lightNode->bounding_box, r);
        }
        
        bool IntersectP(const Ray &ray) {
            if (!RayInShaft(ray))
                return false;
            
            if (state == SHAFT_UNSET) {
                split();
            }
            
            switch (state) {
                case SHAFT_BLOCKED:
                    ShaftBlocked();
                    return true;
                case SHAFT_EMPTY:
                    ShaftEmpty();
                    return false;
                case SHAFT_UNDECIDED:
                default:
                    if (is_leaf) {
                        return shaft->IntersectP(ray);
                    } else {
                        return left->IntersectP(ray) || right->IntersectP(ray);
                    }
            }
        }
        
    private:
        pthread_mutex_t mutex;
        
        void split() {
            pthread_mutex_lock(&mutex);
            
            // we just acquired the lock, check if not set already
            if (state != SHAFT_UNSET) {
                pthread_mutex_unlock(&mutex);
                return;
            }
            
            // we only get here if we should be in UNDECIDED but the subshafts aren't constructed yet
            state = SHAFT_UNDECIDED;
            
            Shaft &shaft = *this->shaft;
            Reference<ElementTreeNode> &receiver = shaft.receiverNode,
                                    &light = shaft.lightNode;
            
            if (receiver->is_leaf && light->is_leaf) {
                is_leaf = true;
                Info("Shaft is leaf");
                pthread_mutex_unlock(&mutex);
                return;
            } else {
                is_leaf = false;
            }
            
            Info("Splitting shaft...");
            
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
                Info("Splitting light...");
                light->split();
                
                left = new ShaftTreeNode(Shaft::constructSubShaft(receiver, light->left, light, shaft));
                right = new ShaftTreeNode(Shaft::constructSubShaft(receiver, light->right, light, shaft));
                Info("Prims in shaft: before: %lu; left: %lu, right %lu", shaft.triangles.size(), left->shaft->triangles.size(), right->shaft->triangles.size());
            } else {
                Info("Splitting receiver...");
                receiver->split();
                
                left = new ShaftTreeNode(Shaft::constructSubShaft(receiver->left, light, receiver, shaft));
                right = new ShaftTreeNode(Shaft::constructSubShaft(receiver->right, light, receiver, shaft));
                Info("Prims in shaft: before: %lu; left: %lu, right %lu", shaft.triangles.size(), left->shaft->triangles.size(), right->shaft->triangles.size());
            }
            
            pthread_mutex_unlock(&mutex);
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
    
    ShaftAccel::ShaftAccel(const prim_list &primitives, const prim_list &lights, uint32_t nbPointsInReceiverLeaf, uint32_t nbPoitnsInLightLeaf)
    : receiver_tree(new ElementTree(primitives, nbPointsInReceiverLeaf)), light_tree(new ElementTree(lights, nbPoitnsInLightLeaf)),
    fallback_accel(new BVHAccel(primitives))
    {
        bounding_box = Union(receiver_tree->root_node->bounding_box, light_tree->root_node->bounding_box);
        shaft_tree = new ShaftTreeNode(Shaft::constructInitialShaft(receiver_tree->root_node, light_tree->root_node));
    }
    
    ShaftAccel::ShaftAccel(const prim_list &primitives, const shape_list &lights, uint32_t nbPointsInReceiverLeaf, uint32_t nbPointsInLightLeaf)
    : receiver_tree(new ElementTree(primitives, nbPointsInReceiverLeaf)), light_tree(new ElementTree(lights, nbPointsInLightLeaf)),
    fallback_accel(new BVHAccel(primitives))
    {
        bounding_box = Union(receiver_tree->root_node->bounding_box, light_tree->root_node->bounding_box);
        shaft_tree = new ShaftTreeNode(Shaft::constructInitialShaft(receiver_tree->root_node, light_tree->root_node));
    }
    
    ShaftAccel::~ShaftAccel() {
        ShaftLogResult();
        delete shaft_tree;
        delete fallback_accel;
    }
    
    bool ShaftAccel::IntersectP(const Ray &ray) const {
        ShaftAccelIntersectP();
        return shaft_tree->IntersectP(ray);
    }
    
    ShaftAccel *createShaftAccel(const std::vector<Reference<Primitive> > &receivers,
                                 const std::vector<Reference<Shape> > &lights,
                                 const ParamSet &ps) {
        uint32_t nbPointsInReceiverNode = (uint32_t)ps.FindOneInt("receiver_treshold", 15);
        uint32_t nbPointsInLightNode = (uint32_t)ps.FindOneInt("light_treshold", 15);
        ps.ReportUnused();
        
        return new ShaftAccel(receivers, lights, nbPointsInReceiverNode, nbPointsInLightNode);
    }
    
} // namespace shaft