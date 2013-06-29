//
//  shaftaccel.cpp
//  pbrt
//
//  Created by Bram Gotink on 29/11/12.
//
//

#include "core/timer.h"
#include "shaftaccel.h"
#include "../bvh.h"
#include "shaft.h"
#include <iostream>
#include "log.h"
#include "paramset.h"
#include "intersection.h"
#include <set>
#include <list>
#include <vector>

#define SHAFT_REMOVE_EMPTY_SHAFTS
//#define SHAFT_HACK_ERRONOUS_BUT_FAST

namespace shaft {
        
    class BVHAccelCreator {
        typedef std::vector<Reference<Primitive> > primarr;
        typedef primarr::iterator primaiter;
        
        typedef std::list<Reference<Primitive> > primlist;
        
        primarr prims;
        const Mesh &mesh;
    public:
        typedef std::set<Shape *> trisset;
        typedef trisset::iterator trisiter;
        
        BVHAccelCreator(const primarr &p, const Mesh &mesh) : mesh(mesh) {
            prims = p;
        }
        
        void set(trisset &shapes) {
            Reference<Material> material = mesh.getSomeMaterial();
            
            prims.clear();
            prims.reserve(shapes.size());

            Reference<Shape> shape;
            Reference<Primitive> prim;
            for (trisiter triangle = shapes.begin(); triangle != shapes.end(); triangle++) {
                shape = *triangle;
                
                /*prim = mesh.getPrimitive(shape);
                if (prim) {
                    newPrims.push_back(Reference<Primitive>(prim));
                }*/
                prims.push_back(Reference<Primitive>(new GeometricPrimitive(shape, material, NULL)));
            }
            Info("Shapes to retain: %lu, primitives kept: %lu", shapes.size(), prims.size());
        }
        
        BVHAccel *createAccel() {
            return new BVHAccel(prims);
        }
    };
    
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
        ShaftTreeNode(const Reference<Shaft> &shaft) : shaft(shaft), left(NULL), right(NULL), state(getState())
        , show(NULL), probVis(NULL) {
            Assert(shaft);
            
            if (state != SHAFT_UNSET) {
                is_leaf = true;
                Info("%s shaft created", state == SHAFT_BLOCKED ? "blocked" : "empty");
            }
        }
        ~ShaftTreeNode() { if (left) delete left; if (right) delete right; if(show) delete show; if(probVis) delete probVis; }
        
        Reference<Shaft> shaft;
        
        ShaftTreeNode *left, *right;
        
        ShaftState state;
        
        bool is_leaf;
        bool *show;
        bool *probVis;
        
        bool RayInShaft(const Ray &ray) const {
#ifdef SHAFT_HACK_ERRONOUS_BUT_FAST
            if (!shaft->receiverNode->bounding_box.Inside(ray(ray.mint)))
#else
            if (!shaft->receiverNode->bounding_box.Inside(ray(-ray.mint)))
#endif
/*
 note: it was originally ray.o,
 and even though ray(ray.mint) _should_ also be correct,
 neither of the above gives a correct answer.
 ray(-ray.mint) does however, so let's use that
 */
                return false;
            /*
#ifdef SHAFT_LOG
            Ray r = Ray(ray.o, ray.d, 0, ray.maxt + 1, ray.time, ray.depth);
#else
            Ray r = Ray(ray.o, ray.d, 0, ray.maxt + 1, ray.time);
#endif
            */
            return RayBBoxIntersect(shaft->lightNode->bounding_box, ray);
        }
        
        bool IntersectP(const Ray &ray, bool showShafts = false) const {
            if (!RayInShaft(ray))
                return false;
            
            switch (state) {
                case SHAFT_BLOCKED:
                    log::ShaftBlocked();
                    return true;
                case SHAFT_EMPTY:
                    log::ShaftEmpty();
                    return false;
                case SHAFT_UNDECIDED:
                default:
                    if (is_leaf) {
                        if (show == NULL)
                            return shaft->IntersectP(ray);
                        else {
                            return *show ? shaft->IntersectP(ray) : false;
                        }
                    } else {
                        return left->IntersectP(ray) || right->IntersectP(ray);
                    }
            }
        }
        
        float Visibility(const Ray &ray, bool *set = NULL, bool is_top = true) const {
            if (!RayInShaft(ray)) {
                return 0.;
            }
            if (set) *set = true;
            
            switch (state) {
                case SHAFT_BLOCKED:
                    return 0.;
                case SHAFT_EMPTY:
                    return 1.;
                case SHAFT_UNDECIDED:
                default:
                    if (is_leaf) {
                        if (!is_top)
                            log::ShaftSetSide(set == NULL);
                        return shaft->Visibility(ray);
                    } else {
                        bool s = false;
                        float v = left->Visibility(ray, &s, false);
                        if (s) return v;
                        return right->Visibility(ray, NULL, false);
                    }
                    break;
            }
        }
        
        void split() {
            if (state != SHAFT_UNSET) return;
            
            doSplit();
            
            if (is_leaf)
                return;
            else {
                left->split();
                right->split();
            }
        }
        
        void showShaft(const Point &shaftPoint, BVHAccelCreator &accel) {
            Assert(state != SHAFT_UNSET);
            Assert(isValid());
            
            if (!is_leaf) {
                left->showShaft(shaftPoint, accel);
                right->showShaft(shaftPoint, accel);
                return;
            }
            
            show = new bool;
            if (shaft->receiverNode->bounding_box.Inside(shaftPoint)) {
#ifdef SHAFT_LOG
                Info("showShaft actually doing it's work at ShaftTreeNode %p, depth: %u", this, shaft->depth);
#else
                Info("showShaft actually doing it's work at ShaftTreeNode %p", this);
#endif
                Info("showShaft for point (%f, %f, %f)", shaftPoint.x, shaftPoint.y, shaftPoint.z);
                *show = true;
                
                BVHAccelCreator::trisset triangles;
                const ElementTreeNode::nblist &nodeTris = shaft->receiverNode->inside_triangles;
                const Shaft::nbllist &shaftTris = shaft->triangles;
                
                Mesh &mesh = shaft->getMesh();
                
                size_t nodeRetained = 0, shaftRetained = 0;
                for (ElementTreeNode::nbciter tidx = nodeTris.begin(); tidx != nodeTris.end(); tidx++) {
                    ::Triangle *triangle = &*mesh.getTriangle(*tidx)->getOriginal();

                    if (triangles.insert(static_cast<Shape *>(triangle)).second)
                        nodeRetained++;
                }
                for (Shaft::nblciter tidx = shaftTris.begin(); tidx != shaftTris.end(); tidx++) {
                    ::Triangle *triangle = &*mesh.getTriangle(*tidx)->getOriginal();
                    
                    if (triangles.insert(static_cast<Shape *>(triangle)).second)
                        shaftRetained++;
                }
                
                Info("Retaining %lu triangles (%lu/%lu from node, %lu/%lu from shaft)", triangles.size(),
                        nodeRetained, nodeTris.size(),
                        shaftRetained, shaftTris.size());
                Info("Total size of mesh: %u", mesh.getNbTriangles());
                accel.set(triangles);
                
            } else *show = false;
        }

        void setUseProbVis(bool useProbVis, RNG *rng = NULL, const string * const type = NULL) {
            Assert(state != SHAFT_UNSET);
            Assert(isValid());
            
            if (state == SHAFT_EMPTY || state == SHAFT_BLOCKED) {
                // we don't need ProbVis here
                return;
            }
            
            if (probVis) return; // already run
            probVis = new bool;
            *probVis = useProbVis;
            
            if (useProbVis && !rng)
                rng = new RNG;
            
            if (is_leaf) {
                shaft->initProbVis(useProbVis, rng, type);
            } else {
                left->setUseProbVis(useProbVis, rng, type);
                right->setUseProbVis(useProbVis, rng, type);
            }
        }
        
        operator bool() const {
            return isValid();
        }
        
        bool isValid() const {
            return is_leaf || (right && left);
        }
        
    private:
                     
        typedef std::vector<Reference<Primitive> > primlist;
        typedef primlist::iterator primiter;
        
        void doSplit() {
            // we just acquired the lock, check if not set already
            if (state != SHAFT_UNSET) {
                return;
            }
            
            Shaft &shaft = *this->shaft;
            Reference<ElementTreeNode> &receiver = shaft.receiverNode,
                                       &light = shaft.lightNode;
            
            if (shaft.isLeaf()) {
                is_leaf = true;
                state = SHAFT_UNDECIDED;
                
                /*Warning("Shaft is leaf (#prims in shaft: %lu, #prims in node: %lu, #points in node: %lu",
                                shaft.triangles.size(),
                                shaft.receiverNode->inside_triangles.size(),
                                shaft.receiverNode->points.size());*/
#ifdef SHAFT_LOG
                log::ShaftLeafCreated(shaft.receiverNode->inside_triangles.size(),
                                shaft.receiverNode->points.size(),
                                shaft.triangles.size(),
                                shaft.getDepth());          
#endif
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
                if (axis != -1) {
                    float diff[3];
                    for (int i = 0; i < 3; i++)
                        diff[i] = (light->bounding_box[1][i] - light->bounding_box[0][i])
                                - (receiver->bounding_box[1][i] - receiver->bounding_box[0][i]);

                    bool neg = false;
                    float max = -1;

                    for (int i = 0; i < 3; i++) {
                        if (diff[i] < 0) {
                            if (-diff[i] > max) {
                                max = -diff[i];
                                neg = true;
                            }
                        } else {
                            if (diff[i] > max) {
                                max = diff[i];
                                neg = false;
                            }
                        }
                    }

                    split_light = !neg;
                } else {
                    split_light = receiver->bounding_box.Extent().LengthSquared()
                                < light->bounding_box.Extent().LengthSquared();
                }
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

#ifdef SHAFT_REMOVE_EMPTY_SHAFTS
            if (left->empty()) {
                Info("Left subshaft is empty!");

                this->shaft = right->shaft;
                this->shaft->depth --;
                this->state = right->state;
                this->is_leaf = right->is_leaf;

                delete left;
                delete right;
                left = right = NULL;

                doSplit();
                return;
            }
            if (right->empty()) {
                Info("Right subshaft is empty!");

                this->shaft = left->shaft;
                this->shaft->depth --;
                this->state = left->state;
                this->is_leaf = left->is_leaf;

                delete left;
                delete right;
                left = right = NULL;

                doSplit();
                return;
            }
#endif
            
            state = SHAFT_UNDECIDED;
        }

        inline bool empty() {
            return shaft->receiverNode->empty();
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
    
    ShaftAccel::ShaftAccel(const prim_list &primitives, const prim_list &lights, uint32_t nbPointsInReceiverLeaf, uint32_t nbPoitnsInLightLeaf, bool b, const Point &shaftPoint, const string * const probVisType)
    : receiver_tree(new ElementTree(primitives, nbPointsInReceiverLeaf)), light_tree(new ElementTree(lights, nbPoitnsInLightLeaf)), showShafts(b), shaftPoint(shaftPoint)
    {
        prim = *primitives.begin();
        bounding_box = Union(receiver_tree->root_node->bounding_box, light_tree->root_node->bounding_box);
        shaft_tree = new ShaftTreeNode(Shaft::constructInitialShaft(receiver_tree->root_node, light_tree->root_node));
        
        shaft_tree->split();
        
        BVHAccelCreator fallbackCreator(primitives, receiver_tree->mesh);
        if (showShafts) {
            shaft_tree->showShaft(shaftPoint, fallbackCreator);
        }
        fallback_accel = fallbackCreator.createAccel();
        
        if (probVisType == NULL) {
            shaft_tree->setUseProbVis(false);
        } else {
            RNG *rng = new RNG;
            shaft_tree->setUseProbVis(true, rng, probVisType);
        }
    }
    
    ShaftAccel::ShaftAccel(const prim_list &primitives, const shape_list &lights, uint32_t nbPointsInReceiverLeaf, uint32_t nbPointsInLightLeaf, bool b, const Point &shaftPoint, const string * const probVisType)
    : receiver_tree(new ElementTree(primitives, nbPointsInReceiverLeaf)), light_tree(new ElementTree(lights, nbPointsInLightLeaf)), showShafts(b), shaftPoint(shaftPoint)
    {
        prim = *primitives.begin();
        bounding_box = Union(receiver_tree->root_node->bounding_box, light_tree->root_node->bounding_box);
        shaft_tree = new ShaftTreeNode(Shaft::constructInitialShaft(receiver_tree->root_node, light_tree->root_node));
        
        Timer buildTimer;
        buildTimer.Start();
        
        shaft_tree->split();
        
        log::ShaftSaveBuildTime(buildTimer.Time());

        BVHAccelCreator fallbackCreator(primitives, receiver_tree->mesh);
        if (showShafts) {
            shaft_tree->showShaft(shaftPoint, fallbackCreator);
        }
        fallback_accel = fallbackCreator.createAccel();
        
        if (probVisType == NULL) {
            shaft_tree->setUseProbVis(false);
        } else {
            RNG *rng = new RNG;
            shaft_tree->setUseProbVis(true, rng, probVisType);
        }
    }
    
    ShaftAccel::~ShaftAccel() {
        log::ShaftLogResult();
        delete shaft_tree;
        delete fallback_accel;
    }
    
    bool ShaftAccel::Intersect(const Ray &ray, Intersection *isect) const {
        return fallback_accel->Intersect(ray, isect);
    }
    
    bool ShaftAccel::IntersectP(const Ray &ray) const {
        log::ShaftAccelIntersectP();
        return shaft_tree->IntersectP(ray, showShafts);
    }
    
    float ShaftAccel::Visibility(const Ray &ray) const {
        log::ShaftAccelIntersectP();
        return shaft_tree->Visibility(ray);
    }
    
    ShaftAccel *createShaftAccel(const std::vector<Reference<Primitive> > &receivers,
                                 const std::vector<Reference<Shape> > &lights,
                                 const ParamSet &ps) {
        uint32_t nbPointsInReceiverNode = (uint32_t)ps.FindOneInt("receiver_treshold", 15);
        uint32_t nbPointsInLightNode = (uint32_t)ps.FindOneInt("light_treshold", 15);
        bool showShafts = ps.FindOneBool("draw_shafts", false);
        Point shaftPoint;
        if (showShafts) {
            shaftPoint = ps.FindOnePoint("shaft_point", Point(0, 0, 0));
        }
        bool useProbVis = ps.FindOneBool("use_probvis", !showShafts);
        string probVisType;
        if (useProbVis) {
            probVisType = ps.FindOneString("probvis_type", "bjorn");
        }
        
        ps.ReportUnused();
        
        return new ShaftAccel(receivers, lights,
                              nbPointsInReceiverNode, nbPointsInLightNode,
                              showShafts, shaftPoint, useProbVis ? &probVisType : NULL);
    }
    
} // namespace shaft