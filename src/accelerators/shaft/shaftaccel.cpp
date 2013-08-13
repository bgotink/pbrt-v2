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
#include <pthread.h>
#include "log.h"
#include "paramset.h"
#include "intersection.h"
#include <set>
#include <list>
#include <vector>
#include "vis/visibility.h"

//#define SHAFT_TRY_LAST_SHAFT

using shaft::vis::ProbabilisticVisibilityCalculator;

namespace shaft {

#ifdef SHAFT_TRY_LAST_SHAFT
# if defined(PBRT_CPP11)
#   pragma message("Using CPP11 thread_local")
    static const thread_local ShaftTreeNode *lastHit;
# elif defined(__GCC__)
#   pragma message("Using GCC's __thread")
    static const __thread ShaftTreeNode *lastHit;
# else
#   pragma message("Warning: no thread-local support detected!")
    static const ShaftTreeNode *lastHit;
# endif
#endif
        
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
    
    class ShaftTreeNode {
    public:
        ShaftTreeNode(const Reference<Shaft> &shaft) : shaft(shaft), left(NULL), right(NULL), state(getState())
        , show(NULL), probVis(NULL) {
            pthread_mutex_init(&mutex, NULL);
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
//            Ray r = Ray(ray.o, ray.d, 0, ray.maxt + 1, ray.time
//#if defined(SHAFT_LOG)
//                    , ray.depth
//#endif
//            );

        	const BBox &receiver_bbox = shaft->receiverNode->bounding_box;
        	if (!receiver_bbox.Inside(ray.o) && !receiver_bbox.Inside(ray(ray.mint)) && !receiver_bbox.Inside(ray(-ray.mint)))
//            if (!shaft->receiverNode->bounding_box.Inside(ray(ray.mint)))
        		return false;

            return shaft->lightNode->bounding_box.Inside(ray(ray.maxt));
//            return shaft->lightNode->bounding_box.IntersectP(r);
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
#ifdef SHAFT_TRY_LAST_SHAFT
                        lastHit = this;
#endif
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
        
        float Visibility(const Ray &ray, bool *set = NULL) const {
            if (!RayInShaft(ray)) {
            	if (set) *set = false;
                return 1.;
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
#ifdef SHAFT_TRY_LAST_SHAFT
                        lastHit = this;
#endif
                        return shaft->Visibility(ray);
                    } else {
                        bool s = false;
                        float v = left->Visibility(ray, &s);
                        if (s) return v;
                        return right->Visibility(ray);
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
                    ::Triangle *triangle = &*mesh.getTriangle(*tidx);

                    if (triangles.insert(static_cast<Shape *>(triangle)).second)
                        nodeRetained++;
                }
                for (Shaft::nblciter tidx = shaftTris.begin(); tidx != shaftTris.end(); tidx++) {
                    ::Triangle *triangle = &*mesh.getTriangle(*tidx);
                    
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

        void setUseProbVis(bool useProbVis, RNG *rng = NULL, const string * const type = NULL,
                           uint32_t preprocess_regular_prims = 8, uint32_t preprocess_regular_lights = 4) {
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
                shaft->initProbVis(useProbVis, rng, type, preprocess_regular_prims, preprocess_regular_lights);
            } else {
                left->setUseProbVis(useProbVis, rng, type, preprocess_regular_prims, preprocess_regular_lights);
                right->setUseProbVis(useProbVis, rng, type, preprocess_regular_prims, preprocess_regular_lights);
            }
        }
        
        operator bool() const {
            return isValid();
        }
        
        bool isValid() const {
            return is_leaf || (right && left);
        }
        
    private:
        pthread_mutex_t mutex;
                     
        typedef std::vector<Reference<Primitive> > primlist;
        typedef primlist::iterator primiter;
        
        void doSplit() {
            pthread_mutex_lock(&mutex);
            
            // we just acquired the lock, check if not set already
            if (state != SHAFT_UNSET) {
                pthread_mutex_unlock(&mutex);
                return;
            }
            
            Shaft &shaft = *this->shaft;
            Reference<ElementTreeNode> &receiver = shaft.receiverNode,
                                       &light = shaft.lightNode;
            
            if (shaft.isLeaf()) {
                is_leaf = true;
                state = SHAFT_UNDECIDED;
                pthread_mutex_unlock(&mutex);
                
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
                    split_light = (receiver->bounding_box[1][axis] - receiver->bounding_box[0][axis])
                                < (light->bounding_box[1][axis] - light->bounding_box[0][axis]);
                } else {
                    split_light = receiver->bounding_box.Extent().LengthSquared()
                                < receiver->bounding_box.Extent().LengthSquared();
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
            
            state = SHAFT_UNDECIDED;
            pthread_mutex_unlock(&mutex);
        }
        
        ShaftState getState() const {
            if (shaft->empty()) {
                return SHAFT_EMPTY;
            }
            
            return SHAFT_UNSET;
        }
    };

    bool ShaftAccel::Intersect(const Ray &ray, Intersection *isect) const {
        return fallback_accel->Intersect(ray, isect);
    }

    bool ShaftAccel::IntersectP(const Ray &ray) const {
        log::ShaftAccelIntersectP();
#ifdef SHAFT_TRY_LAST_SHAFT
        if (lastHit != NULL && lastHit->RayInShaft(ray)) {
            return lastHit->IntersectP(ray, showShafts);
        }
#endif
        return shaft_tree->IntersectP(ray, showShafts);
    }

    float ShaftAccel::Visibility(const Ray &ray) const {
        log::ShaftAccelIntersectP();
#ifdef SHAFT_TRY_LAST_SHAFT
        bool set = false;
        float val = lastHit == NULL ? -1 : lastHit->Visibility(ray, &set);
        if (set) {
            return val;
        }
#endif
        return shaft_tree->Visibility(ray);
    }

    ElementTreeNodeSplitType findSplitType(const string &str) {
        if (str == "center" || str == "CENTER")
            return CENTER;
        if (str == "mean" || str == "MEAN")
            return MEAN;
        if (str == "median" || str == "MEDIAN")
            return MEDIAN;

        Warning("Illegal split type for element tree nodes: %s, defaulting to MEAN", str.c_str());
        return MEAN;
    }

    ShaftAccel::ShaftAccel(const prim_list &primitives, const shape_list &lights, const ParamSet &ps)
                           //const string * const probVisType, uint32_t preprocess_grid_prims, uint32_t preprocess_grid_lights)
    :   receiver_tree(
                    new ElementTree(primitives, (uint32_t)ps.FindOneInt("receiver_treshold", 15),
                                    findSplitType(ps.FindOneString("receiver_split", ps.FindOneString("node_split", "mean"))))
        ),
        light_tree(
                   new ElementTree(lights, (uint32_t)ps.FindOneInt("light_treshold", 15),
                                   findSplitType(ps.FindOneString("light_split", ps.FindOneString("node_split", "mean"))))
        ),
        showShafts(ps.FindOneBool("draw_shafts", false)),
        shaftPoint(ps.FindOnePoint("shaft_point", Point(0, 0, 0)))
    {
        prim = *primitives.begin();

        // set bounding box, create root shaft
        bounding_box = Union(
                   receiver_tree->root_node->bounding_box,
                   light_tree->root_node->bounding_box
        );
        shaft_tree = new ShaftTreeNode(
                   Shaft::constructInitialShaft(receiver_tree->root_node, light_tree->root_node)
        );

        // logging: build started
        Timer buildTimer;
        buildTimer.Start();
        log::ShaftsInitStarted();

        // split the shaft tree and the two node trees
        shaft_tree->split();

        // logging: build ended
        log::ShaftsInitEnded();
        log::ShaftSaveBuildTime(buildTimer.Time());

        // create fallback accelerator, for use in Intersect(const Ray &,Intersection *)
        BVHAccelCreator fallbackCreator(primitives, receiver_tree->mesh);
        if (showShafts) {
            shaft_tree->showShaft(shaftPoint, fallbackCreator);
        }
        fallback_accel = fallbackCreator.createAccel();

        // start timer
        Timer probvisInitTimer;
        probvisInitTimer.Start();

        // initialise probvis, or not
        bool useProbVis = ps.FindOneBool("use_probvis", !showShafts);
        if (!useProbVis) {
            shaft_tree->setUseProbVis(false);
        } else {
            string probVisType = ps.FindOneString("probvis_type", "niels");
            uint32_t preprocess_grid_prims = (uint32_t)ps.FindOneInt("preprocess_regular_prims", 8);
            uint32_t preprocess_grid_lights = (uint32_t)ps.FindOneInt("preprocess_regular_lights", 4);
            RNG *rng = new RNG;
            shaft_tree->setUseProbVis(true, rng, &probVisType, preprocess_grid_prims, preprocess_grid_lights);

            // set stratification, or not
            int nbSamples = ps.FindOneInt("strat_samples", -1);
            if (nbSamples > 0) {
                ProbabilisticVisibilityCalculator::InitStratification(nbSamples);
            } else {
                ProbabilisticVisibilityCalculator::InitStratification(0);
            }
        }

        // end timer
        log::ShaftSaveInitTime(probvisInitTimer.Time());

        ps.ReportUnused();
    }
    
    ShaftAccel::~ShaftAccel() {
        delete shaft_tree;
        delete fallback_accel;
    }
    
    ShaftAccel *createShaftAccel(const std::vector<Reference<Primitive> > &receivers,
                                 const std::vector<Reference<Shape> > &lights,
                                 const ParamSet &ps) {
        return new ShaftAccel(receivers, lights, ps);
    }
    
} // namespace shaft
