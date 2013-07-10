//
//  probabilistic.cpp
//  
//
//  Created by Bram Gotink on 24/12/12.
//
//

#include "visibility.h"
#include "visibility_impl.h"

namespace shaft { namespace vis {
    
    ProbabilisticVisibilityCalculator::ProbabilisticVisibilityCalculator(const shaft::Mesh &mesh, const Reference<Triangle> &mostBlockingOccluder, const nbllist &triangles, const RNG &rng, float mostBlockingOccluderBlocking) : rng(rng), mesh(mesh), mostBlockingOccluder(mostBlockingOccluder), triangles(VisibilityCalculator::getTriangles(mesh, triangles)), mostBlockingOccluderBlocking(mostBlockingOccluderBlocking) {
    }
    
    float ProbabilisticVisibilityCalculator::Visibility(const Ray &ray) const {
        return evaluate(ray, rng.RandomFloat());
    }
    
    bool ProbabilisticVisibilityCalculator::hitsMostBlocking(const Ray &ray) const {
        log::ShaftIntersectTest();
        log::ShaftAddIntersect();
        
        return mostBlockingOccluder->IntersectP(ray);
    }
    
    bool ProbabilisticVisibilityCalculator::hitsOtherOccluder(const Ray &ray) const {
        Reference<Triangle> triangle;
        for (trisciter t = triangles.begin(); t != triangles.end(); t++) {
            triangle = *t;
            if (&*triangle == &*mostBlockingOccluder)
                continue;
            
            log::ShaftIntersectTest();
            log::ShaftAddIntersect();
            
            if (triangle->IntersectP(ray)) {
                return true;
            }
        }
        return false;
    }
    
    ProbabilisticVisibilityCalculator *createProbabilisticVisibilityCalculator(const string &type, const Mesh &mesh, const Reference<Triangle> &mostBlockingOccluder, const ProbabilisticVisibilityCalculator::nbllist &triangles, const RNG &rng, float mostBlockingOccluderBlocking) {
        if (type == "bram")
            return new BramProbVisCalculator(mesh, mostBlockingOccluder, triangles, rng, mostBlockingOccluderBlocking);
        
        if (type == "niels")
            return new NielsProbVisCalculator(mesh, mostBlockingOccluder, triangles, rng, mostBlockingOccluderBlocking);
        
        if (type != "bjorn") {
            Error("Unknown type of ProbabilisticVisibilityCalculator: %s, using bjorn instead.", type.c_str());
        }
        return new BjornProbVisCalculator(mesh, mostBlockingOccluder, triangles, rng, mostBlockingOccluderBlocking);
    }
    
}}
