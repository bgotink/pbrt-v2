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
    
    ProbabilisticVisibilityCalculator::ProbabilisticVisibilityCalculator(const shaft::Mesh &mesh, const Reference<shaft::Triangle> &mostBlockingOccluder, const nbllist &triangles, const RNG &rng, float mostBlockingOccluderBlocking) : rng(rng), mesh(mesh), mostBlockingOccluder(mostBlockingOccluder), triangles(triangles), mostBlockingOccluderBlocking(mostBlockingOccluderBlocking) {
    }
    
    float ProbabilisticVisibilityCalculator::Visibility(const Ray &ray) const {
        return evaluate(ray, rng.RandomFloat());
    }
    
    bool ProbabilisticVisibilityCalculator::hitsMostBlocking(const Ray &ray) const {
        return IntersectsTriangle(mostBlockingOccluder, mesh, ray);
    }
    
    bool ProbabilisticVisibilityCalculator::hitsOtherOccluder(const Ray &ray) const {
        Reference<Triangle> triangle;
        for (nblciter t = triangles.begin(); t != triangles.end(); t++) {
            triangle = mesh.getTriangle(*t);
            if (&*triangle == &*mostBlockingOccluder)
                continue;
            
            if (IntersectsTriangle(triangle, mesh, ray)) {
                return true;
            }
        }
        return false;
    }
    
    ProbabilisticVisibilityCalculator *createProbabilisticVisibilityCalculator(const string &type, const Mesh &mesh, const Reference<shaft::Triangle> &mostBlockingOccluder, const ProbabilisticVisibilityCalculator::nbllist &triangles, const RNG &rng, float mostBlockingOccluderBlocking) {
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