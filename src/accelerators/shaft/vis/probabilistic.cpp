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
    
    ProbabilisticVisibilityCalculator::ProbabilisticVisibilityCalculator(const shaft::Mesh &mesh, const Reference<shaft::Triangle> &mostBlockingOccluder, const nbllist &triangles, const RNG &rng) : rng(rng), mesh(mesh), mostBlockingOccluder(mostBlockingOccluder), triangles(triangles) {
        Info("Most blocking occluder: (%d, %d, %d)",
             this->mostBlockingOccluder->getPoint(0), this->mostBlockingOccluder->getPoint(1), this->mostBlockingOccluder->getPoint(2));
    }
    
    float ProbabilisticVisibilityCalculator::Visibility(const Ray &ray) const {
        return evaluate(ray, rng.RandomFloat());
    }
    
    ProbabilisticVisibilityCalculator *createProbabilisticVisibilityCalculator(const string &type, const Mesh &mesh, const Reference<shaft::Triangle> &mostBlockingOccluder, const ProbabilisticVisibilityCalculator::nbllist &triangles, const RNG &rng) {
        if (type == "bram")
            return new BramProbVisCalculator(mesh, mostBlockingOccluder, triangles, rng);
        
        if (type != "bjorn") {
            Error("Unknown type of ProbabilisticVisibilityCalculator: %s, using bjorn instead.", type.c_str());
        }
        return new BjornProbVisCalculator(mesh, mostBlockingOccluder, triangles, rng);
    }
    
}}