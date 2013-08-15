//
//  probabilistic.cpp
//  
//
//  Created by Bram Gotink on 24/12/12.
//
//

#include "probabilistic.h"
#include "probvis_impl.h"

#define max(a,b) (a > b ? a : b)
#define min(a,b) (a < b ? a : b)

namespace shaft { namespace vis {

    static unsigned int nbSamples(0);
    static float sampleStep(0/0.f);

    /* NOT threadsafe */
    struct Stratifier {

        Stratifier() : current(0) {}

        // precondition: val \in [0,1)
        // postcondition: val \in [0,1)
        float stratify(const float val) const {
            if (nbSamples == 0) return val;

#if !defined(PBRT_CPP11) && !defined(__GCC__)
            Fatal("No stratification supported due to this platform lacking thread_local support");
#endif

            if (current == nbSamples) current = 0;
            return (val + current++) * sampleStep;
        }

    private:
        mutable unsigned int current;
    };

#if defined(PBRT_CPP11)
    thread_local Stratifier stratifier;
#elif defined(__GCC__)
    __thread Stratifier stratifier;
#else
#   pragma message("Warning: no thread-local support detected! Stratifier use unsupported")
    Stratifier stratifier;
#endif
    
    ProbabilisticVisibilityCalculator::ProbabilisticVisibilityCalculator(const shaft::Mesh &mesh, const Reference<Triangle> &mostBlockingOccluder, const nbllist &triangles, const RNG &rng, float mostBlockingOccluderBlocking) : rng(rng), mesh(mesh), mostBlockingOccluder(mostBlockingOccluder), triangles(VisibilityCalculator::getTriangles(mesh, triangles)), mostBlockingOccluderBlocking(mostBlockingOccluderBlocking),
        p_c(.2), p_a((1-p_c)*min(max(mostBlockingOccluderBlocking, .3), .9)), p_b(1-p_c-p_a)
    {
    }
    
    float ProbabilisticVisibilityCalculator::Visibility(const Ray &ray) const {
        return evaluate(ray, stratifier.stratify(rng.RandomFloat()));
    }
    
    bool ProbabilisticVisibilityCalculator::hitsMostBlocking(const Ray &ray) const {
        log::ShaftIntersectTest();
        log::ShaftAddIntersect();
        
        return mostBlockingOccluder->IntersectP(ray);
    }
    
    bool ProbabilisticVisibilityCalculator::hitsOtherOccluder(const Ray &ray) const {
        for (trisciter t = triangles.begin(); t != triangles.end(); t++) {
            log::ShaftIntersectTest();
            log::ShaftAddIntersect();
            
            if ((*t)->IntersectP(ray)) {
                return true;
            }
        }
        return false;
    }

    void ProbabilisticVisibilityCalculator::InitStratification(const unsigned int samples) {
        nbSamples = samples;
        sampleStep = 1 / static_cast<float>(samples);
    }

    uint64_t ProbabilisticVisibilityCalculator::memsize() const {
        return static_cast<uint64_t>(sizeof(ProbabilisticVisibilityCalculator))
                + triangles.size() * sizeof(trislist::value_type);
    }
    
    VisibilityCalculator *createProbabilisticVisibilityCalculator(const string &type, const Mesh &mesh, const Reference<Triangle> &mostBlockingOccluder, const ProbabilisticVisibilityCalculator::nbllist &triangles, const RNG &rng, float mostBlockingOccluderBlocking) {
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
