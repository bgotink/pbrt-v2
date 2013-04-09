//
//  visibility_impl.h
//  pbrt
//
//  Created by Bram Gotink on 27/12/12.
//
//

#ifndef pbrt_visibility_impl_h
#define pbrt_visibility_impl_h

#include "visibility.h"

namespace shaft { namespace vis {
    
    class BjornProbVisCalculator : public ProbabilisticVisibilityCalculator {
    protected:
        virtual float evaluate(const Ray &ray, float p) const;
    public:
        BjornProbVisCalculator(const Mesh &mesh, const Reference<shaft::Triangle> &mostBlockingOccluder, const nbllist &triangles, const RNG &rng, float mostBlockingOccluderBlocking);
    };
    
    class BramProbVisCalculator : public ProbabilisticVisibilityCalculator {
    protected:
        virtual float evaluate(const Ray &ray, float p) const;
    public:
        BramProbVisCalculator(const Mesh &mesh, const Reference<shaft::Triangle> &mostBlockingOccluder, const nbllist &triangles, const RNG &rng, float mostBlockingOccluderBlocking);
    };
    
    class NielsProbVisCalculator : public ProbabilisticVisibilityCalculator {
    protected:
        virtual float evaluate(const Ray &ray, float p) const;
    public:
        NielsProbVisCalculator(const Mesh &mesh, const Reference<shaft::Triangle> &mostBlockinOccluder, const nbllist &triangles,
                               const RNG &rng, float mostBlockingOccluderBlocking);
    };
    
}}

#endif
