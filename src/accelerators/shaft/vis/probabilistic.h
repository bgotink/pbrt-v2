//
//  probabilistic.h
//  pbrt
//
//  Created by Bram Gotink on 15/08/13.
//
//

#ifndef pbrt_probabilistic_h
#define pbrt_probabilistic_h

#include "visibility.h"

namespace shaft { namespace vis {

    class  ProbabilisticVisibilityCalculator : public VisibilityCalculator {
    private:
        const RNG &rng;

    public:
        typedef VisibilityCalculator::nbllist nbllist;
        typedef VisibilityCalculator::nblciter nblciter;

    private:
        const shaft::Mesh &mesh;
        const Reference<Triangle> mostBlockingOccluder;
        const trislist triangles;

    protected:
        const float mostBlockingOccluderBlocking;
        const float p_c, p_a, p_b;

        virtual float evaluate(const Ray &ray, float p) const = 0;
        bool hitsMostBlocking(const Ray &ray) const;
        bool hitsOtherOccluder(const Ray &ray) const;

        inline bool vis_a(const Ray &ray) const {
            return !hitsMostBlocking(ray);
        }

        inline bool vis_b(const Ray &ray) const {
            return !hitsOtherOccluder(ray);
        }

    public:
        ProbabilisticVisibilityCalculator(const shaft::Mesh &mesh, const Reference<Triangle> &mostBlockingOccluder, const nbllist &triangles, const RNG &rng, float mostBlockingOccluderBlocking);

        virtual float Visibility(const Ray &ray) const;
        static void InitStratification(const unsigned int nbSamples);
        virtual uint64_t memsize() const;
    };

} }

#endif
