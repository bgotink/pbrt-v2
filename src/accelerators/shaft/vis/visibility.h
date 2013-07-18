//
//  visibility.h
//  pbrt
//
//  Created by Bram Gotink on 24/12/12.
//
//

#ifndef __pbrt__visibility__
#define __pbrt__visibility__

#include "geometry.h"
#include "rng.h"
#include "../mesh.h"
#include "../tree.h"
#include "../log.h"

namespace shaft {
namespace vis {
    
    class VisibilityCalculator {
    protected:
        typedef std::list<unsigned int> nbllist;
        typedef nbllist::const_iterator nblciter;
        
        typedef std::list<Reference<Triangle> > trislist;
        typedef trislist::const_iterator trisciter;
        
        static trislist getTriangles(const Mesh &mesh, const nbllist &tidx);
        
    public:
        virtual float Visibility(const Ray &ray) const = 0;
        virtual ~VisibilityCalculator();
    };

    VisibilityCalculator *createBlockedVisibilityCalculator();
    
    class ExactVisibilityCalculator : public VisibilityCalculator {
        typedef std::list<const ::Triangle *> trisptrlist;
        typedef trisptrlist::const_iterator trisptrciter;
        
        const shaft::Mesh &mesh;
        const trisptrlist triangles;
        const trislist _triangles;
        const shaft::ElementTreeNode &receiver_node, &light_node;
        
    public:
        ExactVisibilityCalculator(const shaft::Mesh &mesh, const nbllist &triangles,
                                  const Reference<ElementTreeNode> &receiver_node,
                                  const Reference<ElementTreeNode> &light_node);
        
        virtual float Visibility(const Ray &ray) const;
    };
    
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
        const float p_c, p_a, p_b;
        const float mostBlockingOccluderBlocking;
        
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
    };
    
    ProbabilisticVisibilityCalculator *createProbabilisticVisibilityCalculator(const string &type, const shaft::Mesh &mesh, const Reference<Triangle> &mostBlockingOccluder, const ProbabilisticVisibilityCalculator::nbllist &triangles, const RNG &rng, float mostBlockingOccluderBlocking);
}
}

#endif /* defined(__pbrt__visibility__) */
