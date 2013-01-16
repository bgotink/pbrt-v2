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
#include "../Mesh.h"
#include "../tree.h"

namespace shaft {
namespace vis {
    
    class VisibilityCalculator {
    public:
        virtual float Visibility(const Ray &ray) const = 0;
        virtual ~VisibilityCalculator();
    };
    
    class ExactVisibilityCalculator : public VisibilityCalculator {
        typedef std::list<unsigned int> nbllist;
        typedef nbllist::const_iterator nblciter;
        
        const shaft::Mesh &mesh;
        const nbllist &triangles;
        const shaft::ElementTreeNode &receiver_node;
        
    public:
        ExactVisibilityCalculator(const shaft::Mesh &mesh, const nbllist &triangles, const Reference<ElementTreeNode> &receiver_node);
        
        virtual float Visibility(const Ray &ray) const;
    };
    
    class  ProbabilisticVisibilityCalculator : public VisibilityCalculator {
    private:
        const RNG &rng;
        
    public:
        typedef std::list<unsigned int> nbllist;
        typedef nbllist::const_iterator nblciter;
        
    protected:
        const shaft::Mesh &mesh;
        const Reference<shaft::Triangle> mostBlockingOccluder;
        const nbllist triangles;
        
        virtual float evaluate(const Ray &ray, float p) const = 0;
        
    public:
        ProbabilisticVisibilityCalculator(const shaft::Mesh &mesh, const Reference<shaft::Triangle> &mostBlockingOccluder, const nbllist &triangles, const RNG &rng);
        
        virtual float Visibility(const Ray &ray) const;
    };
    
    ProbabilisticVisibilityCalculator *createProbabilisticVisibilityCalculator(const string &type, const shaft::Mesh &mesh, const Reference<shaft::Triangle> &mostBlockingOccluder, const ProbabilisticVisibilityCalculator::nbllist &triangles, const RNG &rng);
}
}

#endif /* defined(__pbrt__visibility__) */
