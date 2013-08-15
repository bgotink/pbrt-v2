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
    public:
        typedef std::list<unsigned int> nbllist;
        typedef nbllist::const_iterator nblciter;
        
        typedef std::list<Reference<Triangle> > trislist;
        typedef trislist::const_iterator trisciter;

    protected:
        static trislist getTriangles(const Mesh &mesh, const nbllist &tidx, const Triangle *const mostBlockingOccluder = NULL);
        
    public:
        virtual float Visibility(const Ray &ray) const = 0;
        virtual ~VisibilityCalculator();
        virtual uint64_t memsize() const = 0;
    };

    VisibilityCalculator *createBlockedVisibilityCalculator();
    VisibilityCalculator *createExactVisibilityCalculator(const shaft::Mesh &mesh,
                                                          const VisibilityCalculator::nbllist &triangles,
                                                          const Reference<ElementTreeNode> &receiver_node,
                                                          const Reference<ElementTreeNode> &light_node);
    
    VisibilityCalculator *createProbabilisticVisibilityCalculator(const string &type, const shaft::Mesh &mesh, const Reference<Triangle> &mostBlockingOccluder, const VisibilityCalculator::nbllist &triangles, const RNG &rng, float mostBlockingOccluderBlocking);
}
}

#endif /* defined(__pbrt__visibility__) */
