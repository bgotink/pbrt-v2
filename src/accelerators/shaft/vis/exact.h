//
//  exact.h
//  pbrt
//
//  Created by Bram Gotink on 15/08/13.
//
//

#ifndef pbrt_exact_h
#define pbrt_exact_h

#include "visibility.h"

namespace shaft { namespace vis {

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
        virtual uint64_t memsize() const;
    };

} }

#endif
