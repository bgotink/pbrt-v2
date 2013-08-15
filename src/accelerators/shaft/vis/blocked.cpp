//
//  blocked.cpp
//  pbrt
//
//  Created by Bram Gotink on 10/06/13.
//
//

#include "visibility.h"
#include "../log.h"

namespace shaft { namespace vis {

    class BlockedVisiblityCalculator : public VisibilityCalculator {
        virtual float Visibility(const Ray &ray) const {
            return 0.f;
        }
        virtual ~BlockedVisiblityCalculator() {}

        virtual uint64_t memsize() const { return sizeof(BlockedVisiblityCalculator); }
    };

    VisibilityCalculator *createBlockedVisibilityCalculator() {
        ::shaft::log::ShaftBlocked();
        return new BlockedVisiblityCalculator;
    }
}}
