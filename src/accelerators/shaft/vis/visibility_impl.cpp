//
//  visibility_impl.c
//  pbrt
//
//  Created by Bram Gotink on 27/12/12.
//
//

#include "visibility_impl.h"
#include "../log.h"

namespace shaft {
namespace vis {
        
#   define P_A  (mostBlockingOccluderBlocking)*(1-P_C)
#   define P_B  (1-mostBlockingOccluderBlocking)*(1-P_C)
#   define ALPHA    1/3
#   define BETA     1/3

#   define P_C  0.3
#   define GAMMA 1 - ALPHA - BETA

    
    BjornProbVisCalculator::BjornProbVisCalculator(const Mesh &mesh, const Reference<shaft::Triangle> &mostBlockingOccluder, const nbllist &triangles, const RNG &rng, float mostBlockingOccluderBlocking)
                    : ProbabilisticVisibilityCalculator(mesh, mostBlockingOccluder, triangles, rng, mostBlockingOccluderBlocking)
    {
    }
    
    float BjornProbVisCalculator::evaluate(const Ray &ray, float p) const {
        if (p < P_A) {
            ProbVis_pa();
            // (Vis_A - alpha) / p_A
            if (hitsMostBlocking(ray)) {
                //Info("Wow, a hit (%d, %d, %d)", mostBlockingOccluder->getPoint(0), mostBlockingOccluder->getPoint(1), mostBlockingOccluder->getPoint(2));
                return - ALPHA / P_A;
            }
            ProbVis_pa_noHit();
            return (1. - ALPHA) / P_A;
        } else if (p < P_B + P_A) {
            ProbVis_pb();
            // (Vis_B - beta) / p_B
            Reference<Triangle> triangle;
            if (hitsOtherOccluder(ray))
                return - BETA / P_B;
            ProbVis_pb_noHit();
            return (1. - BETA) / P_B;
        } else {
            ProbVis_pc();
            // ((!Vis_A * !Vis_B) - (1 - alpha - beta)) / (1 - p_A - p_B)
            
            if (!hitsMostBlocking(ray)) {
                ProbVis_pc_noHit();
                return - GAMMA / P_C;
            } else {
                Reference<Triangle> triangle;
                if (hitsOtherOccluder(ray)) {
                    return 1. - GAMMA / P_C;
                }
                ProbVis_pc_noHit();
                return (- GAMMA) / P_C;
            }
        }
    }
    
    BramProbVisCalculator::BramProbVisCalculator(const Mesh &mesh, const Reference<shaft::Triangle> &mostBlockingOccluder, const nbllist &triangles, const RNG &rng, float mostBlockingOccluderBlocking)
                    : ProbabilisticVisibilityCalculator(mesh, mostBlockingOccluder, triangles, rng, mostBlockingOccluderBlocking)
    {
    }
    
    float BramProbVisCalculator::evaluate(const Ray &ray, float p) const {
        if (p < P_A) {
            ProbVis_pa();
            if (hitsMostBlocking(ray)) {
                return 0;
            }
            ProbVis_pa_noHit();
            return .5 / P_A;
        } else if (p < P_B + P_A) {
            ProbVis_pb();
            if (hitsOtherOccluder(ray)) {
                return 0;
            }
            ProbVis_pb_noHit();
            return .5 / P_B;
        } else {
            ProbVis_pc();
            int total = 0;
            bool missed = true;
            if (hitsMostBlocking(ray)) {
                missed = false;
                total++;
            }
            if (hitsOtherOccluder(ray)) {
                missed = false;
                total--;
            }
            if (missed) ProbVis_pc_noHit();
            return - total * total / (2. * P_C);
        }
    }
    
}
}
