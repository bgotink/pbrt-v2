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
        
#   define P_A   ((mostBlockingOccluderBlocking)*(1-P_C))
#   define P_B   ((1-mostBlockingOccluderBlocking)*(1-P_C))
    //#   define ALPHA    (1.f / 3.f)
    //#   define BETA     (1.f / 3.f)
#   define ALPHA P_A
#   define BETA  P_B

#   define P_C   0.3
#   define GAMMA (1 - ALPHA - BETA)
    
    BjornProbVisCalculator::BjornProbVisCalculator(const Mesh &mesh, const Reference<shaft::Triangle> &mostBlockingOccluder, const nbllist &triangles, const RNG &rng, float mostBlockingOccluderBlocking)
                    : ProbabilisticVisibilityCalculator(mesh, mostBlockingOccluder, triangles, rng, mostBlockingOccluderBlocking)
    {
    }
    
    // a*b = a + b + (1 - a)*(1 - b) - 1
    float BjornProbVisCalculator::evaluate(const Ray &ray, float p) const {
        if (p < P_A) {
            log::ProbVis_pa();
            // (Vis_A - alpha) / p_A
            
            if (vis_a(ray)) {
                log::ProbVis_pa_noHit();
                return (1. - ALPHA) / P_A;
            }
            
            return (0. - ALPHA) / P_A;
        }
        else if (p < (1-P_C)) {
            log::ProbVis_pb();
            // (Vis_B - beta) / p_B
            
            if (vis_b(ray)) {
                log::ProbVis_pb_noHit();
                return (1. - BETA) / P_B;
            }
            
            return (0. - BETA) / P_B;
        }
        else {
            log::ProbVis_pc();
            // ((!Vis_A * !Vis_B) - (1 - alpha - beta)) / (1 - p_A - p_B)
            
            if (vis_a(ray) || vis_b(ray)) {
                log::ProbVis_pc_noHit();
                
                return (0. - GAMMA) / P_C;
            }
            
            return (1. - GAMMA) / P_C;
        }
    }
    
    BramProbVisCalculator::BramProbVisCalculator(const Mesh &mesh, const Reference<shaft::Triangle> &mostBlockingOccluder, const nbllist &triangles, const RNG &rng, float mostBlockingOccluderBlocking)
                    : ProbabilisticVisibilityCalculator(mesh, mostBlockingOccluder, triangles, rng, mostBlockingOccluderBlocking)
    {
    }
    
    // a*b = ( a + b - (a-b)^2 ) / 2
    float BramProbVisCalculator::evaluate(const Ray &ray, float p) const {
        if (p < P_A) {
            log::ProbVis_pa();
            
            if (vis_a(ray)) {
                log::ProbVis_pa_noHit();
                return .5 / P_A;
            }
            
            return 0;
        } else if (p < (1- P_C)) {
            log::ProbVis_pb();
            
            if (vis_b(ray)) {
                log::ProbVis_pb_noHit();
                return .5 / P_B;
            }
            return 0;
        } else {
            log::ProbVis_pc();
            
            int total = 0;
            bool missed = true;
            if (!vis_a(ray)) {
                missed = false;
                total++;
            }
            if (!vis_b(ray)) {
                missed = false;
                total--;
            }
            if (missed) log::ProbVis_pc_noHit();
            return - total * total / (2. * P_C);
        }
    }
    
    NielsProbVisCalculator::NielsProbVisCalculator(const Mesh &mesh, const Reference<shaft::Triangle> &mostBlockingOccluder, const nbllist &triangles, const RNG &rng, float mostBlockingOccluderBlocking)
    : ProbabilisticVisibilityCalculator(mesh, mostBlockingOccluder, triangles, rng, mostBlockingOccluderBlocking)
    {
    }

    // a*b = ( (a+b)^8 - a - b ) / 254
#   define POW_2_8  256.
    float NielsProbVisCalculator::evaluate(const Ray &ray, float p) const {
        if (p < P_A) {
            log::ProbVis_pa();
            
            if (vis_a(ray)) {
                log::ProbVis_pa_noHit();
                
                return - 1. / (254. * P_A);
            }
            
            return 0.;
        } else if (p < (1 - P_C)) {
            log::ProbVis_pb();
            
            if (vis_b(ray)) {
                log::ProbVis_pb_noHit();
                
                return - 1. / (254. * P_B);
            }
            
            return 0.;
        } else {
            log::ProbVis_pc();
            
            if (vis_a(ray)) {
                if (vis_b(ray)) {
                    // a+b == 2
                    log::ProbVis_pc_noHit();
                    return POW_2_8 / (254. * P_C);
                } else {
                    // a+b == 1
                    return 1. / (254. * P_C);
                }
            } else {
                if (vis_b(ray)) {
                    // a+b == 1
                    return 1. / (254. * P_C);
                } else {
                    // a+b == 0
                    return 0.;
                }
            }
        }
    }
    
}
}
