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
        
#   define P_A   p_a
#   define P_B   p_b
#   define ALPHA P_A
#   define BETA  P_B

#   define P_C   p_c
#   define GAMMA P_C
    
    BjornProbVisCalculator::BjornProbVisCalculator(const Mesh &mesh, const Reference<Triangle> &mostBlockingOccluder, const nbllist &triangles, const RNG &rng, float mostBlockingOccluderBlocking)
                    : ProbabilisticVisibilityCalculator(mesh, mostBlockingOccluder, triangles, rng, mostBlockingOccluderBlocking)
    {
    }
    
    // a*b = a + b + (1 - a)*(1 - b) - 1
    float BjornProbVisCalculator::evaluate(const Ray &ray, float p) const {
        if (p < P_A) {
            log::ProbVis_pa();
            // (Vis_A - alpha) / p_A
            
            if (vis_a(ray)) {
#ifdef SHAFT_LOG_VISIBILITY_ALL
                log::ProbVis_pa_result((1. - ALPHA) / P_A, true, vis_b(ray));
#else
                log::ProbVis_pa_noHit();
#endif
                return (1. - ALPHA) / P_A;
            }

#ifdef SHAFT_LOG_VISIBILITY_ALL
            log::ProbVis_pa_result(-ALPHA / P_A, false, false); // second argument doesn't matter here
#endif
            return (0. - ALPHA) / P_A;
        }
        else if (p < (1-P_C)) {
            log::ProbVis_pb();
            // (Vis_B - beta) / p_B
            
            if (vis_b(ray)) {
#ifdef SHAFT_LOG_VISIBILITY_ALL
                log::ProbVis_pb_result((1. - BETA) / P_B, vis_a(ray), true);
#else
                log::ProbVis_pb_noHit();
#endif
                return (1. - BETA) / P_B;
            }

#ifdef SHAFT_LOG_VISIBILITY_ALL
            log::ProbVis_pb_result(-BETA / P_B, false, false); // first argument doesn't matter
#endif
            return (0. - BETA) / P_B;
        }
        else {
            log::ProbVis_pc();
            // ((!Vis_A * !Vis_B) - (1 - alpha - beta)) / (1 - p_A - p_B)

#ifdef SHAFT_LOG_VISIBILITY_ALL
            bool va, vb;
            if ((va = vis_a(ray)) | (vb = vis_b(ray))) {
                log::ProbVis_pc_result(-GAMMA / P_C, va,vb);
#else
            if (vis_a(ray) || vis_b(ray)) {
                log::ProbVis_pc_noHit();
#endif
                
                return (0. - GAMMA) / P_C;
            }

#ifdef SHAFT_LOG_VISIBILITY_ALL
            log::ProbVis_pc_result((1. - GAMMA) / P_C, false,false);
#endif
            return (1. - GAMMA) / P_C;
        }
    }
    
    BramProbVisCalculator::BramProbVisCalculator(const Mesh &mesh, const Reference<Triangle> &mostBlockingOccluder, const nbllist &triangles, const RNG &rng, float mostBlockingOccluderBlocking)
                    : ProbabilisticVisibilityCalculator(mesh, mostBlockingOccluder, triangles, rng, mostBlockingOccluderBlocking)
    {
    }
    
    // a*b = ( a + b - (a-b)^2 ) / 2
    float BramProbVisCalculator::evaluate(const Ray &ray, float p) const {
        if (p < P_A) {
            log::ProbVis_pa();
            
            if (vis_a(ray)) {
#ifdef SHAFT_LOG_VISIBILITY_ALL
                log::ProbVis_pa_result(.5 / P_A, true, vis_b(ray));
#else
                log::ProbVis_pa_noHit();
#endif
                return .5 / P_A;
            }

#ifdef SHAFT_LOG_VISIBILITY_ALL
            log::ProbVis_pa_result(0, false, false);
#endif
            return 0;
        } else if (p < (1- P_C)) {
            log::ProbVis_pb();
            
            if (vis_b(ray)) {
#ifdef SHAFT_LOG_VISIBILITY_ALL
                log::ProbVis_pb_result(.5 / P_B, vis_a(ray), true);
#else
                log::ProbVis_pb_noHit();
#endif
                return .5 / P_B;
            }

#ifdef SHAFT_LOG_VISIBILITY_ALL
            log::ProbVis_pb_result(0, false, false);
#endif
            return 0;
        } else {
            log::ProbVis_pc();
            
            int total = 0;
#ifdef SHAFT_LOG_VISIBILITY_ALL
            bool va = true, vb = true;
#else
            bool missed = true;
#endif
            if (!vis_a(ray)) {
#ifdef SHAFT_LOG_VISIBILITY_ALL
                va = false;
#else
                missed = false;
#endif
                total++;
            }
            if (!vis_b(ray)) {
#ifdef SHAFT_LOG_VISIBILITY_ALL
                vb = false;
#else
                missed = false;
#endif
                total--;
            }
#ifdef SHAFT_LOG_VISIBILITY_ALL
            log::ProbVis_pc_result(- total * total / (2. * P_C), va, vb);
#else
            if (missed) log::ProbVis_pc_noHit();
#endif
            return - total * total / (2. * P_C);
        }
    }
    
    NielsProbVisCalculator::NielsProbVisCalculator(const Mesh &mesh, const Reference<Triangle> &mostBlockingOccluder, const nbllist &triangles, const RNG &rng, float mostBlockingOccluderBlocking)
    : ProbabilisticVisibilityCalculator(mesh, mostBlockingOccluder, triangles, rng, mostBlockingOccluderBlocking)
    {
    }

    // a*b = ( (a+b)^8 - a - b ) / 254
#   define POW_2_8  256.
    float NielsProbVisCalculator::evaluate(const Ray &ray, float p) const {
        if (p < P_A) {
            log::ProbVis_pa();
            
            if (vis_a(ray)) {
#ifdef SHAFT_LOG_VISIBILITY_ALL
                log::ProbVis_pa_result(- 1. / (254. * P_A), true, vis_b(ray));
#else
                log::ProbVis_pa_noHit();
#endif
                
                return - 1. / (254. * P_A);
            }

#ifdef SHAFT_LOG_VISIBILITY_ALL
            log::ProbVis_pa_result(0, false, false);
#endif
            return 0.;
        } else if (p < (1 - P_C)) {
            log::ProbVis_pb();
            
            if (vis_b(ray)) {
#ifdef SHAFT_LOG_VISIBILITY_ALL
                log::ProbVis_pb_result(- 1. / (254. * P_B), vis_a(ray), true);
#else
                log::ProbVis_pb_noHit();
#endif
                
                return - 1. / (254. * P_B);
            }

#ifdef SHAFT_LOG_VISIBILITY_ALL
            log::ProbVis_pb_result(0, false, false);
#endif
            return 0.;
        } else {
            log::ProbVis_pc();
            
            if (vis_a(ray)) {
                if (vis_b(ray)) {
                    // a+b == 2
#ifdef SHAFT_LOG_VISIBILITY_ALL
                    log::ProbVis_pc_result(POW_2_8 / (254. * P_C), true, true);
#else
                    log::ProbVis_pc_noHit();
#endif
                    return POW_2_8 / (254. * P_C);
                } else {
                    // a+b == 1
#ifdef SHAFT_LOG_VISIBILITY_ALL
                    log::ProbVis_pc_result(1. / (254. * P_C), true, false);
#endif
                    return 1. / (254. * P_C);
                }
            } else {
                if (vis_b(ray)) {
                    // a+b == 1
#ifdef SHAFT_LOG_VISIBILITY_ALL
                    log::ProbVis_pc_result(1. / (254. * P_C), false, true);
#endif
                    return 1. / (254. * P_C);
                } else {
                    // a+b == 0
#ifdef SHAFT_LOG_VISIBILITY_ALL
                    log::ProbVis_pc_result(0, false, false);
#endif
                    return 0.;
                }
            }
        }
    }
    
}
}
