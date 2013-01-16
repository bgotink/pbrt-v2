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
        
#   define P_A  0.4
#   define P_B  0.3
#   define ALPHA    P_A
#   define BETA     P_B
        
#   define P_C  1 - P_A - P_B
#   define GAMMA 1 - ALPHA - BETA

    
    BjornProbVisCalculator::BjornProbVisCalculator(const Mesh &mesh, const Reference<shaft::Triangle> &mostBlockingOccluder, const nbllist &triangles, const RNG &rng)
                    : ProbabilisticVisibilityCalculator(mesh, mostBlockingOccluder, triangles, rng)
    {
    }
    
    float BjornProbVisCalculator::evaluate(const Ray &ray, float p) const {
        if (p < P_A) {
            ProbVis_pa();
            // (Vis_A - alpha) / p_A
            if (IntersectsTriangle(mostBlockingOccluder, mesh, ray))
                return - ALPHA / P_A;
            ProbVis_pa_noHit();
            return (1. - ALPHA) / P_A;
        } else if (p < P_B + P_A) {
            ProbVis_pb();
            // (Vis_B - beta) / p_B
            Reference<Triangle> triangle;
            for (nblciter t = triangles.begin(); t != triangles.end(); t++) {
                triangle = mesh.getTriangle(*t);
                if (&*triangle == &*mostBlockingOccluder)
                    continue;
                
                if (IntersectsTriangle(triangle, mesh, ray))
                    return - BETA / P_B;
            }
            ProbVis_pb_noHit();
            return (1. - BETA) / P_B;
        } else {
            ProbVis_pc();
            // ((!Vis_A * !Vis_B) - (1 - alpha - beta)) / (1 - p_A - p_B)
            
            if (!IntersectsTriangle(mostBlockingOccluder, mesh, ray)) {
                ProbVis_pc_noHit();
                return - GAMMA / P_C;
            } else {
                Reference<Triangle> triangle;
                for (nblciter t = triangles.begin(); t != triangles.end(); t++) {
                    triangle = mesh.getTriangle(*t);
                    if (&*triangle == &*mostBlockingOccluder)
                        continue;
                    
                    if (IntersectsTriangle(triangle, mesh, ray)) {
                        ProbVis_pc_noHit();
                        return 1. - GAMMA / P_C;
                    }
                }
                return (- GAMMA) / P_C;
            }
        }
    }
    
    BramProbVisCalculator::BramProbVisCalculator(const Mesh &mesh, const Reference<shaft::Triangle> &mostBlockingOccluder, const nbllist &triangles, const RNG &rng)
                    : ProbabilisticVisibilityCalculator(mesh, mostBlockingOccluder, triangles, rng)
    {
    }
    
    float BramProbVisCalculator::evaluate(const Ray &ray, float p) const {
        return 1;
    }
    
}
}
