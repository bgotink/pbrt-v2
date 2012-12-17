//
//  log.h
//  pbrt
//
//  Created by Bram Gotink on 05/12/12.
//
//

#ifndef pbrt_log_h
#define pbrt_log_h

#include "memory.h"

namespace shaft {

#if true
#define SHAFT_LOG

extern AtomicInt64 nb_intersect_operations;
extern AtomicInt64 nb_intersect_done;
extern AtomicInt64 nb_no_intersected_shaft;
extern AtomicInt64 nb_node_intersect_done;

extern AtomicInt64 nb_shaft_blocked;
extern AtomicInt64 nb_shaft_empty;
extern AtomicInt64 nb_shaftaccel_intersectp;
    
extern AtomicInt64 nb_leave_shafts;
extern AtomicInt64 nb_total_prims_in_leaves;
extern AtomicInt64 nb_total_prims_in_leave_nodes;
extern AtomicInt64 nb_total_points_in_leave_nodes;
extern AtomicInt64 nb_total_depth;
    
inline void ShaftStartIntersectP() {
    AtomicAdd(&nb_intersect_done, 1);
}

inline void ShaftIntersectTest() {
    AtomicAdd(&nb_intersect_operations, 1);
}

inline void ShaftNotIntersected() {
    AtomicAdd(&nb_no_intersected_shaft, 1);
}
    
inline void ShaftNodeIntersectionTest() {
    AtomicAdd(&nb_node_intersect_done, 1);
}
    
inline void ShaftBlocked() {
    AtomicAdd(&nb_shaft_blocked, 1);
}
    
inline void ShaftEmpty() {
    AtomicAdd(&nb_shaft_empty, 1);
}
    
inline void ShaftAccelIntersectP() {
    AtomicAdd(&nb_shaftaccel_intersectp, 1);
}
    
inline void ShaftLeafCreated(uint32_t nbPrims, uint32_t nbPoints, uint32_t nbPrimsInShaft, uint32_t depth) {
    AtomicAdd(&nb_leave_shafts, 1);
    AtomicAdd(&nb_total_points_in_leave_nodes, nbPoints);
    AtomicAdd(&nb_total_prims_in_leave_nodes, nbPrims);
    AtomicAdd(&nb_total_prims_in_leaves, nbPrimsInShaft);
    AtomicAdd(&nb_total_depth, depth);
}
    
inline void ShaftLogResult() {
    fprintf(stderr, "# ray intersectp tests done: %lld\n"
            "# real intersect operations @ shaft: %lld\n"
            "# intersects not done by shaft: %lld\n"
            "# real intersect operations @ nodes: %lld\n",
        nb_intersect_done, nb_intersect_operations,
            nb_no_intersected_shaft, nb_node_intersect_done);
    
    fprintf(stderr, "\n"
            "# shafts blocked: %lld\n"
            "# shafts empty: %lld\n"
            "# ShaftAccel::IntersectP: %lld\n",
            nb_shaft_blocked, nb_shaft_empty,
            nb_shaftaccel_intersectp);
    
    fprintf(stderr, "\n"
            "# leave shafts: %lld\n"
            "# nb prims in leave shafts: %lld (avg. %f)\n"
            "# nb prims in leave nodes: %lld (avg. %f)\n"
            "# nb points in leave nodes: %lld (avg. %f)\n"
            "avg. depth: %f\n",
            nb_leave_shafts,
            nb_total_prims_in_leaves, static_cast<double>(nb_total_prims_in_leaves) / static_cast<double>(nb_leave_shafts),
            nb_total_prims_in_leave_nodes, static_cast<double>(nb_total_prims_in_leave_nodes) / static_cast<double>(nb_leave_shafts),
            nb_total_points_in_leave_nodes, static_cast<double>(nb_total_points_in_leave_nodes) / static_cast<double>(nb_leave_shafts),
            static_cast<float>(nb_total_depth) / static_cast<double>(nb_leave_shafts));
}

#else
    
#define ShaftStartIntersectP()
#define ShaftIntersectTest()
#define ShaftNotIntersected()
#define ShaftLogResult()
#define ShaftNodeIntersectionTest()
#define ShaftBlocked()
#define ShaftEmpty()
#define ShaftAccelIntersectP()
#define ShaftLeafCreated(uint32_t a, uint32_t b, uint32_t c, const Shaft &s)

#endif

}

#endif
