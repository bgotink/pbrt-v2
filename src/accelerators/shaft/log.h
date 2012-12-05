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
    
inline void ShaftLogResult() {
    fprintf(stderr, "# ray intersectp tests done: %lld\n"
            "# real intersect operations @ shaft: %lld\n"
            "# intersects not done by shaft: %lld\n"
            "# real intersect operations @ nodes: %lld\n",
        nb_intersect_done, nb_intersect_operations,
            nb_no_intersected_shaft, nb_node_intersect_done);
    
    fprintf(stderr, "# shafts blocked: %lld\n"
            "# shafts empty: %lld\n"
            "# ShaftAccel::IntersectP: %lld\n",
            nb_shaft_blocked, nb_shaft_empty,
            nb_shaftaccel_intersectp);
}

#else
    
inline void ShaftStartIntersectP() {}
inline void ShaftIntersectTest() {}
inline void ShaftNotIntersected() {}
inline void ShaftLogResult() {}
inline void ShaftNodeIntersectionTest() {}
inline void ShaftBlocked() {}
inline void ShaftEmpty() {}
inline void ShaftAccelIntersectP() {

#endif

}

#endif
