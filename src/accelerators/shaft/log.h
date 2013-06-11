//
//  log.h
//  pbrt
//
//  Created by Bram Gotink on 05/12/12.
//
//

// freely configurable, SHAFT_LOG flag disables all other if disabled

#define SHAFT_LOG
#define SHAFT_SHOW_INTERSECTS
#define SHAFT_SHOW_DEPTHS
#define SHAFT_SHOW_PRIMS
#define SHAFT_SHOW_LEAFS

// end of configuration

#ifndef pbrt_log_h
#define pbrt_log_h

#ifdef SHAFT_LOG
#include "film/falsecolor.h"
#endif // defined(SHAFT_LOG)


#ifndef SHAFT_LOG
#ifdef SHAFT_SHOW_INTERSECTS
#undef SHAFT_SHOW_INTERSECTS
#endif // defined(SHAFT_SHOW_INTERSECTS)
#ifdef SHAFT_SHOW_DEPTHS
#undef SHAFT_SHOW_DEPTHS
#endif // defined(SHAFT_SHOW_DEPTHS)
#ifdef SHAFT_SHOW_PRIMS
#undef SHAFT_SHOW_PRIMS
#endif // defined(SHAFT_SHOW_PRIMS)
#ifdef SHAFT_SHOW_LEAFS
#undef SHAFT_SHOW_LEAFS
#endif // defined(SHAFT_SHOW_LEAFS)
#endif // !defined(SHAFT_LOG)

#include "memory.h"

namespace shaft { namespace log {

#ifdef SHAFT_LOG

extern AtomicUInt64 nb_intersect_operations;
extern AtomicUInt64 nb_intersect_done;
extern AtomicUInt64 nb_no_intersected_shaft;
extern AtomicUInt64 nb_node_intersect_done;

extern AtomicUInt64 nb_shaft_blocked;
extern AtomicUInt64 nb_shaft_empty;
extern AtomicUInt64 nb_shaftaccel_intersectp;
    
extern uint64_t nb_leave_shafts;
extern uint64_t nb_total_prims_in_leaves;
extern uint64_t nb_total_prims_in_leave_nodes;
extern uint64_t nb_total_points_in_leave_nodes;
extern uint64_t nb_total_depth;
extern uint64_t nb_max_points_in_leave_nodes;
    
extern AtomicUInt64 nb_pa, nb_pb, nb_pc;
extern AtomicUInt64 nb_panh, nb_pbnh, nb_pcnh;
    
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
    
inline void ProbVis_pa() {
    AtomicAdd(&nb_pa, 1);
}
    
inline void ProbVis_pb() {
    AtomicAdd(&nb_pb, 1);
}
    
inline void ProbVis_pc() {
    AtomicAdd(&nb_pc, 1);
}
    
inline void ProbVis_pa_noHit() {
    AtomicAdd(&nb_panh, 1);
}
    
inline void ProbVis_pb_noHit() {
    AtomicAdd(&nb_pbnh, 1);
}
    
inline void ProbVis_pc_noHit() {
    AtomicAdd(&nb_pcnh, 1);
}
    
inline void ShaftLeafCreated(uint64_t nbPrims, uint64_t nbPoints, uint64_t nbPrimsInShaft, uint64_t depth) {
    nb_leave_shafts++;
    
    nb_total_points_in_leave_nodes += nbPoints;
    if (nbPoints > nb_max_points_in_leave_nodes)
        nb_max_points_in_leave_nodes = nbPoints;
    
    nb_total_prims_in_leave_nodes += nbPrims;
    nb_total_prims_in_leaves += nbPrimsInShaft;
    
    nb_total_depth += depth;
}
    
void ShaftSaveBuildTime(double buildTime);
    
void ShaftLogResult();
    
#ifdef SHAFT_SHOW_DEPTHS
    extern FalseColorFilm *falseColorShafts;
#endif // defined(SHAFT_SHOW_DEPTHS)
#ifdef SHAFT_SHOW_INTERSECTS
    extern FalseColorFilm *falseColorIntersects;
#endif // defined(SHAFT_SHOW_INTERSECTS)
#ifdef SHAFT_SHOW_PRIMS
    extern FalseColorFilm *falseColorPrims;
#endif // defined(SHAFT_SHOW_PRIMS)
#ifdef SHAFT_SHOW_LEAFS
    extern Film *falseColorLeafs;
#endif // defined(SHAFT_SHOW_LEAFS)
    
    extern ParamSet filmParams;
    extern Filter *filter;
    
#if defined(PBRT_CPP11)
    extern thread_local CameraSample *cameraSample;
#elif defined(__GCC__)
    extern __thread CameraSample *cameraSample;
#else
    extern CameraSample *cameraSample;
#endif
    
#if defined(SHAFT_SHOW_LEAFS)
#if defined(PBRT_CPP11)
    extern thread_local uint32_t hitLeafId;
#elif defined(__GCC__)
    extern __thread uint32_t hitLeafId;
#else
    extern uint32_t hitLeafId;
#endif
#endif // defined(SHAFT_SHOW_LEAFS)
    
    void ShaftNewImage();
    
#ifdef SHAFT_SHOW_DEPTHS
    inline void ShaftDepth(uint64_t depth) {
        if (falseColorShafts != NULL && cameraSample != NULL) falseColorShafts->Set(*cameraSample, depth);
    }
#else
#define ShaftDepth();
#endif
    
#ifdef SHAFT_SHOW_INTERSECTS
    inline void ShaftAddIntersect(uint64_t count = 1) {
        if (falseColorIntersects != NULL && cameraSample != NULL) falseColorIntersects->Add(*cameraSample, count);
    }
#else
#define ShaftAddIntersect();
#endif
    
#ifdef SHAFT_SHOW_PRIMS
    inline void ShaftSetPrimCount(uint64_t count) {
        if (falseColorPrims != NULL && cameraSample != NULL) falseColorPrims->Set(*cameraSample, count);
    }
#else
#define ShaftSetPrimCount();
#endif
    
#define SAVE_FALSE_COLOR(image) \
    if (image != NULL) \
        image->WriteImage(); \
    delete image; \
    image = NULL;
    
    inline void ShaftSaveFalseColor() {
#ifdef SHAFT_SHOW_DEPTHS
        SAVE_FALSE_COLOR(falseColorShafts)
#endif
#ifdef SHAFT_SHOW_INTERSECTS
        SAVE_FALSE_COLOR(falseColorIntersects)
#endif
#ifdef SHAFT_SHOW_PRIMS
        SAVE_FALSE_COLOR(falseColorPrims);
#endif
#ifdef SHAFT_SHOW_LEAFS
        SAVE_FALSE_COLOR(falseColorLeafs);
#endif
    }
    
    void ShaftSaveMetaData(double timeSpent);
    
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
    
#define ProbVis_pa()
#define ProbVis_pb()
#define ProbVis_pc()
#define ProbVis_pa_noHit()
#define ProbVis_pb_noHit()
#define ProbVis_pc_noHit()
    
#define ShaftNewImage();
#define ShaftDepth();
#define ShaftAddIntersect();
#define ShaftSaveFalseColor();
#define ShaftSaveBuildTime();
#define ShaftSaveMetaData();
    
#endif

}}

#endif
