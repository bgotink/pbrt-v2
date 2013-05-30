//
//  log.h
//  pbrt
//
//  Created by Bram Gotink on 05/12/12.
//
//

#ifndef pbrt_log_h
#define pbrt_log_h


#define SHAFT_LOG
#define SHAFT_SHOW_INTERSECTS
#define SHAFT_SHOW_DEPTHS


#ifdef SHAFT_LOG
#include "film/falsecolor.h"
#endif


#include "memory.h"

namespace shaft { namespace log {

#ifdef SHAFT_LOG

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
    
extern AtomicInt64 nb_pa, nb_pb, nb_pc;
extern AtomicInt64 nb_panh, nb_pbnh, nb_pcnh;
    
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
    
inline void ShaftLeafCreated(uint32_t nbPrims, uint32_t nbPoints, uint32_t nbPrimsInShaft, uint32_t depth) {
    AtomicAdd(&nb_leave_shafts, 1);
    AtomicAdd(&nb_total_points_in_leave_nodes, nbPoints);
    AtomicAdd(&nb_total_prims_in_leave_nodes, nbPrims);
    AtomicAdd(&nb_total_prims_in_leaves, nbPrimsInShaft);
    AtomicAdd(&nb_total_depth, depth);
}
    
void ShaftSaveBuildTime(double buildTime);
    
void ShaftLogResult();
    
#ifdef SHAFT_SHOW_DEPTHS
    extern FalseColorFilm *falseColorShafts;
#endif
#ifdef SHAFT_SHOW_INTERSECTS
    extern FalseColorFilm *falseColorIntersects;
#endif
    
    extern ParamSet filmParams;
    extern Filter *filter;
    
#if defined(PBRT_CPP11)
    extern thread_local CameraSample *cameraSample;
#elif defined(__GCC__)
    extern __thread CameraSample *cameraSample;
#else
    extern CameraSample *cameraSample;
#endif
    
    inline void ShaftNewImage() {
#ifdef SHAFT_SHOW_DEPTHS
        falseColorShafts = ::CreateFalseColorFilm("depths", filmParams, filter);
#endif
#ifdef SHAFT_SHOW_INTERSECTS
        falseColorIntersects = ::CreateFalseColorFilm("intersects", filmParams, filter);
#endif
    }
    
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
    
    inline void ShaftSaveFalseColor() {
#ifdef SHAFT_SHOW_DEPTHS
        if (falseColorShafts != NULL)
            falseColorShafts->WriteImage();
        delete falseColorShafts;
        falseColorShafts = NULL;
#endif
#ifdef SHAFT_SHOW_INTERSECTS
        if (falseColorIntersects != NULL)
            falseColorIntersects->WriteImage();
        delete falseColorIntersects;
        falseColorIntersects = NULL;
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
