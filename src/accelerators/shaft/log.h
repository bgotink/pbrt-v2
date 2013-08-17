//
//  log.h
//  pbrt
//
//  Created by Bram Gotink on 05/12/12.
//
//

// freely configurable, SHAFT_LOG flag disables all other if disabled

// enable logging
#define SHAFT_LOG

// false color images
#define SHAFT_SHOW_INTERSECTS
#define SHAFT_SHOW_DEPTHS
#define SHAFT_SHOW_PRIMS
#define SHAFT_SHOW_LEAFS
#define SHAFT_SHOW_SIDES
#define SHAFT_SHOW_EMPTY_LEAVES

// log extra (SLOWER for all formulae!)
#define SHAFT_LOG_VISIBILITY_ALL

#define SHAFT_LOG_TESTRAYS

// end of configuration

#ifndef pbrt_log_h
#define pbrt_log_h

#ifdef SHAFT_LOG
#include "film/falsecolor.h"
#include "accelerators/shaft/shaftaccel.h"
#include "core/geometry.h"
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
#ifdef SHAFT_SHOW_SIDES
#undef SHAFT_SHOW_SIDES
#endif // defined(SHAFT_SHOW_SIDES)
#ifdef SHAFT_SHOW_EMPTY_LEAVES
#undef SHAFT_SHOW_EMPTY_LEAVES
#endif // defined(SHAFT_SHOW_EMPTY_LEAVES)

#ifdef SHAFT_LOG_VISIBILITY_ALL
#undef SHAFT_LOG_VISIBILITY_ALL
#endif // defined(SHAFT_LOG_VISIBILITY_ALL)

#ifdef SHAFT_LOG_TESTRAYS
#undef SHAFT_LOG_TESTRAYS
#endif // defined(SHAFT_LOG_TESTRAYS)

#endif // !defined(SHAFT_LOG)

#include "memory.h"

namespace shaft { namespace log {

typedef volatile double AtomicDouble;

#ifdef SHAFT_LOG

extern AtomicUInt64 nb_intersect_operations;
extern AtomicUInt64 nb_intersect_done;
extern AtomicUInt64 nb_no_intersected_shaft;
extern AtomicUInt64 nb_node_intersect_done;

extern AtomicUInt64 nb_shaft_blocked;
extern AtomicUInt64 nb_shaft_empty;
extern AtomicUInt64 nb_shaftaccel_intersectp;
    
extern AtomicUInt64 nb_pa, nb_pb, nb_pc;

#ifdef SHAFT_LOG_VISIBILITY_ALL
extern AtomicUInt64 nb_pa_na, nb_pa_ab, nb_pa_anb;
extern AtomicUInt64 nb_pb_nb, nb_pb_ab, nb_pb_nab;
extern AtomicUInt64 nb_pc_anb, nb_pc_ab, nb_pc_nab, nb_pc_nanb;

extern AtomicDouble mse;
extern AtomicUInt64 mse_count;
#else
extern AtomicUInt64 nb_panh, nb_pbnh, nb_pcnh;
#endif

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

#ifdef SHAFT_LOG_VISIBILITY_ALL
#define ADD_MSE(e) \
    float _tmpfl = e; \
    AtomicAdd(&mse, _tmpfl * _tmpfl); \
    AtomicAdd(&mse_count, 1);
inline void ProbVis_pa_result(float vis, bool visA, bool visB) {
    if (!visA) {
        AtomicAdd(&nb_pa_na, 1);
        ADD_MSE(vis);
    } else {
        if (visB) {
            AtomicAdd(&nb_pa_ab, 1);
            ADD_MSE(1-vis);
        } else {
            AtomicAdd(&nb_pa_anb, 1);
            ADD_MSE(vis);
        }
    }
}

inline void ProbVis_pb_result(float vis, bool visA, bool visB) {
    if (!visB) {
        AtomicAdd(&nb_pb_nb, 1);
        ADD_MSE(vis);
    } else {
        if (visA) {
            AtomicAdd(&nb_pb_ab, 1);
            ADD_MSE(1-vis);
        } else {
            AtomicAdd(&nb_pb_nab, 1);
            ADD_MSE(vis);
        }
    }
}

inline void ProbVis_pc_result(float vis, bool visA, bool visB) {
    if (visA) {
        if (visB) {
            AtomicAdd(&nb_pc_ab, 1);
            ADD_MSE(1 - vis);
        } else {
            AtomicAdd(&nb_pc_anb, 1);
            ADD_MSE(vis);
        }
    } else {
        if (visB) {
            AtomicAdd(&nb_pc_nab, 1);
            ADD_MSE(vis);
        } else {
            AtomicAdd(&nb_pc_nanb, 1);
            ADD_MSE(vis);
        }
    }
}
#else
inline void ProbVis_pa_noHit() {
    AtomicAdd(&nb_panh, 1);
}
    
inline void ProbVis_pb_noHit() {
    AtomicAdd(&nb_pbnh, 1);
}

inline void ProbVis_pc_noHit() {
    AtomicAdd(&nb_pcnh, 1);
}
#endif

void ShaftsInitStarted();
void ShaftsInitEnded();
    
void ShaftLeafCreated(uint64_t nbPrims, uint64_t nbPoints, uint64_t nbPrimsInShaft, uint64_t depth);
    
void ShaftSaveBuildTime(double buildTime);
void ShaftSaveInitTime(double initTime);
    
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
#ifdef SHAFT_SHOW_SIDES
    extern FalseColorFilm *falseColorSides;
#endif // defined(SHAFT_SHOW_SIDES)
#ifdef SHAFT_SHOW_EMPTY_LEAVES
    extern FalseColorFilm *falseColorEmptyLeaves;
#endif // defined(SHAFT_SHOW_EMPTY_LEAVES)
    
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
    
#ifdef SHAFT_SHOW_EMPTY_LEAVES
    inline void ShaftSetLeafEmpty(bool empty) {
    	if (falseColorEmptyLeaves != NULL && cameraSample != NULL) falseColorEmptyLeaves->Set(*cameraSample, empty ? 2 : 1);
    }
#else
#define ShaftSetLeafEmpty()
#endif

    void ShaftSaveFalseColor();

    void ShaftSaveMetaData(double timeSpent);

#ifdef SHAFT_LOG_TESTRAYS
    void addTestRay();
    void addUsefulTestRay();
#else
#   define addTestRay()
#   define addUsefulTestRay()
#endif

    void setAccelMemsize(uint64_t memsize);

    void storeBBoxSize(const BBox &box);
    
#else
    
#define ShaftStartIntersectP()
#define ShaftIntersectTest()
#define ShaftNotIntersected()
#define ShaftLogResult()
#define ShaftNodeIntersectionTest()
#define ShaftBlocked()
#define ShaftEmpty()
#define ShaftAccelIntersectP()
#define ShaftLeafCreated()
    
#define ProbVis_pa()
#define ProbVis_pb()
#define ProbVis_pc()
#define ProbVis_pa_noHit()
#define ProbVis_pb_noHit()
#define ProbVis_pc_noHit()

#define ShaftsInitStarted()
#define ShaftsInitEnded()
#define ShaftNewImage()
#define ShaftDepth()
#define ShaftAddIntersect()
#define ShaftSaveFalseColor()
#define ShaftSaveBuildTime()
#define ShaftSaveInitTime()
#define ShaftSaveMetaData()

#define addTestRay()
#define addUsefulTestRay()
#define setAccelMemsize()
#define storeBBoxSize()
    
#endif

}}

#endif
