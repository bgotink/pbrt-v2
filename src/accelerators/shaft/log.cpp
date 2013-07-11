//
//  log.cpp
//  pbrt
//
//  Created by Bram Gotink on 16/05/13.
//
//

#include "log.h"

#ifdef SHAFT_LOG

#include <iostream>
#include <fstream>

#ifdef SHAFT_SHOW_LEAFS
#include "film/image.h"
#endif // defined(SHAFT_SHOW_LEAFS)

using namespace std;

namespace shaft { namespace log {
    
    AtomicUInt64 nb_intersect_operations(0);
    AtomicUInt64 nb_intersect_done(0);
    AtomicUInt64 nb_no_intersected_shaft(0);
    AtomicUInt64 nb_node_intersect_done(0);
    
    AtomicUInt64 nb_shaft_blocked(0);
    AtomicUInt64 nb_shaft_empty(0);
    AtomicUInt64 nb_shaftaccel_intersectp(0);
    
    uint64_t nb_leave_shafts = 0;
    uint64_t nb_total_prims_in_leaves = 0;
    uint64_t nb_total_prims_in_leave_nodes = 0;
    uint64_t nb_total_points_in_leave_nodes = 0;
    uint64_t nb_max_points_in_leave_nodes = 0;
    uint64_t nb_total_depth = 0;
    
    AtomicUInt64 nb_pa(0);
    AtomicUInt64 nb_pb(0);
    AtomicUInt64 nb_pc(0);
    AtomicUInt64 nb_panh(0);
    AtomicUInt64 nb_pbnh(0);
    AtomicUInt64 nb_pcnh(0);
    
#ifdef SHAFT_SHOW_DEPTHS
    FalseColorFilm *falseColorShafts;
#endif
#ifdef SHAFT_SHOW_INTERSECTS
    FalseColorFilm *falseColorIntersects;
#endif
#ifdef SHAFT_SHOW_PRIMS
    FalseColorFilm *falseColorPrims;
#endif
#ifdef SHAFT_SHOW_LEAFS
    Film *falseColorLeafs;
#endif
#ifdef SHAFT_SHOW_SIDES
    FalseColorFilm *falseColorSides;
#endif
#ifdef SHAFT_SHOW_EMPTY_LEAVES
    FalseColorFilm *falseColorEmptyLeaves;
#endif
    
    ParamSet filmParams;
    Filter *filter;
    
#if defined(PBRT_CPP11)
    thread_local CameraSample *cameraSample;
#elif defined(__GCC__)
    __thread CameraSample *cameraSample;
#else
    CameraSample *cameraSample;
#endif
    
    
#if defined(SHAFT_SHOW_LEAFS)
#if defined(PBRT_CPP11)
    thread_local uint32_t hitLeafId;
#elif defined(__GCC__)
    __thread uint32_t hitLeafId;
#else
    uint32_t hitLeafId;
#endif
#endif // defined(SHAFT_SHOW_LEAFS)
    
    static double buildTime;

static void PrintStats(ostream &str) {
    str << "# ray intersectp tests done: " << nb_intersect_done << endl;
    str << "# real intersect operations @ shaft: " << nb_intersect_operations << endl;
    str << "# intersects not done by shaft: " << nb_no_intersected_shaft << endl;
    str << "# real intersect operations @ nodes: " << nb_node_intersect_done << endl;
    str << endl;
    
    str << "# shafts blocked: " << nb_shaft_blocked << endl;
    str << "# shafts empty: " << nb_shaft_empty << endl;
    str << "# ShaftAccel::IntersectP: " << nb_shaftaccel_intersectp << endl;
    str << endl;
    
    str << "# leave shafts: " << nb_leave_shafts << endl;
    str << "# nb prims in leave shafts: " << nb_total_prims_in_leaves
        << " (avg. " << static_cast<double>(nb_total_prims_in_leaves) / static_cast<double>(nb_leave_shafts) << ")" << endl;
    str << "# nb prims in leave nodes: " << nb_total_prims_in_leave_nodes
        << " (avg. " << static_cast<double>(nb_total_prims_in_leave_nodes) / static_cast<double>(nb_leave_shafts) << ")" << endl;
    str << "# nb points in leave nodes: " << nb_total_points_in_leave_nodes
        << " (avg. " << static_cast<double>(nb_total_points_in_leave_nodes) / static_cast<double>(nb_leave_shafts) << ")" << endl;
    str << "\tmax: " << nb_max_points_in_leave_nodes << endl;
    str << "avg. depth: " << static_cast<float>(nb_total_depth) / static_cast<double>(nb_leave_shafts) << endl;
    str << endl;
    
    if (nb_pc > 0 || nb_pb > 0 || nb_pa > 0) {
        float tot = nb_pa + nb_pb + nb_pc;
        str << "# times A: " << nb_pa
            << " (" << 100 * (static_cast<float>(nb_pa) / tot) << " %) \t-\t " << nb_panh
            << " (" << 100. * static_cast<double>(nb_panh) / static_cast<double>(nb_pa) << " %) missed" << endl;
        str << "# times B: " << nb_pb
            << " (" << 100 * (static_cast<float>(nb_pb) / tot) << " %) \t-\t " << nb_pbnh
            << " (" << 100. * static_cast<double>(nb_pbnh) / static_cast<double>(nb_pb) << " %) missed" << endl;
        str << "# times C: " << nb_pc
            << " (" << 100 * (static_cast<float>(nb_pc) / tot) << " %) \t-\t " << nb_pcnh
            << " (" << 100. * static_cast<double>(nb_pcnh) / static_cast<double>(nb_pc) << " %) missed" << endl;
        
        str << endl;
    }
}

void ShaftLogResult() {
    PrintStats(cerr);
}
    
void ShaftSaveBuildTime(double bT) {
    buildTime = bT;
}

#define SAVE_FALSE_COLOR(image) \
    if (image != NULL) \
        image->WriteImage(); \
    delete image; \
    image = NULL;

void ShaftSaveFalseColor() {
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
#ifdef SHAFT_SHOW_SIDES
	SAVE_FALSE_COLOR(falseColorSides);
#endif
#ifdef SHAFT_SHOW_EMPTY_LEAVES
	SAVE_FALSE_COLOR(falseColorEmptyLeaves);
#endif
}

void ShaftSaveMetaData(double timeSpent) {
    ofstream metadata;
    
    string img_filename = filmParams.FindOneString("filename", PbrtOptions.imageFile);
    if (img_filename == "")
#ifdef PBRT_HAS_OPENEXR
        img_filename = "pbrt.exr";
#else
        img_filename = "pbrt.tga";
#endif
    
    string filename;
    {
        char newfilename[filename.length() + 1];
        memcpy(newfilename, img_filename.c_str(), img_filename.length() - 4);
        memcpy(newfilename + img_filename.length() - 4, ".txt", 5);
        
        filename = newfilename;
    }
    
    metadata.open(filename.c_str(), ios::out | ios::trunc);
    
    metadata << img_filename << endl << endl;
    PrintStats(metadata);
    
#ifdef SHAFT_SHOW_DEPTHS
    if (falseColorShafts != NULL) {
        metadata << "Shaft Depths:" << endl;
        metadata << "Max depth: " << falseColorShafts->GetMax() << endl;
        metadata << endl;
    }
#endif
    
#ifdef SHAFT_SHOW_INTERSECTS
    if (falseColorIntersects != NULL) {
        metadata << "Intersects:" << endl;
        metadata << "Max Intersects: " << falseColorIntersects->GetMax() << endl;
        metadata << endl;
    }
#endif
    
#ifdef SHAFT_SHOW_PRIMS
    if (falseColorPrims != NULL) {
        metadata << "Prims:" << endl;
        metadata << "Max primitives: " << falseColorPrims->GetMax() << endl;
        metadata << endl;
    }
#endif
    
    metadata << "Building took " << buildTime << " seconds." << endl;
    metadata << "Rendering took " << timeSpent << " seconds." << endl;
    
    metadata.flush();
    metadata.close();

	// reset all counters

	nb_intersect_operations =
	nb_intersect_done =
	nb_no_intersected_shaft =
	nb_node_intersect_done = 0;

	nb_shaft_blocked =
	nb_shaft_empty =
	nb_shaftaccel_intersectp = 0;

	nb_leave_shafts =
	nb_total_prims_in_leaves =
	nb_total_prims_in_leave_nodes =
	nb_total_points_in_leave_nodes =
	nb_total_depth =
	nb_max_points_in_leave_nodes = 0;

	nb_pa = nb_pb = nb_pc =
	nb_panh = nb_pbnh = nb_pcnh = 0;
}
    
#define CREATE_FALSE_COLOR_ON_IMAGEFILM(ptr, name) \
	if (ptr != NULL) \
		delete ptr; \
	ptr = & ::CreateImageFilm(FalseColorFilm::GetFilename(name, filmParams), filmParams, filter)->SetNoDeleteFilter();
    
#define CREATE_FALSE_COLOR(ptr, name) \
	if (ptr != NULL) \
		delete ptr; \
	ptr = ::CreateFalseColorFilm(name, filmParams, filter);
    
void ShaftNewImage() {
#ifdef SHAFT_SHOW_DEPTHS
    CREATE_FALSE_COLOR(falseColorShafts, "depth");
#endif
#ifdef SHAFT_SHOW_INTERSECTS
    CREATE_FALSE_COLOR(falseColorIntersects, "intersects");
#endif
#ifdef SHAFT_SHOW_PRIMS
    CREATE_FALSE_COLOR(falseColorPrims, "primcounts");
#endif
#ifdef SHAFT_SHOW_LEAFS
    CREATE_FALSE_COLOR_ON_IMAGEFILM(falseColorLeafs, "leafs");
#endif
#ifdef SHAFT_SHOW_SIDES
    CREATE_FALSE_COLOR(falseColorSides, "sides");
#endif
#ifdef SHAFT_SHOW_EMPTY_LEAVES
    CREATE_FALSE_COLOR(falseColorEmptyLeaves, "empty.leaves");
#endif
}


}}

#endif
