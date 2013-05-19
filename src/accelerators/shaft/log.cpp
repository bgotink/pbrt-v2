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

using namespace std;

namespace shaft { namespace log {
    
    AtomicInt64 nb_intersect_operations = 0;
    AtomicInt64 nb_intersect_done = 0;
    AtomicInt64 nb_no_intersected_shaft = 0;
    AtomicInt64 nb_node_intersect_done = 0;
    
    AtomicInt64 nb_shaft_blocked = 0;
    AtomicInt64 nb_shaft_empty = 0;
    AtomicInt64 nb_shaftaccel_intersectp = 0;
    
    AtomicInt64 nb_leave_shafts = 0;
    AtomicInt64 nb_total_prims_in_leaves = 0;
    AtomicInt64 nb_total_prims_in_leave_nodes = 0;
    AtomicInt64 nb_total_points_in_leave_nodes = 0;
    AtomicInt64 nb_total_depth = 0;
    
    AtomicInt64 nb_pa = 0;
    AtomicInt64 nb_pb = 0;
    AtomicInt64 nb_pc = 0;
    AtomicInt64 nb_panh = 0;
    AtomicInt64 nb_pbnh = 0;
    AtomicInt64 nb_pcnh = 0;
    
#ifdef SHAFT_SHOW_DEPTHS
    FalseColorFilm *falseColorShafts;
#endif
#ifdef SHAFT_SHOW_INTERSECTS
    FalseColorFilm *falseColorIntersects;
#endif
    
    ParamSet filmParams;
    Filter *filter;
    
#ifdef __GCC__
    __thread CameraSample *cameraSample;
#else
    CameraSample *cameraSample;
#endif

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
    }
}

void ShaftLogResult() {
    PrintStats(cerr);
}
    
void ShaftSaveMetaData() {
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
    }
#endif
    
    metadata.flush();
    metadata.close();
}


}}

#endif
