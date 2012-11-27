//
//  Mesh.cpp
//  pbrt
//
//  Created by Bram Gotink on 15/11/12.
//
//

#include "mesh.h"
#include "surface.h"

#include <list>
#include <map>
#include <vector>
#include <set>

using std::vector;
using std::list;
using std::map;
using std::set;

namespace shaft {
    
    typedef vector<Point> point_v;
    typedef point_v::iterator point_v_iter;
    
    Mesh::Mesh(TriangleMesh& mesh) {
        Reference< ::Triangle> cur_triangle = mesh.getTriangle(0);
        
        int vertex_idx = 0;
        const int ntris = mesh.getNbTriangles();
        list<Point> new_vertices;
        map< Point, uint32_t> vmap;
        list<Reference<Triangle> > new_triangles;
        
        set<RawEdge::idtype> edges_created;
        
        const ::Point *point;
        while (cur_triangle->getIndex() < ntris) {
            Triangle *t = new Triangle();
            Triangle &new_triangle = *t;
            
            for (int i = 0; i < 3; i++) {
                point = &cur_triangle->getPoint(i);
                
                if (vmap.count(*point)) {
                    new_triangle.vertices[i] = vmap[*point];
                } else {
                    new_vertices.push_back(Point(*point));
                    vmap[*point] = vertex_idx;
                    new_triangle.vertices[i] = vertex_idx++;
                }
            }
            
            for (int i = 0; i < 3; i++) {
                uint32_t from = new_triangle.vertices[i];
                uint32_t to = new_triangle.vertices[i == 2 ? 0 : (i+1)];
                
                RawEdge::idtype edge_id = new_triangle.edge_labels[i] = RawEdge::createId(from, to);
                if (edges_created.count(edge_id) == 0) {
                    edges_created.insert(edge_id);
                    is_double_edge[edge_id] = false;
                } else {
                    is_double_edge[edge_id] = true;
                }
            }
            
            new_triangles.push_back(Reference<Triangle> (t));
            
            ++(*cur_triangle);
        }
        
        vertex_pos.reserve(new_vertices.size());
        vertex_pos.insert(vertex_pos.begin(), new_vertices.begin(), new_vertices.end());
        
        triangles.reserve(new_triangles.size());
        triangles.insert(triangles.begin(), new_triangles.begin(), new_triangles.end());
    }
    
    
}
