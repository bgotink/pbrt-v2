//
//  Mesh.cpp
//  pbrt
//
//  Created by Bram Gotink on 15/11/12.
//
//

#include "mesh.h"

#include <list>
#include <map>

using std::vector;
using std::list;
using std::map;

namespace shaft {
    
    typedef vector<Point> point_v;
    typedef point_v::iterator point_v_iter;
    
    Mesh::Mesh(TriangleMesh& mesh) {
        Reference< ::Triangle> cur_triangle = mesh.getTriangle(0);
        
        int vertex_idx = 0;
        const int ntris = mesh.getNbTriangles();
        list<Point> new_vertices;
        map< Point, int> vmap;
        list<Reference<Triangle> > new_triangles;
        
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
            new_triangles.push_back(Reference<Triangle> (t));
            
            ++(*cur_triangle);
        }
        
        vertex_pos.reserve(new_vertices.size());
        vertex_pos.insert(vertex_pos.begin(), new_vertices.begin(), new_vertices.end());
        
        triangles.reserve(new_triangles.size());
        triangles.insert(triangles.begin(), new_triangles.begin(), new_triangles.end());
    }
    
    
}
