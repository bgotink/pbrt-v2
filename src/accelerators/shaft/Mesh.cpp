//
//  Mesh.cpp
//  pbrt
//
//  Created by Bram Gotink on 15/11/12.
//
//

#include "mesh.h"
#include "surface.h"
#include "primitive.h"

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
    
    typedef vector<Reference<Primitive> > prim_list;
    typedef prim_list::const_iterator prim_iter;
    
    typedef list<Reference<TriangleMesh> > shape_list;
    typedef shape_list::iterator shape_iter;
    
    shape_list filter(const vector<Reference<Shape> > &list) {
        shape_list res;
        
        for (vector<Reference<Shape> >::const_iterator shape = list.begin(); shape != list.begin(); shape++) {
            Reference<Shape> s = *shape;
            
            if (!s->isTriangleMesh()) {
                Warning("A non-TriangleMesh-shape, ignored!");
            } else {
                res.push_back(static_cast<TriangleMesh *>(&*s));
            }
        }
        
        return res;
    }
    
    shape_list filter(const prim_list &primitives) {
        shape_list meshes;
        
        {
            prim_iter prim_end = primitives.end();
            for (prim_iter prim = primitives.begin(); prim != prim_end; prim++) {
                Reference<Shape> shape = (*prim)->getShape();
                
                if (!shape) {
                    Warning("A non-GeometricPrimitive primitive in the scene -> ignoring!!");
                } else {
                    if (!shape->isTriangleMesh()) {
                        Warning("The shape is not a TriangleMesh");
                    } else {
                        meshes.push_back(static_cast<TriangleMesh *>(&*shape));
                    }
                }
            }
        }

        return meshes;
    }
    
    Mesh::Mesh(const vector<Reference<Shape> > &shapes) {
        shape_list meshes = filter(shapes);
        init(meshes);
    }
    
    Mesh::Mesh(const prim_list &primitives) {
        shape_list meshes = filter(primitives);
        init(meshes);
    }
    
    void Mesh::init(shape_list &meshes) {
        list<Point> new_vertices;
        map< Point, uint32_t> vmap;
        list<Reference<Triangle> > new_triangles;
        
        const shape_iter meshes_end = meshes.end();
        for (shape_iter shape = meshes.begin(); shape != meshes_end; shape++) {
            TriangleMesh &mesh = **shape;
        
            Reference< ::Triangle> cur_triangle = mesh.getTriangle(0);
        
            int vertex_idx = 0;
            const int ntris = mesh.getNbTriangles();
        
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
        }
        
        vertex_pos.reserve(new_vertices.size());
        vertex_pos.insert(vertex_pos.begin(), new_vertices.begin(), new_vertices.end());
        
        triangles.reserve(new_triangles.size());
        triangles.insert(triangles.begin(), new_triangles.begin(), new_triangles.end());
    }
    
    
}
