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
    
    Mesh::shape_list Mesh::filter(const shape_vector &list) {
        shape_list res;
        
        Info("Filtering shape_vector");
        
        for (shape_vciter shape = list.begin(); shape != list.end(); shape++) {
            Reference<Shape> s = *shape;
            
            shape_vector todo;
            todo.push_back(s);
            
            while (!todo.empty()) {
                s = todo.back(); todo.pop_back();
                
                if (s->CanIntersect()) {
                    Warning("Invalid primitive shape encountered, CanIntersect() returned true");
                } else if (s->isTriangleMesh()) {
                    res.push_back(Reference<TriangleMesh>(static_cast<TriangleMesh *>(&*s)));
                } else {
                    Info("Refining...");
                    s->Refine(todo);
                }
            }
        }
        
        return res;
    }
    
    Mesh::shape_list Mesh::filter(const prim_list &primitives) {
        Info("Filtering prim_list");
        
        shape_list meshes;
        {
            prim_iter prim_end = primitives.end();
            for (prim_iter prim = primitives.begin(); prim != prim_end; prim++) {
                Reference<Shape> shape = (*prim)->getShape();
                
                if (!shape) {
                    Warning("A non-GeometricPrimitive primitive in the scene -> ignoring!!");
                } else {
                    shape_vector todo;
                    todo.push_back(shape);
                    
                    while (!todo.empty()) {
                        shape = todo.back(); todo.pop_back();
                        
                        if (shape->CanIntersect()) {
                            Warning("Invalid primitive shape encountered, CanIntersect() returned true");
                        } else if (shape->isTriangleMesh()) {
                            meshes.push_back(Reference<TriangleMesh>(static_cast<TriangleMesh *>(&*shape)));
                        } else {
                            Info("Refining...");
                            shape->Refine(todo);
                        }
                    }

                }
            }
        }

        return meshes;
    }
    
    Mesh::Mesh(const vector<Reference<Shape> > &shapes) {
        shape_list meshes = filter(shapes);
        Assert(!meshes.empty());
        init(meshes);
    }
    
    Mesh::Mesh(const prim_list &primitives) {
        shape_list meshes = filter(primitives);
        Assert(!meshes.empty());
        init(meshes);
    }
    
    void Mesh::init(shape_list &meshes) {
        list<Point> new_vertices;
        map< Point, uint32_t> vmap;
        list<Reference<Triangle> > new_triangles;
        
        vector<Reference<Primitive> > prims;
        
        int vertex_idx = 0;
        const shape_iter meshes_end = meshes.end();
        for (shape_iter shape = meshes.begin(); shape != meshes_end; shape++) {
            TriangleMesh &mesh = **shape;
        
            Reference< ::Triangle> cur_triangle = mesh.getTriangle(0);
        
            set<RawEdge::idtype> edges_created;
        
            const ::Point *point;
            while (cur_triangle->isValid()) {
                Triangle *t = new Triangle;
                Triangle &new_triangle = *t;
                new_triangle.original = mesh.getTriangle(cur_triangle->getIndex());
            
                for (int i = 0; i < 3; i++) {
                    point = &cur_triangle->getPoint(i);
                
                    if (vmap.count(*point)) {
                        new_triangle.vertices[i] = vmap[*point];
                    } else {
                        new_vertices.push_back(Point(*point));
                        
                        vmap[*point] = vertex_idx;
                        new_triangle.vertices[i] = vertex_idx;
                        
                        vertex_idx++;
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
            
            Reference<Primitive> prim = getPrimitive(Reference<Shape>(&mesh));
            if (prim) {
                prim->FullyRefine(prims);
            }
        }
        
        shape_prim_map.clear();
        for (vector<Reference<Primitive> >::iterator prim = prims.begin(); prim != prims.end(); prim++) {
            shape_prim_map[&* (*prim)->getShape()] = Reference<Primitive>(*prim);
        }
        
        Info("Nb of primitives in prims: %lu, in the shape_prim_map: %lu", prims.size(), shape_prim_map.size());
        
        nbVertices = new_vertices.size();
        vertex_pos.reserve(nbVertices);
        vertex_pos.insert(vertex_pos.begin(), new_vertices.begin(), new_vertices.end());
        Assert(new_vertices.size() == vertex_pos.size());
        
        nbTriangles = new_triangles.size();
        triangles.reserve(nbTriangles);
        triangles.insert(triangles.begin(), new_triangles.begin(), new_triangles.end());
        Assert(new_triangles.size() == triangles.size());
    }
    
    bool IntersectsTriangle(const Reference<Triangle> &triangle, const Mesh &mesh, const Ray &ray) {
#if     true
        return triangle->getOriginal()->IntersectP(ray);
#else
        PBRT_RAY_TRIANGLE_INTERSECTIONP_TEST(const_cast<Ray *>(&ray), const_cast<Triangle *>(&*triangle));
        // Compute $\VEC{s}_1$
        
        // Get triangle vertices in _p1_, _p2_, and _p3_
        const Point &p1 = mesh.getPoint(triangle->getPoint(0));
        const Point &p2 = mesh.getPoint(triangle->getPoint(1));
        const Point &p3 = mesh.getPoint(triangle->getPoint(2));
        Vector e1 = p2 - p1;
        Vector e2 = p3 - p1;
        Vector s1 = Cross(ray.d, e2);
        float divisor = Dot(s1, e1);
        
        if (divisor == 0.)
            return false;
        float invDivisor = 1.f / divisor;
        
        // Compute first barycentric coordinate
        Vector d = ray.o - p1;
        float b1 = Dot(d, s1) * invDivisor;
        if (b1 < 0. || b1 > 1.)
            return false;
        
        // Compute second barycentric coordinate
        Vector s2 = Cross(d, e1);
        float b2 = Dot(ray.d, s2) * invDivisor;
        if (b2 < 0. || b1 + b2 > 1.)
            return false;
        
        // Compute _t_ to intersection point
        float t = Dot(e2, s2) * invDivisor;
        if (t < ray.mint || t > ray.maxt)
            return false;
        
        PBRT_RAY_TRIANGLE_INTERSECTIONP_HIT(const_cast<Ray *>(&ray), t);
        return true;
#endif
    }
    
    Reference<Material> Mesh::getSomeMaterial() const {
        if (shape_prim_map.size() == 0)
            return Reference<Material>(NULL);
        
        return shape_prim_map.begin()->second->getMaterial();
    }
}
