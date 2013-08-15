//
//  Mesh.cpp
//  pbrt
//
//  Created by Bram Gotink on 15/11/12.
//
//

#include "mesh.h"
#include "primitive.h"

#include <list>
#include <map>
#include <vector>
#include <set>
#include <algorithm>

using std::vector;
using std::list;
using std::map;
using std::set;

namespace shaft {
    
    Mesh::trimesh_l Mesh::filter(const shape_v &list) {
    	trimesh_l res;
        
        Info("Filtering shape_vector");
        
        for (shape_vciter shape = list.begin(); shape != list.end(); shape++) {
            Reference<Shape> s = *shape;
            
            shape_v todo;
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
    
    Mesh::trimesh_l Mesh::filter(const prim_v &primitives) {
        Info("Filtering prim_list");
        
        trimesh_l meshes;
        {
            prim_viter prim_end = primitives.end();
            for (prim_viter prim = primitives.begin(); prim != prim_end; prim++) {
                shape_ref shape = (*prim)->getShape();
                
                if (!shape) {
                    Warning("A non-GeometricPrimitive primitive in the scene -> ignoring!!");
                } else {
                	shape_v todo;
                    todo.push_back(shape);
                    
                    while (!todo.empty()) {
                        shape = todo.back(); todo.pop_back();
                        
                        if (shape->CanIntersect()) {
                            Warning("Invalid primitive shape encountered, CanIntersect() returned true");
                        } else if (shape->isTriangleMesh()) {
                            // fugly, but hey, it works
                            shape_prim_map[&* shape] = *prim;
                            
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
    
    Mesh::Mesh(const shape_v &shapes) {
    	trimesh_l meshes = filter(shapes);
        Assert(!meshes.empty());
        init(meshes);
    }
    
    Mesh::Mesh(const prim_v &primitives) {
    	trimesh_l meshes = filter(primitives);
        Assert(!meshes.empty());
        init(meshes);
    }
    
    void Mesh::init(trimesh_l &meshes) {
        set<Point> new_vertices;
        map< Point, uint32_t> vmap;
        list<tris_ref> new_triangles;
        
        vector<prim_ref> prims;
        
        const trimesh_liter meshes_end = meshes.end();
        for (trimesh_liter shape = meshes.begin(); shape != meshes_end; shape++) {
            TriangleMesh &mesh = **shape;

            if (mesh.getNbTriangles() == 0) {
            	Warning("mesh has no triangles");
            }
        
            for (int i = 0, end = mesh.getNbTriangles(); i < end; i++)
                new_triangles.push_back(mesh.getTriangle(i));
            
            for (int i = 0, end = mesh.getNbPoints(); i < end; i++) {
                new_vertices.insert(mesh.getPoint(i));
            }

            prim_ref prim = getPrimitive(shape_ref(&mesh));
            if (prim) {
                prim->FullyRefine(prims);
            }
        }

        nbVertices = new_vertices.size();
        vertex_pos.reserve(nbVertices);
        vertex_pos.assign(new_vertices.begin(), new_vertices.end());

//        std::sort(vertex_pos.begin(), vertex_pos.end());
//        vertex_pos.erase(std::unique(vertex_pos.begin(), vertex_pos.end()), vertex_pos.end());
//
//        Severe("Removed %ld elements", nbVertices- vertex_pos.size());
//        nbVertices = vertex_pos.size();

        shape_prim_map.clear();
        for (prim_viter prim = prims.begin(), end = prims.end(); prim != end; prim++) {
            shape_prim_map[&* (*prim)->getShape()] = prim_ref(*prim);
        }
        
        Info("Nb of primitives in prims: %lu, in the shape_prim_map: %lu",
        		prims.size(), shape_prim_map.size());
        Info("Nb of shapes: %lu, nb of triangles: %lu",
        		meshes.size(), new_triangles.size());
        
        nbVertices = new_vertices.size();
        vertex_pos.reserve(nbVertices);
        vertex_pos.insert(vertex_pos.begin(), new_vertices.begin(), new_vertices.end());
        
        nbTriangles = new_triangles.size();
        triangles.reserve(nbTriangles);
        triangles.insert(triangles.begin(), new_triangles.begin(), new_triangles.end());
    }
    
    Reference<Material> Mesh::getSomeMaterial() const {
        if (shape_prim_map.size() == 0) {
            Error("Asking for some material, but I ain't got none");
            return Reference<Material>(NULL);
        }
        
        return shape_prim_map.begin()->second->getMaterial();
    }

    uint64_t Mesh::memsize() const {
        return static_cast<uint64_t>(sizeof(Mesh))
                + vertex_pos.size() * sizeof(Point)
                + triangles.size() * (sizeof(tris_ref) + sizeof(tris_ref::value_type));
        // ignore shapeprimmap as we don't need this in normal circumstances
    }
}
