//
//  Mesh.h
//  pbrt
//
//  Created by Bram Gotink on 15/11/12.
//
//

#ifndef __pbrt__Mesh__
#define __pbrt__Mesh__

#include "memory.h"
#include "../../shapes/trianglemesh.h"

#include "primitive.h"
#include "material.h"

#include "vector.h"

#include <list>
#include <map>

namespace shaft {
    
    class Triangle : public ReferenceCounted {
        Vector3i vertices, neighbours;
        Vector3l edge_labels;
        
        Reference< ::Triangle> original;
        
        friend class Mesh;
        
    public:
        inline uint32_t operator[](unsigned int i) const { return vertices[i]; }
        inline uint32_t getPoint(unsigned int i) const { return vertices[i]; }
        
        inline Reference< ::Triangle> &getOriginal() { return original; }
        inline const Reference< ::Triangle> &getOriginal() const { return original; }
    };
    
    class Mesh {
        friend class ElementTreeNode;
        friend class ElementTree;
        friend class Shaft;
        
        void init(std::list<Reference<TriangleMesh> > &meshes);
        
        typedef std::map<const Shape *, Reference<Primitive> > shapeprimmap;
        
        typedef std::vector<Point> point_v;
        typedef point_v::iterator point_v_iter;
        
        typedef std::vector<Reference<Primitive> > prim_list;
        typedef prim_list::const_iterator prim_iter;
        typedef prim_list::iterator prim_mutiter;
        
        typedef std::list<Reference<TriangleMesh> > shape_list;
        typedef shape_list::iterator shape_iter;
        
        typedef std::vector<Reference<Shape> > shape_vector;
        typedef shape_vector::iterator shape_viter;
        typedef shape_vector::const_iterator shape_vciter;
        
        shape_list filter(const prim_list &);
        shape_list filter(const std::vector<Reference<Shape> > &);
        
    protected:
        std::vector<Point> vertex_pos;
        std::vector<Reference<Triangle> > triangles;
        std::map<uint64_t, bool> is_double_edge;
        unsigned int nbVertices, nbTriangles;
        
        shapeprimmap shape_prim_map;
        
    public:
        Mesh(const vector<Reference<Primitive> > &primitives);
        Mesh(const vector<Reference<Shape> > &primitives);
        
        inline unsigned int getNbVertices() const { return nbVertices; }
        inline unsigned int getNbTriangles() const { return nbTriangles; }

        inline const Reference<Triangle> &getTriangle(unsigned int idx) const {
            Assert(idx >= 0 && idx < triangles.size());
            return triangles[idx];
        }
        
        inline Reference<Triangle> &getTriangle(unsigned int idx) {
            Assert(idx >= 0 && idx < triangles.size());
            return triangles[idx];
        }

        inline const Point &getPoint(unsigned int idx) const {
            Assert(idx >= 0 && idx < nbVertices);
            return vertex_pos[idx];
        }
        
        Reference<Material> getSomeMaterial() const;
        
        inline Reference<Primitive> getPrimitive(const Reference<Shape> &shape) const {
            if (!shape_prim_map.count(&*shape))
                return Reference<Primitive>(NULL);
            
            return shape_prim_map.at(&*shape);
        }
        
        operator char*() const;
    };
    
    bool IntersectsTriangle(const Reference<Triangle> &triangle, const Mesh &mesh, const Ray &ray);

};

#endif /* defined(__pbrt__Mesh__) */
