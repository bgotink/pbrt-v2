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

#include "vector.h"

#include <list>
#include <map>

namespace shaft {
    
    class Triangle : public ReferenceCounted {
        Vector3i vertices, neighbours;
        Vector3l edge_labels;
        friend class Mesh;
        
    public:
        inline uint32_t operator[](unsigned int i) const { return vertices[i]; }
        inline uint32_t getPoint(unsigned int i) const { return vertices[i]; }
    };
    
    class Mesh {
        friend class ElementTreeNode;
        friend class ElementTree;
        friend class Shaft;
        
        void init(std::list<Reference<TriangleMesh> > &meshes);
        
    protected:
        std::vector<Point> vertex_pos;
        std::vector<Reference<Triangle> > triangles;
        std::map<uint64_t, bool> is_double_edge;
        unsigned int nbVertices, nbTriangles;
        
    public:
        Mesh(const vector<Reference<Primitive> > &primitives);
        Mesh(const vector<Reference<Shape> > &primitives);
        
        inline unsigned int getNbVertices() const { return nbVertices; }
        inline unsigned int getNbTriangles() const { return nbTriangles; }

        inline const Reference<Triangle> &getTriangle(unsigned int idx) const {
            Assert(idx >= 0 && idx < triangles.size());
            return triangles[idx];
        }

        inline const Point &getPoint(unsigned int idx) const {
            Assert(idx >= 0 && idx < nbVertices);
            return vertex_pos[idx];
        }
    };
    
    bool IntersectsTriangle(const Reference<Triangle> &triangle, const Mesh &mesh, const Ray &ray);

};

#endif /* defined(__pbrt__Mesh__) */
