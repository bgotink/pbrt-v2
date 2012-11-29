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
        inline uint32_t operator[](int i) const { return vertices[i]; }
        inline uint32_t getPoint(int i) const { return vertices[i]; }
    };
    
    class Mesh {
        friend class ElementTreeNode;
        friend class ElementTree;
        friend class Shaft;
        
        void init(std::list<Reference<TriangleMesh> > &meshes);
        
    protected:
        std::vector<Point> vertex_pos;
        std::vector<Reference<Triangle> > triangles;
        std::map<uint32_t, bool> is_double_edge;
        
    public:
        Mesh(const vector<Reference<Primitive> > &primitives);
        Mesh(const vector<Reference<Shape> > &primitives);

        inline const Reference<Triangle> &getTriangle(int idx) const {
            Assert(idx >= 0 && idx < triangles.size());
            return triangles[idx];
        }

        inline const Point &getPoint(int idx) const {
            Assert(idx >= 0 && idx < vertex_pos.size());
            return vertex_pos[idx];
        }
    };

};

#endif /* defined(__pbrt__Mesh__) */
