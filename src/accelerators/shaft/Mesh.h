//
//  Mesh.h
//  pbrt
//
//  Created by Bram Gotink on 15/11/12.
//
//

#ifndef __pbrt__Mesh__
#define __pbrt__Mesh__

#include "api.h"
#include "memory.h"
#include "../../shapes/trianglemesh.h"

#include "vector.h"

namespace shaft {
    
    class Triangle : public ReferenceCounted {
        Vector3i vertices, neighbours, edge_labels;
        friend class Mesh;
        
    public:
        int operator[](int i) const { return vertices[i]; }
    };
    
    class Mesh {
        friend class ElementTreeNode;
        friend class ElementTree;
        
    protected:
        std::vector<Point> vertex_pos;
        std::vector<Reference<Triangle> > triangles;
        std::vector<bool> is_double_edge;
        
    public:
        Mesh(TriangleMesh &mesh);
    };
    
};

#endif /* defined(__pbrt__Mesh__) */
