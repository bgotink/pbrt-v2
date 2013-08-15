//
//  mesh.h
//  pbrt
//
//  Created by Bram Gotink on 15/11/12.
//
//

#ifndef __pbrt__mesh__
#define __pbrt__mesh__

#include "memory.h"
#include "../../shapes/trianglemesh.h"

#include "primitive.h"
#include "material.h"

#include "vector.h"

#include <list>
#include <map>

namespace shaft {
    
    class Mesh {
        friend struct ElementTreeNode;
        friend struct ElementTree;
        friend class Shaft;
        
        typedef Reference<Primitive> prim_ref;
        typedef Reference<Shape> shape_ref;
        typedef Reference<TriangleMesh> trimesh_ref;
        typedef Reference<Triangle> tris_ref;

        typedef std::map<const Shape *, Reference<Primitive> > shapeprimmap;
        
        typedef std::vector<Point> point_v;
        typedef point_v::iterator point_viter;
        
        typedef std::vector<prim_ref> prim_v;
        typedef prim_v::const_iterator prim_viter;
        typedef prim_v::iterator prim_vmutiter;
        
        typedef std::list<trimesh_ref> trimesh_l;
        typedef trimesh_l::iterator trimesh_liter;
        
        typedef std::vector<shape_ref> shape_v;
        typedef shape_v::iterator shape_viter;
        typedef shape_v::const_iterator shape_vciter;
        
        typedef std::vector<tris_ref> tris_v;

        void init(trimesh_l &meshes);

        trimesh_l filter(const prim_v &);
        trimesh_l filter(const shape_v &);
        
    protected:
        point_v vertex_pos;
        tris_v triangles;
        unsigned int nbVertices, nbTriangles;
        
        shapeprimmap shape_prim_map;
        
    public:
        Mesh(const prim_v &primitives);
        Mesh(const shape_v &primitives);
        
        inline unsigned int getNbVertices() const { return nbVertices; }
        inline unsigned int getNbTriangles() const { return nbTriangles; }

        inline const tris_ref &getTriangle(unsigned int idx) const {
            Assert(idx >= 0 && idx < triangles.size());
            return triangles[idx];
        }
        
        inline tris_ref &getTriangle(unsigned int idx) {
            Assert(idx >= 0 && idx < triangles.size());
            return triangles[idx];
        }

        inline const Point &getPoint(unsigned int idx) const {
            Assert(idx >= 0 && idx < nbVertices);
            return vertex_pos[idx];
        }
        
        Reference<Material> getSomeMaterial() const;
        
        inline prim_ref getPrimitive(const shape_ref &shape) const {
            if (!shape_prim_map.count(&*shape))
                return Reference<Primitive>(NULL);
            
            return shape_prim_map.at(&*shape);
        }

        uint64_t memsize() const;
        
        operator char*() const;
    };
    
};

#endif /* defined(__pbrt__mesh__) */
