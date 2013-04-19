//
//  vector.h
//  pbrt
//
//  Created by Bram Gotink on 15/11/12.
//
//

#ifndef pbrt_vector_h
#define pbrt_vector_h

#include "pbrt.h"
#include "geometry.h"

inline int sign(float f) {
    Assert(!isnan(f));
    if (f == 0.f) return 0;
    return (f > 0.f) ? 1 : -1;
}

namespace shaft{
    
    template <class T>
    class Vector3 {
    public:
        T x, y, z;
        
        Vector3() : x(0), y(0), z(0) {};
        Vector3(T x, T y, T z) : x(x),y(y),z(z) {}
        ~Vector3() {}
        
        inline Vector3 &operator=(Vector3 v) {
            x = v.x; y = v.y; z = v.z;
            return *this;
        }
        
        inline T &operator[](int i) {
            Assert(i >= 0 && i < 3);
            return (&x)[i];
        }
        
        inline T operator[](int i) const {
            Assert(i >= 0 && i < 3);
            return (&x)[i];
        }
    };
    
    typedef Vector3<uint32_t> Vector3i;
    typedef Vector3<uint64_t> Vector3l;
    
    template <class T>
    class Vector4 {
    public:
        T x, y, z, w;
        
        inline Vector4() : x(0), y(0), z(0), w(0) {}
        inline Vector4(T x, T y, T z, T w) : x(x), y(y), z(z), w(w) {}
        inline ~Vector4() {};
        
        inline Vector4<T> &operator=(Vector4 v) {
            x = v.x; y = v.y; z = v.z; w = v.w;
            return *this;
        }
        
        inline T &operator[](int i) {
            Assert(i >= 0 && i < 4);
            return (&x)[i];
        }
        
        inline T operator[](int i) const {
            Assert(i >= 0 && i < 4);
            return (&x)[i];
        }
    };
    
    typedef Vector4<float> Vector4f;
    
    inline float operator*(const Point &o, const Vector4f &p) {
        return o.x * p.x + o.y * p.y + o.z * p.z + p.w;
    }
    
    inline float operator*(const Vector4f &p, const Point &o) {
        return o * p;
    }
    
    inline float operator*(const ::Vector &v, const Vector4f &p) {
        return v.x * p.x + v.y * p.y + v.z * p.z;
    }
    
    inline float operator*(const Vector4f &p, const ::Vector &v) {
        return v * p;
    }
    
    inline Vector4f CreatePlane(const Point &a, const Point &b, const Point&c) {
        ::Vector n = (b - a) ^ (c - a);
        float d = -n.x * a.x - n.y * a.y - n.z * a.z;
        
        if (n.x + n.y + n.z == 0) {
            Severe("Creating plane between (%f,%f,%f), (%f,%f,%f), (%f,%f,%f) yields (0, 0, 0, %f)",
                  a.x, a.y, a.z,
                  b.x, b.y, b.z,
                  c.x, c.y, c.z,
                  d);
        }
        
        return Vector4f(n.x, n.y, n.z, d);
    }
    
    inline Point getPointOnPlane(const Vector4f &p) {
        float t = p.x + p.y + p.z;
        Assert(t != 0.f);
        Assert(!isnan(t));
        
        if (t == 0) {
            Error("This function doesn't work for a plane (a*x + b*y + c*z + d == 0) with (a + b + c) == 0 (%f + %f + %f == 0)",
                  p.x, p.y, p.z);
        }
        
        t = -p.w / t;
        Assert(!isnan(t));
        
        // p = a*x + b*y + c*z + d == 0
        // -> a point:
        // x = y = z = -d / (a + b + c)
        // given (a + b + c) != 0
        
        return Point(t, t, t);
    }
    
    inline Vector getNormal(const Vector4f &p) {
        return Vector(p.x, p.y, p.z);
    }
    
    enum BBoxPlaneResult {
        FRONT, BACK, BOTH
    };
    
    inline float Distance(const Vector4f &plane, const Point &point) {
        return (plane * point) / (float) sqrt(plane.x * plane.x + plane.y * plane.y + plane.z * plane.z);
    }
    
    inline BBoxPlaneResult operator*(const BBox &box, const Vector4f &plane) {
        // cf http://replay.googlecode.com/svn-history/r57/trunk/source/aabb.cpp
        
        float tmp[2];
        unsigned int mask;
        {
            float t = Distance(plane, Point(0, 0, 0));
            tmp[0] = tmp[1] = t;
        }

        for (int i = 0; i < 3; i++) {
            mask = plane[i] > 0.f ? 0 : 1;
            
            tmp[mask] += box.pMax[i] * plane[i];
            tmp[mask ^ 1] += box.pMin[i] * plane[i];
        }
    
        if (tmp[0] > tmp[1])
            swap(tmp[0], tmp[1]);
        
        if (sign(tmp[0]) != sign(tmp[1]))
            return BOTH;

        if (tmp[1] > 0.f)
            return FRONT;
        return BACK;
    }
    
    inline BBoxPlaneResult operator*(const Vector4f &plane, const BBox &box) {
        return box * plane;
    }
    
}

#endif
