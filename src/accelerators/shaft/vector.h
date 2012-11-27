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

int sign(float f) {
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
        
        Vector3 &operator=(Vector3 v) {
            x = v.x; y = v.y; z = v.z;
            return *this;
        }
        
        T &operator[](int i) {
            Assert(i >= 0 && i < 3);
            return (&x)[i];
        }
        
        T operator[](int i) const {
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
        
        Vector4() : x(0), y(0), z(0), w(0) {}
        Vector4(T x, T y, T z, T w) : x(x), y(y), z(z), w(w) {}
        ~Vector4() {};
        
        Vector4<T> &operator=(Vector4 v) {
            x = v.x; y = v.y; z = v.z; w = v.w;
            return *this;
        }
        
        T &operator[](int i) {
            Assert(i >= 0 && i < 4);
            return (&x)[i];
        }
        
        T operator[](int i) const {
            Assert(i >= 0 && i < 4);
            return (&x)[i];
        }
    };
    
    typedef Vector4<float> Vector4f;
    
    float operator*(const Point &o, const Vector4f &p) {
        return o.x * p.x + o.y * p.y + o.z * p.z + p.w;
    }
    
    inline float operator*(const Vector4f &p, const Point &o) {
        return o * p;
    }
    
    Vector4f CreatePlane(const Point &a, const Point &b, const Point&c) {
        ::Vector n = (b - a) ^ (c - a);
        float d = -n.x * a.x - n.y * a.y - n.z * a.z;
        return Vector4f(n.x, n.y, n.z, d);
    }
    
    Point getPointOnPlane(const Vector4f &p) {
        float t = p.x + p.y + p.z;
        Assert(t != 0.f);
        
        if (t == 0) {
            Warning("This function doesn't work for a plane (a*x + b*y + c*z + d == 0) with (a + b + c) == 0");
        }
        
        t = -p.w / t;
        
        // p = a*x + b*y + c*z + d == 0
        // -> a point:
        // x = y = z = -d / (a + b + c)
        // given (a + b + c) != 0
        
        return Point(t, t, t);
    }
    
    Vector getNormal(const Vector4f &p) {
        return Vector(p.x, p.y, p.z);
    }
    
    enum BBoxPlaneResult {
        FRONT, BACK, BOTH
    };
    
    float Distance(const Vector4f &plane, const Point &point) {
        return (plane * point) / (float) sqrt(plane.x * plane.x + plane.y * plane.y + plane.z * plane.z);
    }
    
    BBoxPlaneResult operator*(const BBox &box, const Vector4f &plane) {
        // cf http://replay.googlecode.com/svn-history/r57/trunk/source/aabb.cpp
        
        float tmp[2];
        unsigned int mask;
        {
            float t = Distance(plane, Point(0, 0, 0));
            tmp[0] = tmp[1] = t;
        }

        for (int i = 0; i < 3; i++) {
            mask = sign(plane[i]);
            
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
    
    BBoxPlaneResult operator*(const Vector4f &plane, const BBox &box) {
        return box * plane;
    }
    
}

#endif
