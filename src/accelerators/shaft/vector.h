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
    
    typedef Vector3<int> Vector3i;
    
    template <class T>
    class Vector4 {
    public:
        T x, y, z, w;
        
        Vector4() : x(0), y(0), z(0), w(0) {}
        Vector4(T x, T y, T z, T w) : x(x), y(y), z(z), w(w) {}
        ~Vector4();
        
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
    
    float operator*(const Vector3i &o, const Vector4f &p) {
        return o.x * p.x + o.y * p.y + o.z * p.z + p.w;
    }
    
    inline float operator*(const Vector4f &p, const Vector3i &o) {
        return o * p;
    }
    
    Vector4f CreatePlane(const Point &a, const Point &b, const Point&c) {
        ::Vector n = (b - a) ^ (c - a);
        float d = -n.x * a.x - n.y * a.y - n.z * a.z;
        return Vector4f(n.x, n.y, n.z, d);
    }
    
}

#endif
