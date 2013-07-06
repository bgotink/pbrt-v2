//
//  vector.h
//  pbrt
//
//  Created by Bram Gotink on 31/05/13.
//
//

#ifndef __pbrt__vector__
#define __pbrt__vector__

#include "pbrt.h"
#include "geometry.h"

namespace shaft { namespace surface {

template <class T, size_t Size>
class Vector {
    typedef Vector<T,Size>  self_t;
    typedef T               type_t;
    
protected:
    type_t value[Size];
    
public:
    Vector() {
        memset(value, 0, Size * sizeof(type_t));
    }
    
    Vector(const type_t arr[Size]) {
        memcpy(value, arr, Size * sizeof(type_t));
    }
    
    inline bool operator==(const self_t &v) const {
        return !memcmp(value, v.value, Size * sizeof(type_t));
    }
    inline bool operator!=(const self_t &v) const {
        return !!memcmp(value, v.value, Size * sizeof(type_t));
    }

    inline self_t &operator=(const self_t &v) {
        memcpy(value, v.value, Size * sizeof(type_t));
    }
    
    inline type_t operator[](uint64_t i) const {
        return value[i];
    }
    
    inline type_t &operator[](uint64_t i) {
        return value[i];
    }
    
    self_t operator+(const self_t &v) const {
        self_t result;
        
        for (size_t i = 0; i < Size; i++)
            result[i] = value[i] + v[i];
        
        return result;
    }
    self_t operator-(const self_t &v) const {
        self_t result;
        
        for (size_t i = 0; i < Size; i++)
            result[i] = value[i] - v[i];
        
        return result;
    }
    self_t operator-() const {
        self_t result;
        
        for (size_t i = 0; i < Size; i++)
            result[i] = -value[i];
        
        return result;
    }
    
    self_t &operator+=(const self_t &v) {
        for (size_t i = 0; i < Size; i++)
            value[i] += v[i];
        
        return *this;
    }
    self_t &operator-=(const self_t &v) {
        for (size_t i = 0; i < Size; i++)
            value[i] -= v[i];
        
        return *this;
    }
    
    type_t operator*(const self_t &v) const {
        type_t result;
        
        for (size_t i = 0; i < Size; i++)
            result = value[i] * v[i];
        
        return result;
    }
};
    
template <typename T>
class Vector3 : public Vector<T,3> {
    typedef T           type_t;
    typedef Vector3<T>  self_t;
    typedef Vector<T,3> super_t;
    
public:
    Vector3() : super_t() {}
    Vector3(type_t a, type_t b, type_t c) : super_t() {
        this->value[0] = a;
        this->value[1] = b;
        this->value[2] = c;
    }
};
    
template <typename T>
class Vector4 : public Vector<T,4> {
    typedef T           type_t;
    typedef Vector4<T>  self_t;
    typedef Vector<T,4> super_t;
    
public:
    Vector4() : super_t() {}
    Vector4(type_t a, type_t b, type_t c, type_t d) : super_t() {
        this->value[0] = a;
        this->value[1] = b;
        this->value[2] = c;
        this->value[3] = d;
    }
};
    
typedef Vector3<uint64_t> vector3_uint64_t;
typedef Vector4<uint64_t> vector4_uint64_t;
    
typedef Vector3<float> vector3_float_t;
typedef Vector4<float> vector4_float_t;
    
typedef vector4_float_t plane_t;
typedef ::Vector vector_t;
typedef ::Point point_t;
typedef ::BBox bbox_t;

inline float operator*(const plane_t &plane, const point_t &pt) {
    return pt.x * plane[0] + pt.y * plane[1] + pt.z * plane[2] + plane[3];
}

inline float operator*(const point_t &pt, const plane_t &plane) {
    return plane * pt;
}

inline float operator*(const plane_t &plane, const vector_t &vec) {
    return vec.x * plane[0] + vec.y * plane[1] + vec.z * plane[2];
}

inline float operator*(const vector_t &vec, const plane_t &plane) {
    return plane * vec;
}
    
// plane stuff
    
inline vector_t getNormal(const plane_t &plane) {
    return vector_t(plane[0], plane[1], plane[2]);
}
    
typedef enum { FRONT, BACK, BOTH } PlaneBoxResult;
    
PlaneBoxResult operator*(const plane_t &plane, const bbox_t &bbox);
    
inline PlaneBoxResult operator*(const BBox &bbox, const plane_t &plane) {
    return plane * bbox;
}

}}

#endif /* defined(__pbrt__vector__) */
