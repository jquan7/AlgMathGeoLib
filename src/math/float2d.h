/*******************************************************************************
Copyright © Toramon Co., Ltd. 2017-2023. All Rights Reserved.
File name    : float2d.h
Author       : qijianquan qijq@toramon.com
Version      :
Date         : 2023-11-21 15:15:15
LastEditors  :
LastEditTime :
Description  : 常用函数
Others       :
Log          :
*******************************************************************************/

#pragma once

#include <cmath>
#include <algorithm>

#include "../namespaces.h"
#include "constants.h"
#include "func.h"

MATH_BEGIN_NAMESPACE

// vector of form (x,y).
class TRM_float2d_c {
public:
    enum {
        Size = 2  // number of elements
    };
    float x, y;  // components

    /* Constructors */
    // initialize
    TRM_float2d_c() { x = 0, y = 0; }

    TRM_float2d_c(float x_, float y_);

    // copy constructor
    TRM_float2d_c(const TRM_float2d_c &rhs) { x = rhs.x, y = rhs.y; }

    // Casts to a C array
    inline float *ptr() { return &x; }
    inline const float *ptr() const { return &x; }

    /* operators +,-,*,/,=,+=,-=,*=,/=,>,== */
    TRM_float2d_c operator +() const { return *this; }
    TRM_float2d_c operator +(const TRM_float2d_c &v) const;  // (x + v.x, y + v.y)
    TRM_float2d_c operator -() const;  // (-x, -y)
    TRM_float2d_c operator -(const TRM_float2d_c &v) const;  // (x - v.x, y - v.y)
    TRM_float2d_c operator *(float scalar) const;  // (x * scalar, y * scalar)
    TRM_float2d_c operator *(const TRM_float2d_c &v) const;  // (x * v.x, y * v.y)
    TRM_float2d_c operator /(float scalar) const;  // (x / scalar, y / scalar)
    TRM_float2d_c operator /(const TRM_float2d_c &v) const;  // (x / v.x, y / v.y)
    TRM_float2d_c &operator =(const TRM_float2d_c &v);  // Assign
    TRM_float2d_c &operator +=(const TRM_float2d_c &v);  // Adds a vector from this vector, in-place
    TRM_float2d_c &operator -=(const TRM_float2d_c &v);  // Subtracts a vector from this vector, in-place
    TRM_float2d_c &operator *=(float scalar);  // Multiplies this vector by a scalar, in-place.
    TRM_float2d_c &operator *=(const TRM_float2d_c &v);  // Multiplies this vector by a vector, in-place.
    TRM_float2d_c &operator /=(float scalar);  // Divides this vector by a scalar, in-place.
    TRM_float2d_c &operator /=(const TRM_float2d_c &v);  // Divides this vector by a vector, in-place.
    bool operator >(const TRM_float2d_c& v) { return this->LengthSq() > v.LengthSq(); }
    bool operator ==(const TRM_float2d_c& v) { return this->Equal(v); }

    // Sets all elements of this vector.
    void Set(float x_, float y_) {x = x_, y = y_;};

    // Computes the length of this vector.
    float Length() const { return std::sqrt(this->LengthSq()); };

    // Computes the squared length of this vector.
    float LengthSq() const { return x*x + y*y; };

    // Normalize
    float Normalize();
    TRM_float2d_c Normalized() const;

    // Scale
    float ScaleToLength(float newLength);

    // Tests if the length of this vector is 1
    bool IsNormalized() const { return Equal(LengthSq(), 1.f); };

    // Tests if this is the null vector
    bool IsZero() const { return EqZero(LengthSq()); };

    // Tests if two vectors are perpendicular to each other.
    bool IsPerpendicular(const TRM_float2d_c &other, float epsilonSq = CEPS) const;

    // Tests if two vectors are equal.
    bool Equal(const TRM_float2d_c &other) const { return (Equal(x, other.x) && Equal(y, other.y)); };
    bool Equal(float x_, float y_) const { return (Equal(x, x_) && Equal(y, y_)); };

    // Element-wise func.
    TRM_float2d_c Abs() const { return TRM_float2d_c(std::abs(x), std::abs(y)); };  // (|x|, |y|)
    TRM_float2d_c Neg() const { return TRM_float2d_c(-x, -y); };;  // (-x, -y)
    TRM_float2d_c Recip() const;  // (1/x, 1/y)

    TRM_float2d_c Clamp(float floor, float ceil) const {
        return TRM_float2d_c(math::Clamp(x, floor, ceil), math::Clamp(y, floor, ceil));
    };
    TRM_float2d_c Clamp(const TRM_float2d_c &floor, const TRM_float2d_c &ceil) const
    {
        return TRM_float2d_c(math::Clamp(x, floor.x, ceil.x), math::Clamp(y, floor.y, ceil.y));
    }
    TRM_float2d_c Clamp01() const { return Clamp(0.f, 1.f); }

    TRM_float2d_c Min(float floor) const {
	    return TRM_float2d_c(std::min(x, floor), std::min(y, floor));
    }
    TRM_float2d_c Min(const TRM_float2d_c &floor) const {
        return TRM_float2d_c(std::min(x, floor.x), std::min(y, floor.y));
    }
    TRM_float2d_c Max(float ceil) const {
        return TRM_float2d_c(std::max(x, ceil),  std::max(y, ceil));
    }
    TRM_float2d_c Max(const TRM_float2d_c &ceil) const {
        return TRM_float2d_c(std::max(x, ceil.x),  std::max(y, ceil.y));
    }

    // Computes the distance
    float Distance(const TRM_float2d_c &point) const;
    float DistanceSq(const TRM_float2d_c &point) const;  // squared distance

    // Computes the product
    float Dot(const TRM_float2d_c &v) const;  // dot product
    float Cross(const TRM_float2d_c &v) const;  // cross product

    // Projects this vector
    TRM_float2d_c ProjectTo(const TRM_float2d_c &direction) const;  // given unnormalized direction

    // Angle between two vectors, in radians
    float AngleBetween(const TRM_float2d_c &other) const;  // given unnormalized direction

    // Tests if the points p1, p2 and p3 lie on a straight line
    static bool AreCollinear(const TRM_float2d_c &p1, const TRM_float2d_c &p2, const TRM_float2d_c &p3);

    static const TRM_float2d_c zero;  // (0, 0)
    static const TRM_float2d_c one;  // (1, 1)
    static const TRM_float2d_c unitX;  // (1, 0)
    static const TRM_float2d_c unitY;  // (0, 1)
};

inline float Dot(const TRM_float2d_c &a, const TRM_float2d_c &b) { return a.Dot(b); }
inline float Cross(const TRM_float2d_c &a, const TRM_float2d_c &b) { return a.Cross(b); }
inline TRM_float2d_c Abs(const TRM_float2d_c &a) { return a.Abs(); }
inline float Length(const TRM_float2d_c &a) { return a.Length(); }
inline float Distance(const TRM_float2d_c &a, const TRM_float2d_c &b) { return a.Distance(b); }
inline TRM_float2d_c Min(const TRM_float2d_c &a, const TRM_float2d_c &b) { return a.Min(b); }
inline TRM_float2d_c Max(const TRM_float2d_c &a, const TRM_float2d_c &b) { return a.Max(b); }
inline TRM_float2d_c Clamp(const TRM_float2d_c &a, float floor, float ceil) { return a.Clamp(floor, ceil); }
inline TRM_float2d_c Clamp(const TRM_float2d_c &a, const TRM_float2d_c &floor, const TRM_float2d_c &ceil) { return a.Clamp(floor, ceil); }
inline TRM_float2d_c Clamp01(const TRM_float2d_c &a) { return a.Clamp01(); }
inline TRM_float2d_c ProjectTo(const TRM_float2d_c &a, const TRM_float2d_c &b) { return a.ProjectTo(b); }

MATH_END_NAMESPACE
