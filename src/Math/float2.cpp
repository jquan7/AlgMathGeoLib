/*******************************************************************************
Copyright © Toramon Co., Ltd. 2017-2023. All Rights Reserved.
File name    : float2.cpp
Author       : qijianquan qijq@toramon.com
Version      :
Date         : 2023-11-20 15:15:15
LastEditors  :
LastEditTime :
Description  : A 2D (x,y) ordered pair.
Others       :
Log          :
*******************************************************************************/
#include "float2.h"
#include "float2.inl"
#include "float3.h"
#include "float4.h"
#include "float3x3.h"
#include "float3x4.h"
#include "float4x4.h"
#include "MathFunc.h"
#include <string.h>
#include <stdlib.h>
#include <locale.h>

#ifdef MATH_ENABLE_STL_SUPPORT
#include "myassert.h"
#include <iostream>
#include <utility>
#include <algorithm>
#endif

MATH_BEGIN_NAMESPACE

using namespace std;

float2::float2(float x_, float y_)
:x(x_), y(y_)
{
}

float2::float2(float scalar)
:x(scalar), y(scalar)
{
}

float2::float2(const float *data)
{
    if (data) {
        x = data[0], y = data[1];
    } else {
        x = y = 0;
    }
}

float2 float2::Swizzled(int i, int j) const
{
    return float2(At(i), At(j));
}

float3 float2::Swizzled(int i, int j, int k) const
{
    return float3(At(i), At(j), At(k));
}

float4 float2::Swizzled(int i, int j, int k, int l) const
{
    return float4(At(i), At(j), At(k), At(l));
}

float float2::LengthSq() const
{
    return x*x + y*y;
}

float float2::Length() const
{
    return Sqrt(LengthSq());
}

void float2::SetFromPolarCoordinates(float theta, float length)
{
    float sin, cos;
    SinCos(theta, sin, cos);
    x = cos * length;
    y = sin * length;
}

float2 float2::FromPolarCoordinates(float theta, float length)
{
    float2 euclidean;
    euclidean.SetFromPolarCoordinates(theta, length);
    return euclidean;
}

float2 float2::ToPolarCoordinates() const
{
    float radius = Length();
    if (radius > 1e-4f)
        return float2(atan2(y, x), radius);
    else
        return float2::zero;
}

float float2::AimedAngle() const
{
    if (IsZero())
        return 0;
    return atan2(y, x);
}

// Will always produce a normalized vector.
float float2::Normalize()
{
    if (!IsFinite()) {
        Set(1.f, 0.f);
        return 0;
    }
    float len_sq = LengthSq();
    if (len_sq > 1e-6f) {
        float length = Sqrt(len_sq);
        *this *= 1.f / length;
        return length;
    } else {
        Set(1.f, 0.f);
        return 0;
    }
}

float2 float2::Normalized() const
{
    float2 copy = *this;
    float old_len = copy.Normalize();
    if (old_len == 0.f)
        return float2::unitX;
    return copy;
}

float float2::ScaleToLength(float new_len)
{
    float length = LengthSq();
    if (length < 1e-6f)
        return 0.f;

    length = Sqrt(length);
    float scalar = new_len / length;
    *this *= scalar;
    return length;
}

float2 float2::ScaledToLength(float new_len) const
{
    if (IsZero())
        return float2(new_len, 0);
    float2 v = *this;
    v.ScaleToLength(new_len);
    return v;
}

bool float2::IsNormalized(float epsilon) const
{
    return MATH_NS::Abs(LengthSq()-1.f) <= epsilon;
}

bool float2::IsZero(float epsilon) const
{
    return LengthSq() <= epsilon;
}

bool float2::IsFinite() const
{
    return MATH_NS::IsFinite(x) && MATH_NS::IsFinite(y);
}

bool float2::IsPerpendicular(const float2 &other, float epsilon) const
{
    float dot = Dot(other);
    return dot*dot <= epsilon * LengthSq() * other.LengthSq();
}

bool float2::Equals(const float2 &rhs, float epsilon) const
{
    return EqualAbs(x, rhs.x, epsilon) && EqualAbs(y, rhs.y, epsilon);
}

bool float2::Equals(float x_, float y_, float epsilon) const
{
    return EqualAbs(x, x_, epsilon) && EqualAbs(y, y_, epsilon);
}

#if defined(MATH_ENABLE_STL_SUPPORT)
std::string float2::ToString() const
{
    char str[256];
    sprintf(str, "(%f, %f)", x, y);
    return str;
}

std::string float2::SerializeToString() const
{
    char str[256];
    char *s = SerializeFloat(x, str);
    *s++ = ',';
    s = SerializeFloat(y, s);
    MARK_UNUSED(s);
    return str;
}

std::string float2::SerializeToCodeString() const
{
    return "float2(" + SerializeToString() + ")";
}
#endif

float float2::SumOfElements() const
{
    return x + y;
}

float float2::ProductOfElements() const
{
    return x * y;
}

float float2::AverageOfElements() const
{
    return (x + y) * 0.5f;
}

float float2::MinElement() const
{
    return MATH_NS::Min(x, y);
}

int float2::MinElementIndex() const
{
    return (x <= y) ? 0 : 1;
}

float float2::MaxElement() const
{
    return  MATH_NS::Max(x, y);
}

int float2::MaxElementIndex() const
{
    return (x > y) ? 0 : 1;
}

float2 float2::Abs() const
{
    return float2(MATH_NS::Abs(x), MATH_NS::Abs(y));
}

float2 float2::Neg() const
{
    return float2(-x, -y);
}

float2 float2::Recip() const
{
    float x_ = (MATH_NS::IsZero(x)) ? 1 : x;
    float y_ = (MATH_NS::IsZero(y)) ? 1 : y;
    return float2(1.f / x_, 1.f / y_);
}

float2 float2::Min(float floor) const
{
    return float2(MATH_NS::Min(x, floor),  MATH_NS::Min(y, floor));
}

float2 float2::Min(const float2 &floor) const
{
    return float2(MATH_NS::Min(x, floor.x),  MATH_NS::Min(y, floor.y));
}

float2 float2::Max(float ceil) const
{
    return float2(MATH_NS::Max(x, ceil),  MATH_NS::Max(y, ceil));
}

float2 float2::Max(const float2 &ceil) const
{
    return float2(MATH_NS::Max(x, ceil.x),  MATH_NS::Max(y, ceil.y));
}

float2 float2::Clamp(const float2 &floor, const float2 &ceil) const
{
    return float2(MATH_NS::Clamp(x, floor.x, ceil.x),  MATH_NS::Clamp(y, floor.y, ceil.y));
}

float2 float2::Clamp(float floor, float ceil) const
{
    return float2(MATH_NS::Clamp(x, floor, ceil),  MATH_NS::Clamp(y, floor, ceil));
}

float2 float2::Clamp01() const
{
    return Clamp(0.f, 1.f);
}

float float2::DistanceSq(const float2 &rhs) const
{
    float dx = x - rhs.x;
    float dy = y - rhs.y;
    return dx * dx + dy * dy;
}

float float2::Distance(const float2 &rhs) const
{
    return Sqrt(DistanceSq(rhs));
}

float float2::Dot(const float2 &rhs) const
{
    return x * rhs.x + y * rhs.y;
}

float2 float2::Perp() const
{
    return float2(y, -x);
}

float float2::PerpDot(const float2 &rhs) const
{
    return x * rhs.y - y * rhs.x;
}

float2 float2::ProjectTo(const float2 &direction) const
{
    if (direction.IsZero())
        return *this;
    return direction * this->Dot(direction) / direction.LengthSq();
}

float2 float2::ProjectToNorm(const float2 &direction) const
{
    if (!direction.IsNormalized())
        return this->ProjectTo(direction);
    return direction * this->Dot(direction);
}

float float2::AngleBetween(const float2 &other) const
{
    if (this->IsZero() || other.IsZero())
        return 0;
    return acos(Dot(other)) / Sqrt(LengthSq() * other.LengthSq());
}

float float2::AngleBetweenNorm(const float2 &other) const
{
    if (!this->IsNormalized() || !other.IsNormalized())
        return this->AngleBetween(other);
    return acos(Dot(other));
}

float2 float2::Lerp(const float2 &b, float t) const
{
    MATH_NS::Clamp01(t);
    return (1.f - t) * *this + t * b;
}

float2 float2::Lerp(const float2 &a, const float2 &b, float t)
{
    return a.Lerp(b, t);
}

void float2::Decompose(const float2 &direction, float2 &parallel, float2 &perpendicular) const
{
    if (!direction.IsNormalized()) {
        direction.Normalized();
    }
    parallel = this->Dot(direction) * direction;
    perpendicular = *this - parallel;
}

void float2::Orthogonalize(const float2 &a, float2 &b)
{
    if (!a.IsZero())
        b -= a.Dot(b) / a.Length() * a;
}

bool float2::AreOrthogonal(const float2 &a, const float2 &b, float epsilon)
{
    return a.IsPerpendicular(b, epsilon);
}

void float2::Orthonormalize(float2 &a, float2 &b)
{
    if (!a.IsZero()) {
        a.Normalize();
        b -= a.Dot(b) * a;
    }
}

float2 float2::FromScalar(float scalar)
{
    return float2(scalar, scalar);
}

void float2::SetFromScalar(float scalar)
{
    x = scalar;
    y = scalar;
}

void float2::Set(float x_, float y_)
{
    x = x_;
    y = y_;
}

void float2::Rotate90CW()
{
    float x_bak = x;
    x = y;
    y = -x_bak;
}

float2 float2::Rotated90CW() const
{
    return float2(y, -x);
}

void float2::Rotate90CCW()
{
    float x_bak = x;
    x = -y;
    y = x_bak;
}

float2 float2::Rotated90CCW() const
{
    return float2(-y, x);
}

bool float2::OrientedCCW(const float2 &a, const float2 &b, const float2 &c)
{
    // Compute the determinant
    // | ax ay 1 |
    // | bx by 1 |
    // | cx cy 1 |
    // See Christer Ericson, Real-Time Collision Detection, p.32.
    return (a.x-c.x)*(b.y-c.y) - (a.y-c.y)*(b.x-c.x) >= 0.f;
}

float2 float2::operator +(const float2 &rhs) const
{
    return float2(x + rhs.x, y + rhs.y);
}

float2 float2::operator -(const float2 &rhs) const
{
    return float2(x - rhs.x, y - rhs.y);
}

float2 float2::operator -() const
{
    return float2(-x, -y);
}

float2 float2::operator *(float scalar) const
{
    return float2(x * scalar, y * scalar);
}

float2 operator *(float scalar, const float2 &rhs)
{
    return float2(scalar * rhs.x, scalar * rhs.y);
}

float2 float2::operator /(float scalar) const
{
    float inv_scalar = (MATH_NS::IsZero(scalar)) ? 1.f : 1.f / scalar;
    return float2(x * inv_scalar, y * inv_scalar);
}

float2 &float2::operator =(const float2 &rhs)
{
    x = rhs.x;
    y = rhs.y;
    return *this;
}

float2 &float2::operator +=(const float2 &rhs)
{
    x += rhs.x;
    y += rhs.y;
    return *this;
}

float2 &float2::operator -=(const float2 &rhs)
{
    x -= rhs.x;
    y -= rhs.y;
    return *this;
}

float2 &float2::operator *=(float scalar)
{
    x *= scalar;
    y *= scalar;
    return *this;
}

float2 float2::Add(float s) const
{
    return float2(x + s, y + s);
}

float2 float2::Sub(float s) const
{
    return float2(x - s, y - s);
}

float2 float2::SubLeft(float s) const
{
    return float2(s - x, s - y);
}

float2 float2::DivLeft(float s) const
{
    float x_ = (MATH_NS::IsZero(x)) ? x : s / x;
    float y_ = (MATH_NS::IsZero(y)) ? y : s / y;
    return float2(x_, y_);
}

float2 float2::Mul(const float2 &rhs) const
{
    return float2(x * rhs.x, y * rhs.y);
}

float2 float2::Div(const float2 &rhs) const
{
    float x_ = (MATH_NS::IsZero(rhs.x)) ? x : x / rhs.x;
    float y_ = (MATH_NS::IsZero(rhs.y)) ? y : y / rhs.y;
    return float2(x_, y_);
}

float2 &float2::operator /=(float scalar)
{
    float inv_scalar = (MATH_NS::IsZero(scalar)) ? 1.f : 1.f / scalar;
    x *= inv_scalar;
    y *= inv_scalar;
    return *this;
}

#ifdef MATH_ENABLE_STL_SUPPORT
std::ostream &operator <<(std::ostream &out, const float2 &rhs)
{
    std::string str = rhs.ToString();
    out << str;
    return out;
}
#endif

float2 Mul2D(const float3x3 &transform, const float2 &v) {
    return transform.Transform(v.x, v.y, 0.f).xy();
}
float2 MulPos2D(const float3x4 &transform, const float2 &v) {
    return transform.Transform(float4(v.x, v.y, 0.f, 1.f)).xy();
}
float2 MulPos2D(const float4x4 &transform, const float2 &v) {
    return transform.Transform(float4(v.x, v.y, 0.f, 1.f)).xy();
}
float2 MulDir2D(const float3x4 &transform, const float2 &v) {
    return transform.Transform(float4(v.x, v.y, 0.f, 0.f)).xy();
}
float2 MulDir2D(const float4x4 &transform, const float2 &v) {
    return transform.Transform(float4(v.x, v.y, 0.f, 0.f)).xy();
}

const float2 float2::zero = float2(0, 0);
const float2 float2::one = float2(1, 1);
const float2 float2::unitX = float2(1, 0);
const float2 float2::unitY = float2(0, 1);
const float2 float2::nan = float2(FLOAT_NAN, FLOAT_NAN);
const float2 float2::inf = float2(FLOAT_INF, FLOAT_INF);

MATH_END_NAMESPACE
