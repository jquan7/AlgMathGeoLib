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

#include "float2d.h"

MATH_BEGIN_NAMESPACE

const TRM_float2d_c TRM_float2d_c::zero = TRM_float2d_c(0, 0);
const TRM_float2d_c TRM_float2d_c::one = TRM_float2d_c(1, 1);
const TRM_float2d_c TRM_float2d_c::unitX = TRM_float2d_c(1, 0);
const TRM_float2d_c TRM_float2d_c::unitY = TRM_float2d_c(0, 1);

/* Constructors */
// initialize
TRM_float2d_c::TRM_float2d_c(float x_, float y_)
:x(x_), y(y_) {}

/* operators +,-,*,/,=,+=,-=,*=,/=,>,== */
TRM_float2d_c TRM_float2d_c::operator +(const TRM_float2d_c &rhs) const
{
    return TRM_float2d_c(x + rhs.x, y + rhs.y);
}

TRM_float2d_c TRM_float2d_c::operator -() const
{
    return TRM_float2d_c(-x, -y);
}

TRM_float2d_c TRM_float2d_c::operator -(const TRM_float2d_c &rhs) const
{
    return TRM_float2d_c(x - rhs.x, y - rhs.y);
}

TRM_float2d_c TRM_float2d_c::operator *(float scalar) const
{
    return TRM_float2d_c(x * scalar, y * scalar);
}

TRM_float2d_c TRM_float2d_c::operator *(const TRM_float2d_c &v) const {
    return TRM_float2d_c(x * v.x, y * v.y);
}

TRM_float2d_c TRM_float2d_c::operator /(float scalar) const
{
    if (NeZero(scalar)) {
        float inv = 1.f / scalar;
        return TRM_float2d_c(x * inv, y * inv);
    }
    return *this;
}

TRM_float2d_c TRM_float2d_c::operator /(const TRM_float2d_c &v) const {
    if (NeZero(v.x) && NeZero(v.y))
        return TRM_float2d_c(x / v.x, y / v.y);
    return *this;
}

TRM_float2d_c &TRM_float2d_c::operator =(const TRM_float2d_c &rhs)
{
    x = rhs.x, y = rhs.y;
    return *this;
}

TRM_float2d_c &TRM_float2d_c::operator +=(const TRM_float2d_c &rhs)
{
    x += rhs.x, y += rhs.y;
    return *this;
}

TRM_float2d_c &TRM_float2d_c::operator -=(const TRM_float2d_c &rhs)
{
    x -= rhs.x, y -= rhs.y;
    return *this;
}

TRM_float2d_c &TRM_float2d_c::operator *=(float scalar)
{
    x *= scalar, y *= scalar;
    return *this;
}
TRM_float2d_c &TRM_float2d_c::operator *=(const TRM_float2d_c &v)
{
    x *= v.x, y *= v.y;
    return *this;
}

TRM_float2d_c &TRM_float2d_c::operator /=(float scalar)
{
    if (NeZero(scalar)) {
        float inv = 1.f / scalar;
        x *= inv, y *= inv;
    }
    return *this;
}

TRM_float2d_c &TRM_float2d_c::operator /=(const TRM_float2d_c &v)
{
    if (NeZero(v.x) && NeZero(v.y))
        x /= v.x, y /= v.y;
    return *this;
}

// Normalize
float TRM_float2d_c::Normalize()
{
    float lensq = LengthSq();
    if (NeZero(lensq))
    {
        float length = std::sqrt(lensq);
        *this *= 1.f / length;
        return length;
    }
    else
    {
        Set(1.f, 0.f);
        return 0;
    }
}

TRM_float2d_c TRM_float2d_c::Normalized() const
{
    TRM_float2d_c copy = *this;
    copy.Normalize();
    return copy;
}

// Scale
float TRM_float2d_c::ScaleToLength(float new_len)
{
    float length = LengthSq();
    if (EqZero(length))
        return 0.f;

    length = std::sqrt(length);
    float scalar = new_len / length;
    x *= scalar, y *= scalar;
    return length;
}


// Tests if two vectors are perpendicular to each other.
bool TRM_float2d_c::IsPerpendicular(const TRM_float2d_c &other, float epsilonSq) const
{
    float dot = Dot(other);
    return dot*dot <= epsilonSq * LengthSq() * other.LengthSq();
}

// Element-wise func.
TRM_float2d_c TRM_float2d_c::Recip() const
{
    if (NeZero(x) && NeZero(y))
        return TRM_float2d_c(1.f / x, 1.f / y);
    return *this;
}

// Computes the distance
float TRM_float2d_c::Distance(const TRM_float2d_c &rhs) const
{
    return std::sqrt(DistanceSq(rhs));
}

float TRM_float2d_c::DistanceSq(const TRM_float2d_c &rhs) const
{
    float dx = x - rhs.x, dy = y - rhs.y;
    return dx * dx + dy * dy;
}

// Dot product
float TRM_float2d_c::Dot(const TRM_float2d_c &rhs) const
{
    return x * rhs.x + y * rhs.y;
}

// cross product
float TRM_float2d_c::Cross(const TRM_float2d_c &rhs) const
{
    return x * rhs.y - y * rhs.x;
}

// Projects this vector
TRM_float2d_c TRM_float2d_c::ProjectTo(const TRM_float2d_c &direction) const
{
    if (direction.IsZero())
        return *this;
    return direction * this->Dot(direction) / direction.LengthSq();
}

// Angle between two vectors, in radians
float TRM_float2d_c::AngleBetween(const TRM_float2d_c &other) const
{
    return acos(Dot(other)) / std::sqrt(LengthSq() * other.LengthSq());
}

bool TRM_float2d_c::AreCollinear(const TRM_float2d_c &p1, const TRM_float2d_c &p2, const TRM_float2d_c &p3)
{
    return EqZero((p2-p1).Cross(p3-p1));
}


MATH_END_NAMESPACE
