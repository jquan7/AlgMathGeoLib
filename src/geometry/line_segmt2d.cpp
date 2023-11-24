/*******************************************************************************
Copyright © Toramon Co., Ltd. 2017-2023. All Rights Reserved.
File name    : line_segmt2d.cpp
Author       : qijianquan qijq@toramon.com
Version      :
Date         : 2023-11-21 15:15:15
LastEditors  :
LastEditTime :
Description  : line_segment 2d
Others       :
Log          :
*******************************************************************************/

#include "line_segmt2d.h"
#include "AABB2d.h"
#include "circle2d.h"
#include "line2d.h"
#include "OBB2d.h"
#include "polygon2d.h"
#include "rect2d.h"

GEO_BEGIN_NAMESPACE

using namespace trm::math;

// constructor
TRM_linesg2d_c::TRM_linesg2d_c(const TRM_vec2d_c &a_, const TRM_vec2d_c &b_)
:a(a_), b(b_)
{
}

// Returns a point on the line.
TRM_vec2d_c TRM_linesg2d_c::GetPoint(float d) const {
    return a * (1.f - d) + b * d;
}
TRM_vec2d_c TRM_linesg2d_c::CenterPoint() const {
    return (a + b) * 0.5f;
}

// Reverses the direction. (swaps the start and end points)
void TRM_linesg2d_c::Reverse() {
    std::swap(a, b);
}

// Returns the normalized direction vector a->b
TRM_vec2d_c TRM_linesg2d_c::Dir() const {
    return (b - a).Normalized();
}

// Computes the length
float TRM_linesg2d_c::Length() const {
    return a.Distance(b);
}
float TRM_linesg2d_c::LengthSq() const {
    return a.DistanceSq(b);
}

// Tests if 2 line are same
bool TRM_linesg2d_c::Equal(const TRM_linesg2d_c &rhs) const {
    return a.Equal(rhs.a) && b.Equal(rhs.b);
}

// Tests if the given point or line segment is contained
bool TRM_linesg2d_c::Contains(const TRM_vec2d_c &point) const {
    return EqZero(ClosestPoint(point).DistanceSq(point));
}
bool TRM_linesg2d_c::Contains(const TRM_linesg2d_c &other) const {
    return Contains(other.a) && Contains(other.b);
}

// Computes the distance between this line segment and the given object.
float TRM_linesg2d_c::Distance(const TRM_vec2d_c &point) const {
    float d;
    return Distance(point, d);
}
float TRM_linesg2d_c::Distance(const TRM_linesg2d_c &other) const {
    float d, d2;
    return Distance(other, d, d2);
}
float TRM_linesg2d_c::Distance(const TRM_linesg2d_c &other, float &d) const {
    float d2;
    return Distance(other, d, d2);
}
float TRM_linesg2d_c::Distance(const TRM_vec2d_c &point, float &d) const {
    TRM_vec2d_c closest = ClosestPoint(point, d);
    return closest.Distance(point);
}
float TRM_linesg2d_c::Distance(const TRM_linesg2d_c &other, float &d, float &d2) const {
    ClosestPoint(other, d, d2);
    return GetPoint(d).Distance(other.GetPoint(d2));
}
float TRM_linesg2d_c::DistanceSq(const TRM_vec2d_c &point) const {
    float d;
    TRM_vec2d_c closestPoint = ClosestPoint(point, d);
    return closestPoint.DistanceSq(point);
}
float TRM_linesg2d_c::DistanceSq(const TRM_linesg2d_c &other) const {
    float d, d2;
    ClosestPoint(other, d, d2);
    return GetPoint(d).DistanceSq(other.GetPoint(d2));
}

// Tests if 2 line segment are intersect.
bool TRM_linesg2d_c::Intersects(const TRM_linesg2d_c &other) const {
    EqZero(Distance(other));
}

// Computes the closest point on this line segment to the given object.
TRM_vec2d_c TRM_linesg2d_c::ClosestPoint(const TRM_vec2d_c &point) const {
    float d;
    return ClosestPoint(point, d);
}
TRM_vec2d_c TRM_linesg2d_c::ClosestPoint(const TRM_vec2d_c &point, float &d) const {
    TRM_vec2d_c dir = b - a;
    d = Clamp01(Dot(point - a, dir) / dir.LengthSq());
    return a + dir * d;
}
TRM_vec2d_c TRM_linesg2d_c::ClosestPoint(const TRM_linesg2d_c &other) const {
    float d, d2;
    return ClosestPoint(other, d, d2);
}
TRM_vec2d_c TRM_linesg2d_c::ClosestPoint(const TRM_linesg2d_c &other, float &d) const {
    float d2;
    return ClosestPoint(other, d, d2);
}
TRM_vec2d_c TRM_linesg2d_c::ClosestPoint(const TRM_linesg2d_c &other, float &d, float &d2) const {
    TRM_vec2d_c dir = b - a;
    Line2DClosestPointLineLine(a, dir, other.a, other.b - other.a, d, d2);
    if (d >= 0.f && d <= 1.f && d2 >= 0.f && d2 <= 1.f)
        return a + dir * d;
    // Only d2 is out of bounds.
    else if (d >= 0.f && d <= 1.f) {
        TRM_vec2d_c p;
        if (d2 < 0.f) {
            d2 = 0.f, p = other.a;
        }
        else {
            d2 = 1.f, p = other.b;
        }
        return ClosestPoint(p, d);
    }
    // Only d is out of bounds.
    else if (d2 >= 0.f && d2 <= 1.f) {
        TRM_vec2d_c p;
        if (d < 0.f) {
            d = 0.f, p = a;
        }
        else {
            d = 1.f, p = b;
        }
        other.ClosestPoint(p, d2);
        return p;
    }
    // Both u and u2 are out of bounds.
    else  {
        TRM_vec2d_c p, p2;
        if (d < 0.f) {
            p = a, d = 0.f;
        }
        else {
            p = b, d = 1.f;
        }

        if (d2 < 0.f) {
            p2 = other.a, d2 = 0.f;
        }
        else {
            p2 = other.b, d2 = 1.f;
        }

        float T, T2;
        TRM_vec2d_c closestPoint = ClosestPoint(p2, T);
        TRM_vec2d_c closestPoint2 = other.ClosestPoint(p, T2);
        if (closestPoint.DistanceSq(p2) <= closestPoint2.DistanceSq(p)) {
            d = T;
            return closestPoint;
        }
        else {
            d2 = T2;
            return p;
        }
    }
}

void TRM_linesg2d_c::Line2DClosestPointLineLine(const TRM_vec2d_c &v0, const TRM_vec2d_c &v10,
    const TRM_vec2d_c &v2, const TRM_vec2d_c &v32, float &d, float &d2) const {
    d = d2 = 0;
    if (v10.IsZero() || v32.IsZero())
        return;
    TRM_vec2d_c v02 = v0 - v2;
    float d0232 = v02.Dot(v32);
    float d3210 = v32.Dot(v10);
    float d3232 = v32.Dot(v32);
    if (EqZero(d3232))
        return;
    float d0210 = v02.Dot(v10);
    float d1010 = v10.Dot(v10);
    float denom = d1010*d3232 - d3210*d3210;
    if (denom != 0.f)
        d = (d0232*d3210 - d0210*d3232) / denom;
    else
        d = 0.f;
    d2 = (d0232 + d * d3210) / d3232;
}

GEO_END_NAMESPACE
