/*******************************************************************************
Copyright © Toramon Co., Ltd. 2017-2023. All Rights Reserved.
File name    : line2d.cpp
Author       : qijianquan qijq@toramon.com
Version      :
Date         : 2023-11-21 15:15:15
LastEditors  :
LastEditTime :
Description  : line2d
Others       :
Log          :
*******************************************************************************/

#pragma once

#include "line2d.h"
#include "AABB2d.h"
#include "circle2d.h"
#include "line_segmt2d.h"
#include "OBB2d.h"
#include "polygon2d.h"
#include "rect2d.h"

GEO_BEGIN_NAMESPACE

// constructor
TRM_line2d_c::TRM_line2d_c(const TRM_vec2d_c &pos_, const TRM_vec2d_c &dir_)
:pos(pos_), dir(dir_) {
    dir.Normalize();
}
TRM_line2d_c::TRM_line2d_c(const TRM_linesg2d_c &line_segment) {
    pos = line_segment.a;
    dir = (line_segment.b - line_segment.a).Normalized();
}

// Gets a point along the line at the given distance.
TRM_vec2d_c TRM_line2d_c::GetPoint(float d) const {
    return pos + (dir * d);
}

// Tests if the given object is fully contained on this line.
bool TRM_line2d_c::Contains(const TRM_vec2d_c &point) const {
    return EqZero(ClosestPoint(point).DistanceSq(point));
}
bool TRM_line2d_c::Contains(const TRM_linesg2d_c &line_segment) const {
    return Contains(line_segment.a) && Contains(line_segment.b);
}

// Tests if two lines are equal.
bool TRM_line2d_c::Equal(const TRM_line2d_c &line) const {
    return Contains(line.pos) && math::Equal(std::abs(dir.Dot(line.dir)), 1.f);
}

// Computes the distance between this line and the given object.
float TRM_line2d_c::Distance(const TRM_vec2d_c &point) const {
    float d;
    return Distance(point, d);
}
float TRM_line2d_c::Distance(const TRM_line2d_c &other) const {
    float d, d2;
    return Distance(other, d, d2);
}
float TRM_line2d_c::Distance(const TRM_line2d_c &other, float &d) const {
    float d2;
    return Distance(other, d, d2);
}
float TRM_line2d_c::Distance(const TRM_linesg2d_c &other) const {
    float d, d2;
    return Distance(other, d, d2);
}
float TRM_line2d_c::Distance(const TRM_linesg2d_c &other, float &d) const {
    float d2;
    return Distance(other, d, d2);
}
float TRM_line2d_c::Distance(const TRM_vec2d_c &point, float &d) const {
    return ClosestPoint(point, d).Distance(point);
}
float TRM_line2d_c::Distance(const TRM_line2d_c &other, float &d, float &d2) const {
    TRM_vec2d_c c = ClosestPoint(other, d, d2);
    return c.Distance(other.GetPoint(d2));
}
float TRM_line2d_c::Distance(const TRM_linesg2d_c &other, float &d, float &d2) const {
    TRM_vec2d_c c = ClosestPoint(other, d, d2);
    return c.Distance(other.GetPoint(d2));
}

// Computes the closest point on this line to the given object.
TRM_vec2d_c TRM_line2d_c::ClosestPoint(const TRM_vec2d_c &point) const {
    float d;
    return ClosestPoint(point, d);
}
TRM_vec2d_c TRM_line2d_c::ClosestPoint(const TRM_line2d_c &other) const {
    float d, d2;
    return ClosestPoint(other, d, d2);
}
TRM_vec2d_c TRM_line2d_c::ClosestPoint(const TRM_line2d_c &other, float &d) const {
    float d2;
    return ClosestPoint(other, d, d2);
}
TRM_vec2d_c TRM_line2d_c::ClosestPoint(const TRM_linesg2d_c &other) const {
    float d, d2;
    return ClosestPoint(other, d, d2);
}
TRM_vec2d_c TRM_line2d_c::ClosestPoint(const TRM_linesg2d_c &other, float &d) const {
    float d2;
    return ClosestPoint(other, d, d2);
}
TRM_vec2d_c TRM_line2d_c::ClosestPoint(const TRM_vec2d_c &point, float &d) const {
    d = Dot(point - pos, dir);
    return GetPoint(d);
}
TRM_vec2d_c TRM_line2d_c::ClosestPoint(const TRM_line2d_c &other, float &d, float &d2) const {
    ClosestPointLineLine(pos, dir, other.pos, other.dir, d, d2);
    return GetPoint(d);
}
TRM_vec2d_c TRM_line2d_c::ClosestPoint(const TRM_linesg2d_c &other, float &d, float &d2) const {
    ClosestPointLineLine(pos, dir, other.a, other.b - other.a, d, d2);
    if (d2 < 0.f) {
        d2 = 0.f;
        return ClosestPoint(other.a, d);
    }
    else if (d2 > 1.f) {
        d2 = 1.f;
        return ClosestPoint(other.b, d);
    }
    else {
        return GetPoint(d);
    }
}

// Tests whether this line and the given object intersect.
bool TRM_line2d_c::Intersects(const TRM_AABB2d_c &aabb, float &enter, float &exit) const {
    return aabb.Intersects(*this, enter, exit);
}
bool TRM_line2d_c::Intersects(const TRM_AABB2d_c &aabb) const {
    return aabb.Intersects(*this);
}
bool TRM_line2d_c::Intersects(const TRM_OBB2d_c &obb, float &enter, float &exit) const {
    return obb.Intersects(*this, enter, exit);
}
bool TRM_line2d_c::Intersects(const TRM_OBB2d_c &obb) const {
    return obb.Intersects(*this);
}
bool TRM_line2d_c::Intersects(const TRM_polygon2d_c &polygon) const {
    return polygon.Intersects(*this);
}

// Converts this Line to a LineSegment.
TRM_linesg2d_c TRM_line2d_c::ToLineSegment(float d) const {
    return TRM_linesg2d_c(pos, GetPoint(d));
}
TRM_linesg2d_c TRM_line2d_c::ToLineSegment(float start, float end) const {
    return TRM_linesg2d_c(GetPoint(start), GetPoint(end));
}

// Tests if the given three points are collinear.
bool TRM_line2d_c::AreCollinear(const TRM_vec2d_c &p1, const TRM_vec2d_c &p2, const TRM_vec2d_c &p3) {
    TRM_vec2d_c::AreCollinear(p1, p2, p3);
}

void TRM_line2d_c::ClosestPointLineLine(const TRM_vec2d_c &v0, const TRM_vec2d_c &v10,
    const TRM_vec2d_c &v2, const TRM_vec2d_c &v32, float &d, float &d2)
{
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
    if (NeZero(denom))
        d = (d0232*d3210 - d0210*d3232) / denom;
    else
        d = 0.f;
    d2 = (d0232 + d * d3210) / d3232;
}

GEO_END_NAMESPACE
