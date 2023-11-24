/*******************************************************************************
Copyright © Toramon Co., Ltd. 2017-2023. All Rights Reserved.
File name    : AABB2d.cpp
Author       : qijianquan qijq@toramon.com
Version      :
Date         : 2023-11-21 15:15:15
LastEditors  :
LastEditTime :
Description  : 2D Axis-Aligned Bounding Box
Others       :
Log          :
*******************************************************************************/

#pragma once

#include "AABB2d.h"
#include "circle2d.h"
#include "line_segmt2d.h"
#include "line2d.h"
#include "OBB2d.h"
#include "polygon2d.h"
#include "rect2d.h"

GEO_BEGIN_NAMESPACE

using namespace trm::math;

float TRM_AABB2d_c::DistanceSq(const TRM_float2d_c &pt) const {
    TRM_float2d_c cp = pt.Clamp(minpt, maxpt);
    return cp.DistanceSq(pt);
}

// Expands this AABB to enclose the given object.
void TRM_AABB2d_c::Enclose(const TRM_float2d_c &point) {
    minpt = math::Min(minpt, point);
    maxpt = math::Max(maxpt, point);
}

void TRM_AABB2d_c::Enclose(const TRM_linesg2d_c &linesg) {
    Enclose(math::Min(linesg.a, linesg.b), math::Max(linesg.a, linesg.b));
}

void TRM_AABB2d_c::Enclose(const TRM_float2d_c &minpt_, const TRM_float2d_c &maxpt_) {
    minpt = math::Min(minpt, minpt_);
    maxpt = math::Max(maxpt, maxpt_);
}

void TRM_AABB2d_c::Enclose(const TRM_float2d_c* points, int num) {
    if (!points || 0 == num)
        return;
    for(int i = 0; i < num; ++i)
        Enclose(points[i]);
}

void TRM_AABB2d_c::SetFromCenterAndSize(const TRM_float2d_c &center, const TRM_float2d_c &size) {
    TRM_float2d_c half_size = size * 0.5f;
    minpt = center - half_size;
    maxpt = center + half_size;
}

void TRM_AABB2d_c::SetFrom(const TRM_OBB2d_c &obb) {
    TRM_float2d_c half_size = math::Abs(obb.axis[0] * obb.r.x) + math::Abs(obb.axis[1] * obb.r.y);
    SetFromCenterAndSize(obb.pos, half_size * 2.f);
}

// Finds the set intersection of this and the given AABB.
TRM_AABB2d_c TRM_AABB2d_c::Intersection(const TRM_AABB2d_c &aabb) const {
    return TRM_AABB2d_c(math::Max(minpt, aabb.minpt), math::Min(maxpt, aabb.maxpt));
}

// Tests whether this AABB and the given object intersect.
bool TRM_AABB2d_c::Intersects(const TRM_line2d_c &line, float &near, float &far) const {
    near = -INF, far = INF;
    return IntersectLineAABB(line.pos, line.dir, near, far);
}

bool TRM_AABB2d_c::Intersects(const TRM_line2d_c &line) const {
    float near = -INF, far = INF;
    return IntersectLineAABB(line.pos, line.dir, near, far);
}

bool TRM_AABB2d_c::Intersects(const TRM_linesg2d_c &linesg, float &near, float &far) const {
    near = -INF, far = INF;
    TRM_vec2d_c dir = linesg.b - linesg.a;
    float len = dir.Length();
    if (math::EqZero(len))
        return Contains(linesg.a);
    float inv = 1.f / len;
    near = 0.f, far = len;
    dir *= inv;
    return IntersectLineAABB(linesg.a, dir, near, far);
}

bool TRM_AABB2d_c::Intersects(const TRM_linesg2d_c &linesg) const {
    TRM_vec2d_c dir = linesg.b - linesg.a;
    float len = dir.Length();
    if (math::EqZero(len))
        return Contains(linesg.a);
    float inv = 1.f / len, near = 0.f, far = len;
    dir *= inv;
    return IntersectLineAABB(linesg.a, dir, near, far);
}

bool TRM_AABB2d_c::Intersects(const TRM_AABB2d_c &aabb) const {
    return minpt.x < aabb.maxpt.x && minpt.y < aabb.maxpt.y &&
        aabb.minpt.x < maxpt.x && aabb.minpt.y < maxpt.y;
}

bool TRM_AABB2d_c::Intersects(const TRM_OBB2d_c &obb) const {
    return obb.Intersects(*this);
}

bool TRM_AABB2d_c::Intersects(const TRM_polygon2d_c &polygon) const {
    polygon.Intersects(*this);
}

// Tests if the given object is fully contained inside this AABB.
bool TRM_AABB2d_c::Contains(const TRM_float2d_c &pt) const {
    return pt.x >= minpt.x && pt.y >= minpt.y
        && pt.x <= maxpt.x && pt.y <= maxpt.y;
}

bool TRM_AABB2d_c::Contains(int x, int y) const {
    return x >= minpt.x && y >= minpt.y
        && x <= maxpt.x && y <= maxpt.y;
}

bool TRM_AABB2d_c::Contains(const TRM_linesg2d_c &linesg) const {
    return Contains(math::Min(linesg.a, linesg.b), math::Max(linesg.a, linesg.b));
}

bool TRM_AABB2d_c::Contains(const TRM_float2d_c &minpt_, const TRM_float2d_c &maxpt_) const {
    return minpt_.x >= minpt.x && minpt_.y >= minpt.y
        && maxpt_.x <= maxpt.x && maxpt_.y <= maxpt.y;
}

bool TRM_AABB2d_c::Contains(const TRM_AABB2d_c &aabb) const { return Contains(aabb.minpt, aabb.maxpt); }

// Tests if this AABB is degenerate.
bool TRM_AABB2d_c::IsDegenerate() const {
    return minpt.x >= maxpt.x || minpt.y >= maxpt.y;
}

bool TRM_AABB2d_c::HasNegativeVolume() const {
    return maxpt.x < minpt.x || maxpt.y < minpt.y;
}

TRM_float2d_c TRM_AABB2d_c::PosInside(const TRM_float2d_c &normalizedPos) const {
    return normalizedPos * (maxpt - minpt) + minpt;
}

TRM_float2d_c TRM_AABB2d_c::ToNormalizedLocalSpace(const TRM_float2d_c &pt) const {
    return (pt - minpt) / (maxpt - minpt);
}

TRM_AABB2d_c TRM_AABB2d_c::operator +(const TRM_float2d_c &pt) const {
    TRM_AABB2d_c a;
    a.minpt = minpt + pt;
    a.maxpt = maxpt + pt;
    return a;
}

TRM_AABB2d_c TRM_AABB2d_c::operator -(const TRM_float2d_c &pt) const {
    TRM_AABB2d_c a;
    a.minpt = minpt - pt;
    a.maxpt = maxpt - pt;
    return a;
}

bool TRM_AABB2d_c::Equal(const TRM_AABB2d_c &rhs) const { return minpt.Equal(rhs.minpt) && maxpt.Equal(rhs.maxpt); }

// Computes the intersection of a line (or line segment) and an AABB.
// For a Line-AABB test, pass in (near = -FLOAT_INF; far = FLOAT_INF;)
// For a LineSegment-AABB test, pass in (near = 0.f; far = LineSegment.Length();)
bool TRM_AABB2d_c::IntersectLineAABB(const TRM_vec2d_c &pos, const TRM_vec2d_c &dir, float &near, float &far) const {
    dir.Normalize();
    if (near > far)
        std::swap(near, far);

    // Test each cardinal plane (X, Y) in turn.
    if (math::NeZero(dir.x))
    {
        float recip = 1 / dir.x;
        float t1 = (minpt.x - pos.x) * recip;
        float t2 = (maxpt.x - pos.x) * recip;

        // near tracks distance to intersect (enter) the AABB.
        // far tracks the distance to exit the AABB.
        if (t1 < t2)
            near = std::max(t1, near), far = std::min(t2, far);
        else // Swap t1 and t2.
            near = std::max(t2, near), far = std::min(t1, far);

        if (near > far)
            return false;
    }
    else if (pos.x < minpt.x || pos.x > maxpt.x)
        return false;

    if (math::NeZero(dir.y))
    {
        float recip = 1 / dir.y;
        float t1 = (minpt.y - pos.y) * recip;
        float t2 = (maxpt.y - pos.y) * recip;

        if (t1 < t2)
            near = std::max(t1, near), far = std::min(t2, far);
        else // Swap t1 and t2.
            near = std::max(t2, near), far = std::min(t1, far);

        if (near > far)
            return false;
    }
    else if (pos.y < minpt.y || pos.y > maxpt.y)
        return false;

    return near <= far;
}

GEO_END_NAMESPACE
