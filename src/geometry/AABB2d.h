/*******************************************************************************
Copyright © Toramon Co., Ltd. 2017-2023. All Rights Reserved.
File name    : AABB2d.h
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

#include "geometry_fwd.h"

GEO_BEGIN_NAMESPACE

class TRM_AABB2d_c {
public:
    TRM_float2d_c minpt, maxpt;

    // Constructor
    TRM_AABB2d_c() {}  // minpt and maxpt are all undefined
    TRM_AABB2d_c(const TRM_float2d_c &minpt_, const TRM_float2d_c &maxpt_);
    TRM_AABB2d_c(const TRM_OBB2d_c &obb);

    float Width() const { return maxpt.x - minpt.x; };
    float Height() const { return maxpt.y - minpt.y; };

    TRM_float2d_c CenterPoint() const { return (minpt + maxpt) * 0.5f; };
    TRM_float2d_c Size() const { return maxpt - minpt; };
    TRM_float2d_c HalfSize() const { return Size() * 0.5f; };

    float DistanceSq(const TRM_float2d_c &pt) const;

    // Expands this AABB to enclose the given object.
    void Enclose(const TRM_float2d_c &point);
    void Enclose(const TRM_linesg2d_c &linesg);
    void Enclose(const TRM_float2d_c &minpt_, const TRM_float2d_c &maxpt_);
    void Enclose(const TRM_float2d_c* points, int num);

    void SetFromCenterAndSize(const TRM_float2d_c &center, const TRM_float2d_c &size);
    void SetFrom(const TRM_OBB2d_c &obb);

    // Finds the set intersection of this and the given AABB.
    TRM_AABB2d_c Intersection(const TRM_AABB2d_c &aabb) const;

    // Tests whether this AABB and the given object intersect.
    bool Intersects(const TRM_line2d_c &line, float &near, float &far) const;
    bool Intersects(const TRM_line2d_c &line) const;
    bool Intersects(const TRM_linesg2d_c &linesg, float &near, float &far) const;
    bool Intersects(const TRM_linesg2d_c &linesg) const;
    bool Intersects(const TRM_AABB2d_c &aabb) const;
    bool Intersects(const TRM_OBB2d_c &obb) const;
    bool Intersects(const TRM_polygon2d_c &polygon) const;

    // Tests if the given object is fully contained inside this AABB.
    bool Contains(const TRM_float2d_c &pt) const;
    bool Contains(int x, int y) const;
    bool Contains(const TRM_linesg2d_c &linesg) const;
    bool Contains(const TRM_float2d_c &minpt_, const TRM_float2d_c &maxpt_) const;
    bool Contains(const TRM_AABB2d_c &aabb) const;

    // Tests if this AABB is degenerate.
    bool IsDegenerate() const;
    bool HasNegativeVolume() const;

    TRM_float2d_c PosInside(const TRM_float2d_c &normalizedPos) const;
    TRM_float2d_c ToNormalizedLocalSpace(const TRM_float2d_c &pt) const;

    TRM_AABB2d_c operator +(const TRM_float2d_c &pt) const;
    TRM_AABB2d_c operator -(const TRM_float2d_c &pt) const;

    bool Equal(const TRM_AABB2d_c &rhs) const;

    // Computes the intersection of a line (or line segment) and an AABB.
    // For a Line-AABB test, pass in (near = -FLOAT_INF; far = FLOAT_INF;)
    // For a LineSegment-AABB test, pass in (near = 0.f; far = LineSegment.Length();)
    bool IntersectLineAABB(const TRM_vec2d_c &pos, const TRM_vec2d_c &dir, float &near, float &far) const;
};

GEO_END_NAMESPACE
