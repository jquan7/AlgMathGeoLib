/*******************************************************************************
Copyright © Toramon Co., Ltd. 2017-2023. All Rights Reserved.
File name    : line_segmt2d.h
Author       : qijianquan qijq@toramon.com
Version      :
Date         : 2023-11-21 15:15:15
LastEditors  :
LastEditTime :
Description  : line_segment 2d
Others       :
Log          :
*******************************************************************************/

#pragma once

#include "geometry_fwd.h"

GEO_BEGIN_NAMESPACE

class TRM_linesg2d_c {
public:
    // starting point, end point
    TRM_vec2d_c a, b;

    // constructor
    TRM_linesg2d_c() {}  // a and b are undefined
    TRM_linesg2d_c(const TRM_vec2d_c &a_, const TRM_vec2d_c &b_);

    // Returns a point on the line.
    TRM_vec2d_c GetPoint(float d) const;
    TRM_vec2d_c CenterPoint() const;  // GetPoint(0.5f)

    // Reverses the direction. (swaps the start and end points)
    void Reverse();

    // Returns the normalized direction vector a->b
    TRM_vec2d_c Dir() const;

    // Computes the length
    float Length() const;  // |b-a|
    float LengthSq() const;  // |b-a|^2

    // Tests if 2 line are same
    bool Equal(const TRM_linesg2d_c &rhs) const;

    // Tests if the given point or line segment is contained
    bool Contains(const TRM_vec2d_c &point) const;
    bool Contains(const TRM_linesg2d_c &other) const;

    // Computes the distance between this line segment and the given object.
    float Distance(const TRM_vec2d_c &point) const;
    float Distance(const TRM_linesg2d_c &other) const;
    float Distance(const TRM_linesg2d_c &other, float &d) const;
    float Distance(const TRM_vec2d_c &point, float &d) const;
    float Distance(const TRM_linesg2d_c &other, float &d, float &d2) const;
    float DistanceSq(const TRM_vec2d_c &point) const;
    float DistanceSq(const TRM_linesg2d_c &other) const;

    // Tests if 2 line segment are intersect.
    bool Intersects(const TRM_linesg2d_c &other) const;

    // Computes the closest point on this line segment to the given object.
    TRM_vec2d_c ClosestPoint(const TRM_vec2d_c &point) const;
    TRM_vec2d_c ClosestPoint(const TRM_vec2d_c &point, float &d) const;
    TRM_vec2d_c ClosestPoint(const TRM_linesg2d_c &other) const;
    TRM_vec2d_c ClosestPoint(const TRM_linesg2d_c &other, float &d) const;
    TRM_vec2d_c ClosestPoint(const TRM_linesg2d_c &other, float &d, float &d2) const;

    /**
     * @brief   Computes the closest point pair on two lines.
     * @param   [const TRM_vec2d_c] v0 - The starting point of the first line.
     * @param   [const TRM_vec2d_c] v10 - The direction vector of the first line. This can be unnormalized.
     * @param   [const TRM_vec2d_c] v2 - The starting point of the second line.
     * @param   [const TRM_vec2d_c] v32 - The direction vector of the second line. This can be unnormalized.
     * @param   [float &] d -  Receives the normalized distance of the closest point along the first line.
     * @param   [float &] d2 - Receives the normalized distance of the closest point along the second line.
     * @return  the closest point on line start0<->end0 to the second line.
     */
    void Line2DClosestPointLineLine(const TRM_vec2d_c &v0, const TRM_vec2d_c &v10,
        const TRM_vec2d_c &v2, const TRM_vec2d_c &v32, float &d, float &d2) const;
};

GEO_END_NAMESPACE
