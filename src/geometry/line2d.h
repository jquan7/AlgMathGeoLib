/*******************************************************************************
Copyright © Toramon Co., Ltd. 2017-2023. All Rights Reserved.
File name    : line2d.h
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

#include "geometry_fwd.h"

GEO_BEGIN_NAMESPACE

class TRM_line2d_c {
public:
    // origin of line.
    TRM_vec2d_c pos;

    // The normalized direction vector
    TRM_vec2d_c dir;

    // constructor
    TRM_line2d_c() {};  // pos and dir are undefined
    TRM_line2d_c(const TRM_vec2d_c &pos_, const TRM_vec2d_c &dir_);
    explicit TRM_line2d_c(const TRM_linesg2d_c &line_segment);

    // Gets a point along the line at the given distance.
    TRM_vec2d_c GetPoint(float d) const;

    // Tests if the given object is fully contained on this line.
	bool Contains(const TRM_vec2d_c &point) const;
	bool Contains(const TRM_linesg2d_c &line_segment) const;

    // Tests if two lines are equal.
    bool Equal(const TRM_line2d_c &line) const;

    // Computes the distance between this line and the given object.
	float Distance(const TRM_vec2d_c &point) const;
	float Distance(const TRM_line2d_c &other) const;
	float Distance(const TRM_line2d_c &other, float &d) const;
	float Distance(const TRM_linesg2d_c &other) const;
	float Distance(const TRM_linesg2d_c &other, float &d) const;
	float Distance(const TRM_vec2d_c &point, float &d) const;
	float Distance(const TRM_line2d_c &other, float &d, float &d2) const;
	float Distance(const TRM_linesg2d_c &other, float &d, float &d2) const;

    // Computes the closest point on this line to the given object.
	TRM_vec2d_c ClosestPoint(const TRM_vec2d_c &point) const;
	TRM_vec2d_c ClosestPoint(const TRM_line2d_c &other) const;
	TRM_vec2d_c ClosestPoint(const TRM_line2d_c &other, float &d) const;
	TRM_vec2d_c ClosestPoint(const TRM_linesg2d_c &other) const;
	TRM_vec2d_c ClosestPoint(const TRM_linesg2d_c &other, float &d) const;
	TRM_vec2d_c ClosestPoint(const TRM_vec2d_c &point, float &d) const;
	TRM_vec2d_c ClosestPoint(const TRM_line2d_c &other, float &d, float &d2) const;
	TRM_vec2d_c ClosestPoint(const TRM_linesg2d_c &other, float &d, float &d2) const;

    // Tests whether this line and the given object intersect.
	bool Intersects(const TRM_AABB2d_c &aabb, float &enter, float &exit) const;
	bool Intersects(const TRM_AABB2d_c &aabb) const;
	bool Intersects(const TRM_OBB2d_c &obb, float &enter, float &exit) const;
	bool Intersects(const TRM_OBB2d_c &obb) const;
	bool Intersects(const TRM_polygon2d_c &polygon) const;

    // Converts this Line to a LineSegment.
	TRM_linesg2d_c ToLineSegment(float d) const;
	TRM_linesg2d_c ToLineSegment(float start, float end) const;

    // Tests if the given three points are collinear.
    static bool AreCollinear(const TRM_vec2d_c &p1, const TRM_vec2d_c &p2, const TRM_vec2d_c &p3);

	static void ClosestPointLineLine(const TRM_vec2d_c &v0, const TRM_vec2d_c &v10,
		const TRM_vec2d_c &v2, const TRM_vec2d_c &v32, float &d, float &d2);
};

GEO_END_NAMESPACE
