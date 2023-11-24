/*******************************************************************************
Copyright © Toramon Co., Ltd. 2017-2023. All Rights Reserved.
File name    : OBB2D.h
Author       : qijianquan qijq@toramon.com
Version      :
Date         : 2023-11-21 15:15:15
LastEditors  :
LastEditTime :
Description  : 2D Oriented Bounding Box
Others       :
Log          :
*******************************************************************************/

#pragma once

#include "geometry_fwd.h"

GEO_BEGIN_NAMESPACE

class TRM_OBB2d_c {
public:
	// Center position.
	TRM_vec2d_c pos;

	// Half-sizes to x, y directions in the local space of this OBB2D.
	TRM_vec2d_c r;

	// Specifies normalized direction vectors for the local axes.
	/** axis[0]: +X direction, axis[1] the +Y direction
		The scale of these vectors is always normalized.
		The axis vectors must always be orthonormal. */
	TRM_vec2d_c axis[2];

	// Constructor
	TRM_OBB2d_c() {}  // pos, r and axis are undefined
	TRM_OBB2d_c(const TRM_vec2d_c &pos_, const TRM_vec2d_c &r_, const TRM_vec2d_c &axis0, const TRM_vec2d_c &axis1);
	TRM_OBB2d_c(const TRM_AABB2d_c &aabb);

	inline static int NumFaces() { return 1; }
	inline static int NumEdges() { return 4; }
	inline static int NumVertices() { return 4; }

	// Sets this OBB2D from an AABB2d.
	void SetFrom(const TRM_AABB2d_c &aabb);
    // void SetFrom(const TRM_AABB2d_c &aabb, float angle);

	// Returns the tightest AABB2d that contains this OBB2D.
	TRM_AABB2d_c MinimalEnclosingAABB() const;

	// Returns the side lengths.
	TRM_vec2d_c Size() const;

	// Returns the half-side lengths.
	TRM_vec2d_c HalfSize() const;

	// Tests if this OBB2D is degenerate.
	bool IsDegenerate() const;

	// Returns the center point.
	TRM_vec2d_c CenterPoint() const;

	// Computes the closest point inside this OBB2D to the given point.
	TRM_vec2d_c ClosestPoint(const TRM_vec2d_c &point) const;

	// Computes the distance between this OBB2D and the given object.
	float Distance(const TRM_vec2d_c &point) const;

	// Tests if the given object is fully contained inside this OBB2d_c.
	bool Contains(const TRM_vec2d_c &point) const;
	bool Contains(const TRM_linesg2d_c &lineSegment) const;
	bool Contains(const TRM_AABB2d_c &aabb) const;
	bool Contains(const TRM_OBB2d_c &obb) const;
	bool Contains(const TRM_polygon2d_c &polygon) const;

	// Tests whether this TRM_OBB2d_c and the given object intersect.
	bool Intersects(const TRM_OBB2d_c &obb) const;
	bool Intersects(const TRM_AABB2d_c &aabb) const;
	bool Intersects(const TRM_line2d_c &line, float &near, float &far) const;
	bool Intersects(const TRM_line2d_c &line) const;
	bool Intersects(const TRM_linesg2d_c &lineSegment, float &near, float &far) const;
	bool Intersects(const TRM_linesg2d_c &lineSegment) const;
	bool Intersects(const TRM_polygon2d_c &polygon) const;

	// Expands this OBB2D to enclose the given object.
	void Enclose(const TRM_vec2d_c &point);

	bool Equal(const TRM_OBB2d_c &rhs) const;
};

GEO_END_NAMESPACE
