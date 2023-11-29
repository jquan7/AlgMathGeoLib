/*******************************************************************************
Copyright © Toramon Co., Ltd. 2017-2023. All Rights Reserved.
File name    : GJK2D.h
Author       : qijianquan qijq@toramon.com
Version      :
Date         : 2023-11-27 15:15:15
LastEditors  :
LastEditTime :
Description  : Gilbert-Johnson-Keerthi (GJK)
Others       :
Log          :
*******************************************************************************/
#pragma once

#include "../MathGeoLibFwd.h"
#include "../Math/float2.h"
#include "../Geometry/Polygon2DRef.h"

MATH_BEGIN_NAMESPACE

vec2d UpdateSimplex2D(vec2d *s, int &n);

#define SUPPORT2D(dir, minS, maxS) (a.ExtremePoint(dir, maxS) - b.ExtremePoint(-dir, minS));

template<typename A, typename B>
bool GJKIntersect2D(const A &a, const B &b)
{
	vec2d support[3];
	// Start with an arbitrary point in the Minkowski set shape.
	support[0] = a.AnyPointFast() - b.AnyPointFast();
	if (support[0].LengthSq() < 1e-7f) // Robustness check: Test if the first arbitrary point we guessed produced the zero vector we are looking for!
		return true;
	vec2d d = -support[0]; // First search direction is straight toward the origin from the found point.
	int n = 1; // Stores the current number of points in the search simplex.
	int nIterations = 50; // Robustness check: Limit the maximum number of iterations to perform to avoid infinite loop if types A or B are buggy!
	while(nIterations-- > 0)
	{
		// Compute the extreme point to the direction d in the Minkowski set shape.
		float maxS, minS;
		vec2d newSupport = SUPPORT2D(d, minS, maxS);
		// If the most extreme point in that search direction did not walk past the origin, then the origin cannot be contained in the Minkowski
		// convex shape, and the two convex objects a and b do not share a common point - no intersection!
		if (minS + maxS < 0.f)
			return false;
		// Add the newly evaluated point to the search simplex.
		assert(n < 3);
		support[n++] = newSupport;
		// Examine the current simplex, prune a redundant part of it, and produce the next search direction.
		d = UpdateSimplex2D(support, n);
		if (n == 0) // Was the origin contained in the current simplex? If so, then the convex shapes a and b do share a common point - intersection!
			return true;
	}
	assume(false && "GJK2D intersection test did not converge to a result!");
	// TODO: enable this on Polygon2DRef
//	assume2(false && "GJK2D intersection test did not converge to a result!", a.SerializeToString(), b.SerializeToString());
	return false; // Report no intersection.
}


inline bool GJKIntersect2D(const vec2d *a, int numA, const vec2d *b, int numB)
{
	Polygon2DRef p1 = { a, numA };
	Polygon2DRef p2 = { b, numB };
	return GJKIntersect2D(p1, p2);
}

// A helper adapter class to allow passing a vec2d into GJKIntersect2D() function.
struct GJKVec2DProxy
{
	vec2d v;
	GJKVec2DProxy(const vec2d &v):v(v) {}
	vec2d AnyPointFast() const { return v; }
	vec2d ExtremePoint(const vec2d &) const { return v; }
	vec2d ExtremePoint(const vec2d &direction, float &projectionDistance) const
	{
		vec2d extremePoint = ExtremePoint(direction);
		projectionDistance = extremePoint.Dot(direction);
		return extremePoint;
	}
};

inline bool GJKContainsPt2D(const vec2d *a, int numA, const vec2d &b)
{
	Polygon2DRef p1 = { a, numA };
	GJKVec2DProxy p2 = b;
	return GJKIntersect2D(p1, p2);
}

// A helper adapter class to allow passing a vec2d into GJKIntersect2D() function.
struct GJKOBB2DProxy
{
	vec2d center, axis0, axis1;
	GJKOBB2DProxy(const vec2d &center, const vec2d &axis0, const vec2d &axis1):center(center), axis0(axis0), axis1(axis1) {}
	vec2d AnyPointFast() const { return center; }
	vec2d ExtremePoint(const vec2d &direction) const
	{
		vec2d pt = center;
		pt += axis0 * (Dot(direction, axis0) >= 0.f ? 1.0f : -1.0f);
		pt += axis1 * (Dot(direction, axis1) >= 0.f ? 1.0f : -1.0f);
		return pt;
	}

	vec2d ExtremePoint(const vec2d &direction, float &projectionDistance) const
	{
		vec2d extremePoint = ExtremePoint(direction);
		projectionDistance = extremePoint.Dot(direction);
		return extremePoint;
	}
};

inline bool GJKIntersectOBB2D(const vec2d *a, int numA, const vec2d &center, const vec2d &axis0, const vec2d &axis1)
{
	Polygon2DRef p1 = { a, numA };
	GJKOBB2DProxy p2(center, axis0, axis1);
	return GJKIntersect2D(p1, p2);
}

MATH_END_NAMESPACE
