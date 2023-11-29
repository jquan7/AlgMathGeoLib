/*******************************************************************************
Copyright © Toramon Co., Ltd. 2017-2023. All Rights Reserved.
File name    : GJK2D.cpp
Author       : qijianquan qijq@toramon.com
Version      :
Date         : 2023-11-27 15:15:15
LastEditors  :
LastEditTime :
Description  : Implementation of the Gilbert-Johnson-Keerthi (GJK)
             :   convex polygon intersection test in 2D.
Others       :
Log          :
*******************************************************************************/
#include "GJK2D.h"
#include "../Geometry/LineSegment2D.h"
#include "../Geometry/Triangle2D.h"
#include "../Math/assume.h"
#include "../Math/MathConstants.h"
#include "../Math/MathFunc.h"

MATH_BEGIN_NAMESPACE

/// This function examines the 2D simplex defined by the array of points in s, and calculates which voronoi region
/// of that simplex the origin is closest to. Based on that information, the function constructs a new 2D simplex
/// that will be used to continue the search, and returns a new search direction for the GJK algorithm.
/** @param s [in, out] An array of points in the simplex. When this function returns, this point array is updated to contain the new 2D search simplex.
	@param n [in, out] The number of points in the array s. When this function returns, this reference is updated to specify how many
	                   points the new 2D search simplex contains.
	@return The new search direction vector. */
vec2d UpdateSimplex2D(vec2d *s, int &n)
{
	assert1(n == 2 || n == 3, n);

	if (n == 2) {
		// Four voronoi regions that the origin could be in:
		// 0) closest to vertex s[0].
		// 1) closest to vertex s[1].
		// 2) closest to line segment s[0]->s[1]. XX
		// 3) contained in the line segment s[0]->s[1], and our search is over and the algorithm is now finished. XX
		// By construction of the simplex, the cases 0) and 1) can never occur. Then only the cases marked with XX need to be checked.

		vec2d d01 = s[1] - s[0];
		vec2d newSearchDir = Perp2D(d01);
		if (newSearchDir.LengthSq() > 1e-7f) {
			return Dot(newSearchDir, s[0]) <= 0.f ? newSearchDir : -newSearchDir; // Case 2)
		}
		else {
			// Case 3)
			n = 0;
			return vec2d::zero;
		}
	} else {
		assert(n == 3);
		// Seven voronoi regions:
		// 0) closest to vertex s[0].
		// 1) closest to vertex s[1].
		// 2) closest to vertex s[2].
		// 3) closest to edge s[0]->s[1].
		// 4) closest to edge s[1]->s[2].  XX
		// 5) closest to edge s[0]->s[2].  XX
		// 6) contained in the triangle s[0]->s[1]->s[2], and our search is over and the algorithm is now finished.  XX

		// By construction of the simplex, the origin must always be in a voronoi region that includes the point s[2], since that
		// was the last added point. But it cannot be the case 2), since previous search took us deepest towards the direction s[1]->s[2],
		// and case 2) implies we should have been able to go even deeper in that direction, or that the origin is not included in the convex shape,
		// a case which has been checked for already before. Therefore the cases 0)-3) can never occur. Only the cases marked with XX need to be checked.

		vec2d d12 = s[2]-s[1];
		vec2d d02 = s[2]-s[0];

		vec2d e12 = Perp2D(d12);
		if (Dot(d02, e12) <= 0.f) e12 = -e12;
		float t12 = Dot(s[1], e12);
		if (t12 < 0.f) {
			// Case 4: Edge 1->2 is closest.
			vec2d newDir = e12;
			s[0] = s[1];
			s[1] = s[2];
			n = 2;
			return newDir;
		}

		vec2d e02 = Perp2D(d02);
		if (Dot(d12, e02) <= 0.f) e02 = -e02;
		float t02 = Dot(s[0], e02);
		if (t02 < 0.f) {
			// Case 5: Edge 0->2 is closest.
			vec2d newDir = e02;
			s[1] = s[2];
			n = 2;
			return newDir;
		}

		// Case 6) The origin lies directly inside the triangle. For robustness, terminate the search here immediately with success.
		n = 0;
		return vec2d::zero;
	}
}

MATH_END_NAMESPACE
