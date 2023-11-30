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
#include "../../Geometry/LineSegment2D.h"
#include "../../Geometry/Triangle2D.h"
#include "../../Math/assume.h"
#include "../../Math/MathConstants.h"
#include "../../Math/MathFunc.h"
#include "../../Math/float2.h"
#include "../../Geometry/Polygon2DRef.h"

MATH_BEGIN_NAMESPACE

#define SUPPORT2D(dir, minS, maxS) (a.ExtremePoint(dir, maxS) - b.ExtremePoint(-dir, minS));

/**
 * @brief   Calculates which voronoi region of the simplex the origin is closest to.
 * @param   [vec *] s - An array of points in the simplex.
 * @param   [int &] n - The number of points.
 * @return  The new search direction vector.
 */
vec2d UpdateSimplex2D(vec2d *s, int &n)
{
    if (2 == n) {
        // Four voronoi regions that the origin could be in:
        // 0) closest to vertex s[0].
        // 1) closest to vertex s[1].
        // 2) closest to line segment s[0]->s[1]. XX
        // 3) contained in the line segment s[0]->s[1], and our search is over and the algorithm is now finished. XX
        // By construction of the simplex, the cases 0) and 1) can never occur. Then only the cases marked with XX need to be checked.

        vec2d d01 = s[1] - s[0];
        vec2d new_search_dir = Perp2D(d01);
        if (new_search_dir.LengthSq() > 1e-7f) {
            return Dot(new_search_dir, s[0]) <= 0.f ? new_search_dir : -new_search_dir; // Case 2)
        }
    } else  if (3 == n) {
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
            vec2d new_dir = e12;
            s[0] = s[1];
            s[1] = s[2];
            n = 2;
            return new_dir;
        }

        vec2d e02 = Perp2D(d02);
        if (Dot(d12, e02) <= 0.f) e02 = -e02;
        float t02 = Dot(s[0], e02);
        if (t02 < 0.f) {
            // Case 5: Edge 0->2 is closest.
            vec2d new_dir = e02;
            s[1] = s[2];
            n = 2;
            return new_dir;
        }
    }

    // n:2 Case 3)
    // n:3 Case 6) The origin lies directly inside the triangle.
    //     For robustness, terminate the search here immediately with success.
    // n:other
    n = 0;
    return vec2d::zero;
}

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
    int n_iters = 50; // Robustness check: Limit the maximum number of iterations to perform to avoid infinite loop if types A or B are buggy!
    while (n_iters-- > 0) {
        // Compute the extreme point to the direction d in the Minkowski set shape.
        float max_sup, min_sup;
        vec2d new_sup = SUPPORT2D(d, min_sup, max_sup);
        // If the most extreme point in that search direction did not walk past the origin, then the origin cannot be contained in the Minkowski
        // convex shape, and the two convex objects a and b do not share a common point - no intersection!
        if (min_sup + max_sup < 0.f)
            return false;
        // Add the newly evaluated point to the search simplex.
        if (n >= 3)
            return true;
        support[n++] = new_sup;
        // Examine the current simplex, prune a redundant part of it, and produce the next search direction.
        d = UpdateSimplex2D(support, n);
        if (n == 0) // Was the origin contained in the current simplex? If so, then the convex shapes a and b do share a common point - intersection!
            return true;
    }
    // TODO: enable this on Polygon2DRef
    // assume2(false && "GJK2D intersection test did not converge to a result!", a.SerializeToString(), b.SerializeToString());
    return false; // Report no intersection.
}

bool GJKIntersect2D(const vec2d *a, int na, const vec2d *b, int nb)
{
    Polygon2DRef p1 = {a, na};
    Polygon2DRef p2 = {b, nb};
    return GJKIntersect2D(p1, p2);
}

// A helper adapter class to allow passing a vec2d into GJKIntersect2D() function.
struct GJKVec2DProxy
{
    vec2d v;
    GJKVec2DProxy(const vec2d &v):v(v) {}
    vec2d AnyPointFast() const { return v; }
    vec2d ExtremePoint(const vec2d &) const { return v; }
    vec2d ExtremePoint(const vec2d &direction, float &project_dist) const {
        vec2d extreme = ExtremePoint(direction);
        project_dist = extreme.Dot(direction);
        return extreme;
    }
};

// A helper adapter class to allow passing a vec2d into GJKIntersect2D() function.
struct GJKOBB2DProxy
{
    vec2d center, axis0, axis1;
    GJKOBB2DProxy(const vec2d &center, const vec2d &axis0, const vec2d &axis1)
        :center(center), axis0(axis0), axis1(axis1) {}
    vec2d AnyPointFast() const { return center; }
    vec2d ExtremePoint(const vec2d &direction) const {
        vec2d pt = center;
        pt += axis0 * (Dot(direction, axis0) >= 0.f ? 1.0f : -1.0f);
        pt += axis1 * (Dot(direction, axis1) >= 0.f ? 1.0f : -1.0f);
        return pt;
    }

    vec2d ExtremePoint(const vec2d &direction, float &project_dist) const {
        vec2d extreme = ExtremePoint(direction);
        project_dist = extreme.Dot(direction);
        return extreme;
    }
};

bool GJKContainsPt2D(const vec2d *a, int na, const vec2d &b)
{
    Polygon2DRef p1 = {a, na};
    GJKVec2DProxy p2 = b;
    return GJKIntersect2D(p1, p2);
}

bool GJKIntersectOBB2D(const vec2d *a, int na, const vec2d &center,
    const vec2d &axis0, const vec2d &axis1)
{
    Polygon2DRef p1 = {a, na};
    GJKOBB2DProxy p2(center, axis0, axis1);
    return GJKIntersect2D(p1, p2);
}

MATH_END_NAMESPACE
