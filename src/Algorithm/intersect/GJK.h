/*******************************************************************************
Copyright © Toramon Co., Ltd. 2017-2023. All Rights Reserved.
File name    : GJK.h
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

#include "../../MathGeoLibFwd.h"
#include "../../Math/float3.h"
#include "../../Geometry/AABB.h"

MATH_BEGIN_NAMESPACE

vec UpdateSimplex(vec *s, int &n);

// 计算多边形在给定方向上的最远点
#define SUPPORT(dir, minS, maxS) (a.ExtremePoint(dir, maxS) - b.ExtremePoint(-dir, minS));

/**
 * @brief   Test if two objects are intersect.
 * @param   [const A &] A - object A.
 * @param   [const B &] B - object B.
 * @return  intersect or not.
 */
template<typename A, typename B>
bool GJKIntersect(const A &a, const B &b)
{
    vec support[4];
    // Start with an arbitrary point in the Minkowski set shape.
    support[0] = a.AnyPointFast() - b.AnyPointFast();
    if (support[0].LengthSq() < 1e-7f) // Robustness check: Test if the first arbitrary point we guessed produced the zero vector we are looking for!
        return true;
    vec d = -support[0]; // First search direction is straight toward the origin from the found point.
    int n = 1; // Stores the current number of points in the search simplex.
    int n_iters = 50; // Robustness check: Limit the maximum number of iterations to perform to avoid infinite loop if types A or B are buggy!
    while (n_iters-- > 0) {
        // Compute the extreme point to the direction d in the Minkowski set shape.
        float max_sup, min_sup;
        vec new_sup = SUPPORT(d, min_sup, max_sup);
        // If the most extreme point in that search direction did not walk past the origin, then the origin cannot be contained in the Minkowski
        // convex shape, and the two convex objects a and b do not share a common point - no intersection!
        if (min_sup + max_sup < 0.f)
            return false;
        // Add the newly evaluated point to the search simplex.
        if (n >= 4)
            return true;
        support[n++] = new_sup;
        // Examine the current simplex, prune a redundant part of it, and produce the next search direction.
        d = UpdateSimplex(support, n);
        if (n == 0) // Was the origin contained in the current simplex? If so, then the convex shapes a and b do share a common point - intersection!
            return true;
    }
    // TODO: enable
    // assume2(false && "GJK intersection test did not converge to a result!", a.SerializeToString(), b.SerializeToString());
    return false; // Report no intersection.
}

// This computes GJK intersection, but by first translating both objects to a coordinate frame that is as closely
// centered around world origin as possible, to gain floating point precision.
template<typename A, typename B>
bool FloatingPointOffsetedGJKIntersect(const A &a, const B &b)
{
    AABB ab = a.MinimalEnclosingAABB();
    AABB bb = b.MinimalEnclosingAABB();
    vec offset = (Min(ab.minpt, bb.minpt) + Max(ab.maxpt, bb.maxpt)) * 0.5f;
    const vec fp_precision_offset = -offset;
    return GJKIntersect(a.Translated(fp_precision_offset), b.Translated(fp_precision_offset));
}

MATH_END_NAMESPACE
