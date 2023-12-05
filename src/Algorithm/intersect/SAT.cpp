/*******************************************************************************
Copyright © Toramon Co., Ltd. 2017-2023. All Rights Reserved.
File name    : SAT.cpp
Author       : qijianquan qijq@toramon.com
Version      :
Date         : 2023-11-27 15:15:15
LastEditors  :
LastEditTime :
Description  : Separating Axis Theorem -based convex object intersection test.
Others       :
Log          :
*******************************************************************************/

#include "SAT.h"
#include "../../Math/float2.inl"

MATH_BEGIN_NAMESPACE

bool HasCcwWindingOrder(const float2 *poly, int num_vertices)
{
    int prev2 = num_vertices - 2;
    int prev1 = num_vertices - 1;
    for (int i = 0; i < num_vertices; ++i) {
        if (PerpDot2D(poly[prev2], poly[prev1], poly[i]) < -1e-2f)
            return false;
        prev2 = prev1;
        prev1 = i;
    }
    return true;
}

bool SATCollide2D(const float2 *a, int na, const float2 *b, int nb)
{
    if (na <= 0 || nb <= 0 || !a || !b ||
        !HasCcwWindingOrder(a, na) || !HasCcwWindingOrder(b, nb))
        return false;

    float2 edge;
    int prev = na - 1;
    for (int i = 0; i < na; ++i) {
        edge = a[i] - a[prev];

        // N.b. for improved numerical stability, could do
        /*  float minDistance = (b[0] - a[i]).PerpDot(edge);
             for(int j = 1; j < nb; ++j)
             minDistance = Min(minDistance, (b[j] - a[i]).PerpDot(edge));
             if (minDistance > 0)
             return false;
           but uncertain how much that will improve. */

        float max_this = a[i].PerpDot(edge);
        float min_other = b[0].PerpDot(edge);
        for(int j = 1; j < nb; ++j)
            min_other = Min(min_other, b[j].PerpDot(edge));
        if (min_other > max_this)
            return false;
        prev = i;
    }

    prev = nb - 1;
    for (int i = 0; i < nb; ++i) {
        edge = b[i] - b[prev];
        float max_this = b[i].PerpDot(edge);
        float min_other = a[0].PerpDot(edge);
        for(int j = 1; j < na; ++j)
            min_other = Min(min_other, a[j].PerpDot(edge));
        if (min_other > max_this)
            return false;
        prev = i;
    }

    return true;
}

float SATCollide2dCollisionPoint(const float2 *a, int na, const float2 *b, int nb,
    float2 &collision_point, float2 &collision_normal)
{
    if (na <= 0 || nb <= 0 || !a || !b ||
        !HasCcwWindingOrder(a, na) || !HasCcwWindingOrder(b, nb))
        return false;

    float2 edge;
    float min_penetration_distance = -FLOAT_INF;
    int prev = na - 1;
    for (int i = 0; i < na; ++i) {
        edge = (a[i] - a[prev]).Normalized();
        float max_this = a[i].PerpDot(edge);
        float min_other = b[0].PerpDot(edge);
        int index = 0;
        for (int j = 1; j < nb; ++j) {
            float penetration_distance = b[j].PerpDot(edge);
            if (penetration_distance < min_other) {
                min_other = penetration_distance;
                index = j;
            }
        }

        float dist = min_other - max_this;
        if (dist > 0.f)
            return dist;
        if (dist > min_penetration_distance) {
            min_penetration_distance = dist;
            collision_normal = edge.Perp();
            collision_point = b[index];
        }

        prev = i;
    }

    prev = nb - 1;
    for (int i = 0; i < nb; ++i) {
        edge = (b[i] - b[prev]).Normalized();

        float max_this = b[i].PerpDot(edge);
        float min_other = a[0].PerpDot(edge);
        int index = 0;
        for (int j = 1; j < na; ++j) {
            float penetration_distance = a[j].PerpDot(edge);
            if (penetration_distance < min_other) {
                min_other = penetration_distance;
                index = j;
            }
        }

        float dist = min_other - max_this;
        if (dist > 0.f)
            return dist;
        if (dist > min_penetration_distance) {
            min_penetration_distance = dist;
            collision_normal = -edge.Perp();
            collision_point = a[index];
        }

        prev = i;
    }

    return min_penetration_distance;
}

MATH_END_NAMESPACE
