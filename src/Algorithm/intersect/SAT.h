/*******************************************************************************
Copyright © Toramon Co., Ltd. 2017-2023. All Rights Reserved.
File name    : SAT.h
Author       : qijianquan qijq@toramon.com
Version      :
Date         : 2023-11-27 15:15:15
LastEditors  :
LastEditTime :
Description  : Separating Axis Theorem -based convex object intersection test.
Others       :
Log          :
*******************************************************************************/
#pragma once

#include "../../MathGeoLibFwd.h"
#include "../../Math/float2.h"
#include "../../Math/float3.h"

MATH_BEGIN_NAMESPACE

bool SATCollide2D(const float2 *a, int na, const float2 *b, int nb);

// Returns the penetration distance between the two objects. If > 0, the objects are not colliding.
// If < 0, then the objects penetrate that deep along outCollisionNormal vector.
float SATCollide2dCollisionPoint(const float2 *a, int na, const float2 *b, int nb,
    float2 &collision_point, float2 &collision_normal);

/**
 * @brief   Test if two objects are intersect.
 * @param   [const A &] A - object A.
 * @param   [const B &] B - object B.
 * @return  intersect or not.
 */
template<typename A, typename B>
bool SATIntersect(const A &a, const B &b);

MATH_END_NAMESPACE
