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

#include "../../MathGeoLibFwd.h"

MATH_BEGIN_NAMESPACE

/**
 * @brief   Test if two objects are intersect.
 * @param   [const A &] a - object A.
 * @param   [const B &] b - object B.
 * @return  intersect or not.
 */
template<typename A, typename B>
bool GJKIntersect2D(const A &a, const B &b);

/**
 * @brief   Test if two Polygons are intersect.
 * @param   [const A &] a - object A.
 * @param   [const B &] b - object B.
 * @return  intersect or not.
 */
bool GJKIntersect2D(const vec2d *a, int na, const vec2d *b, int nb);

bool GJKContainsPt2D(const vec2d *a, int na, const vec2d &b);

bool GJKIntersectOBB2D(const vec2d *a, int na, const vec2d &center,
    const vec2d &axis0, const vec2d &axis1);

MATH_END_NAMESPACE
