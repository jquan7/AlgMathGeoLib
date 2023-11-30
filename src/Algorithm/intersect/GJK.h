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

MATH_BEGIN_NAMESPACE

/**
 * @brief   Test if two objects are intersect.
 * @param   [const A &] A - object A.
 * @param   [const B &] B - object B.
 * @return  intersect or not.
 */
template<typename A, typename B>
bool GJKIntersect(const A &a, const B &b);

// This computes GJK intersection, but by first translating both objects to a coordinate frame that is as closely
// centered around world origin as possible, to gain floating point precision.
template<typename A, typename B>
bool FloatingPointOffsetedGJKIntersect(const A &a, const B &b);

MATH_END_NAMESPACE
