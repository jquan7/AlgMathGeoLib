/*******************************************************************************
Copyright © Toramon Co., Ltd. 2017-2023. All Rights Reserved.
File name    : func.h
Author       : qijianquan qijq@toramon.com
Version      :
Date         : 2023-11-21 15:15:15
LastEditors  :
LastEditTime :
Description  : 常用函数
Others       :
Log          :
*******************************************************************************/

#pragma once

#include <cmath>
#include <algorithm>

#include "../namespaces.h"
#include "constants.h"

MATH_BEGIN_NAMESPACE

// MACRO TODO
// #define sind(a) (sin((a) * PI_D180))  // a / 180 * pi
// #define cosd(a) (cos((a) * PI_D180))  // a / 180 * pi
// #define tand(a) (tan((a) * PI_D180))  // a / 180 * pi

/**
 * @brief   角度转弧度
 * @param   [const float] degrees - 角度值
 * @return  float - 弧度值
 */
static inline float DegToRad(const float degrees) { return degrees * PI_D180; }

/**
 * @brief   弧度转角度
 * @param   [const float] radians - 弧度值
 * @return  float - 角度值
 */
static inline float RadToDeg(float radians) { return radians / PI_D180; }

/**
 * @brief   Clamps the given input value to the range [min, max].
 * @param   [const T] val - input value
 * @param   [T] floor - 下限
 * @param   [T] ceil - 上限
 * @return  T - 结果
 */
template<typename T>
static inline T Clamp(const T val, T floor, T ceil)
{
    if (floor > ceil)
        std::swap(floor, ceil);
    return val <= ceil ? (val >= floor ? val : floor) : ceil;
}

template<typename T>
static inline T Clamp01(const T val) { return Clamp(val, T(0), T(1)); }

/**
 * @brief   浮点数判等
 * @param   [const float &] a - input value
 * @param   [const float &] b - input value
 * @return  bool - 是否相等
 */
bool inline Equal(const float &a, const float &b) { return std::abs(a-b) <= CEPS; }
bool inline Equal(const double &a, const double &b) { return std::abs(a-b) <= EPS; }
bool inline EqZero(const float &a) { return std::abs(a) <= CEPS; }
bool inline EqZero(const double &a) { return std::abs(a) <= EPS; }
bool inline NeZero(const float &a) { return std::abs(a) > CEPS; }
bool inline NeZero(const double &a) { return std::abs(a) > EPS; }

/**
 * @brief   快选
 * @param   [T*] input - 输入数组
 * @param   [int] p - 起始索引
 * @param   [int] r - 终止索引
 * @param   [int] k - 排序后的索引
 * @return  T - 排序后的第k位值
 */
template<typename T>
T QuickSelect(T* input, int p, int r, int k);

/**
 * @brief   快选
 * @param   [T*] input - 输入数组
 * @param   [int] N - 数组长度
 * @return  T - 数组中值
 */
template<typename T>
T Median(T* input, int N);


MATH_END_NAMESPACE
