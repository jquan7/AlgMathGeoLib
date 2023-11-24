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

#include "../namespaces.h"

#ifdef __GNUC__
#define NOT_NECESSARILY_USED __attribute__ ((unused))
#else
#define NOT_NECESSARILY_USED
#endif

MATH_BEGIN_NAMESPACE

// Math Define
static const float NOT_NECESSARILY_USED Q5 = 32;
static const float NOT_NECESSARILY_USED Q15 = 32768;
static const float NOT_NECESSARILY_USED IQ5 = 0.03125;
static const float NOT_NECESSARILY_USED IQ15 = 0.00003051758;

static const float NOT_NECESSARILY_USED INF = 1e9;
static const float NOT_NECESSARILY_USED EPS = 1e-9;
static const float NOT_NECESSARILY_USED CEPS = 1e-6;  // 浮点比较
static const float NOT_NECESSARILY_USED PI = 3.14159265358;
static const float NOT_NECESSARILY_USED PI_D180 = PI / 180.f;

// QAP LOG
static const int NOT_NECESSARILY_USED QAP_LOG_LEN_MAX = 10240;

MATH_END_NAMESPACE