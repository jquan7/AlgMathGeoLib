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

#define ENABLE_NAMESPACE

#ifdef ENABLE_NAMESPACE

#define ALG_BEGIN_NAMESPACE namespace trm { namespace alg {
#define ALG_END_NAMESPACE }}

#define GEO_BEGIN_NAMESPACE namespace trm { namespace geo {
#define GEO_END_NAMESPACE }}

#define MATH_BEGIN_NAMESPACE namespace trm { namespace math {
#define MATH_END_NAMESPACE }}

#else

#define ALG_BEGIN_NAMESPACE
#define ALG_END_NAMESPACE

#define GEO_BEGIN_NAMESPACE
#define GEO_END_NAMESPACE

#define MATH_BEGIN_NAMESPACE
#define MATH_END_NAMESPACE

#endif
