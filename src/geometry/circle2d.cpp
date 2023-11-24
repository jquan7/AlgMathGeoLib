/*******************************************************************************
Copyright © Toramon Co., Ltd. 2017-2023. All Rights Reserved.
File name    : circle2d.cpp
Author       : qijianquan qijq@toramon.com
Version      :
Date         : 2023-11-21 15:15:15
LastEditors  :
LastEditTime :
Description  : circle 2d
Others       :
Log          :
*******************************************************************************/

#include "circle2d.h"
#include "AABB2d.h"
#include "line_segmt2d.h"
#include "line2d.h"
#include "OBB2d.h"
#include "polygon2d.h"
#include "rect2d.h"

GEO_BEGIN_NAMESPACE

using namespace trm::math;

// Tests if this TRM_circle2d_c is degenerate.
bool TRM_circle2d_c::IsDegenerate() const {
    return EqZero(r);
}

// Tests if the given object is fully contained inside this circle.
bool TRM_circle2d_c::Contains(const TRM_float2d_c &point) const {
    return pos.DistanceSq(point) <= r * r;
}

// Computes the distance between this circle and the given object.
float TRM_circle2d_c::Distance(const TRM_float2d_c &point) const {
    std::max(0.f, SignedDistance(point));
}

float TRM_circle2d_c::SignedDistance(const TRM_float2d_c &point) const {
    return pos.Distance(point) - r;
}

GEO_END_NAMESPACE
