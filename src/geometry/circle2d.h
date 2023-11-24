/*******************************************************************************
Copyright © Toramon Co., Ltd. 2017-2023. All Rights Reserved.
File name    : circle2d.h
Author       : qijianquan qijq@toramon.com
Version      :
Date         : 2023-11-21 15:15:15
LastEditors  :
LastEditTime :
Description  : circle 2d
Others       :
Log          :
*******************************************************************************/

#pragma once

#include "geometry_fwd.h"

GEO_BEGIN_NAMESPACE

class TRM_circle2d_c {
public:
    // Center position.
    TRM_float2d_c pos;

    // Radius
    float r;

    // Constructor
    TRM_circle2d_c() {}  // pos and r are all undefined
    TRM_circle2d_c(const TRM_float2d_c &center, float radius) {
        pos=center, r=std::abs(radius);
    }  // radius > 0

    // Tests if this TRM_circle2d_c is degenerate.
    bool IsDegenerate() const;  // radius > 0

    // Tests if the given object is fully contained inside this circle.
    bool Contains(const TRM_float2d_c &point) const;

    // Computes the distance between this circle and the given object.
    float Distance(const TRM_float2d_c &point) const;
    float SignedDistance(const TRM_float2d_c &point) const;
};

GEO_END_NAMESPACE
