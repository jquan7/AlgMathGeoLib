/*******************************************************************************
Copyright © Toramon Co., Ltd. 2017-2023. All Rights Reserved.
File name    : OBB2D.cpp
Author       : qijianquan qijq@toramon.com
Version      :
Date         : 2023-11-21 15:15:15
LastEditors  :
LastEditTime :
Description  : 2D Oriented Bounding Box
Others       :
Log          :
*******************************************************************************/

#include "OBB2d.h"
#include "AABB2d.h"
#include "circle2d.h"
#include "line_segmt2d.h"
#include "line2d.h"
#include "polygon2d.h"
#include "rect2d.h"

GEO_BEGIN_NAMESPACE

// Constructor
TRM_OBB2d_c::TRM_OBB2d_c(const TRM_vec2d_c &pos_, const TRM_vec2d_c &r_,
    const TRM_vec2d_c &axis0, const TRM_vec2d_c &axis1)
    :pos(pos_), r(r_) {
    axis[0] = axis0; axis[1] = axis1;
}
TRM_OBB2d_c::TRM_OBB2d_c(const TRM_AABB2d_c &aabb) {
    SetFrom(aabb);
}

// Sets this OBB2D from an AABB2d.
void TRM_OBB2d_c::SetFrom(const TRM_AABB2d_c &aabb) {
    pos = aabb.CenterPoint();
    r = aabb.HalfSize();
    axis[0] = TRM_vec2d_c(1, 0);
    axis[1] = TRM_vec2d_c(0, 1);
}
// void SetFrom(const TRM_AABB2d_c &aabb, float angle);

// Returns the tightest AABB2d that contains this OBB2D.
TRM_AABB2d_c TRM_OBB2d_c::MinimalEnclosingAABB() const {

}

// Returns the side lengths.
TRM_vec2d_c TRM_OBB2d_c::Size() const {

}

// Returns the half-side lengths.
TRM_vec2d_c TRM_OBB2d_c::HalfSize() const;

// Tests if this OBB2D is degenerate.
bool TRM_OBB2d_c::IsDegenerate() const;

// Returns the center point.
TRM_vec2d_c TRM_OBB2d_c::CenterPoint() const;

// Computes the closest point inside this OBB2D to the given point.
TRM_vec2d_c TRM_OBB2d_c::ClosestPoint(const TRM_vec2d_c &point) const;

// Computes the distance between this OBB2D and the given object.
float TRM_OBB2d_c::Distance(const TRM_vec2d_c &point) const;

// Tests if the given object is fully contained inside this OBB2d_c.
bool TRM_OBB2d_c::Contains(const TRM_vec2d_c &point) const;
bool TRM_OBB2d_c::Contains(const TRM_linesg2d_c &lineSegment) const;
bool TRM_OBB2d_c::Contains(const TRM_AABB2d_c &aabb) const;
bool TRM_OBB2d_c::Contains(const TRM_OBB2d_c &obb) const;
bool TRM_OBB2d_c::Contains(const TRM_polygon2d_c &polygon) const;

// Tests whether this TRM_OBB2d_c and the given object intersect.
bool TRM_OBB2d_c::Intersects(const TRM_OBB2d_c &obb) const;
bool TRM_OBB2d_c::Intersects(const TRM_AABB2d_c &aabb) const;
bool TRM_OBB2d_c::Intersects(const TRM_line2d_c &line, float &near, float &far) const;
bool TRM_OBB2d_c::Intersects(const TRM_line2d_c &line) const;
bool TRM_OBB2d_c::Intersects(const TRM_linesg2d_c &lineSegment, float &near, float &far) const;
bool TRM_OBB2d_c::Intersects(const TRM_linesg2d_c &lineSegment) const;
bool TRM_OBB2d_c::Intersects(const TRM_polygon2d_c &polygon) const;

// Expands this OBB2D to enclose the given object.
void TRM_OBB2d_c::Enclose(const TRM_vec2d_c &point);

bool TRM_OBB2d_c::Equal(const TRM_OBB2d_c &rhs) const {
    return pos.Equal(rhs.pos) && r.Equal(rhs.r) && axis[0].Equal(rhs.axis[0]) && axis[1].Equal(rhs.axis[1]);
}

GEO_END_NAMESPACE
