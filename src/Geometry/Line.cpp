/*******************************************************************************
Copyright © Toramon Co., Ltd. 2017-2023. All Rights Reserved.
File name    : Line.h
Author       : qijianquan qijq@toramon.com
Version      :
Date         : 2023-11-27 15:15:15
LastEditors  :
LastEditTime :
Description  : Line in 3D space
Others       :
Log          :
*******************************************************************************/
#include "Line.h"
#include "LineSegment.h"
#include "OBB.h"
#include "Triangle.h"
#include "Plane.h"
#include "Polygon.h"
#include "AABB.h"
#include "Circle.h"
#include "../Math/float3x3.h"
#include "../Math/float3x4.h"
#include "../Math/float4x4.h"
#include "../Math/Quat.h"
#include "../Math/MathFunc.h"

#ifdef MATH_ENABLE_STL_SUPPORT
#include <iostream>
#endif

MATH_BEGIN_NAMESPACE

Line::Line(const vec &pos_, const vec &dir_)
:pos(pos_), dir(dir_)
{
    if (!dir.IsNormalized()) {
        printf("The direction need to be Normalized!\n");
        dir.Normalized();
    }
}

Line::Line(const LineSegment &lineseg)
:pos(lineseg.a), dir(lineseg.Dir())
{
}

bool Line::IsFinite() const
{
    return pos.IsFinite() && dir.IsFinite();
}

vec Line::GetPoint(float d) const
{
    return pos + d * dir;
}

void Line::Translate(const vec &offset)
{
    pos += offset;
}

void Line::Transform(const float3x3 &transform)
{
    pos = transform.Transform(pos);
    dir = transform.Transform(dir);
}

void Line::Transform(const float3x4 &transform)
{
    pos = transform.MulPos(pos);
    dir = transform.MulDir(dir);
}

void Line::Transform(const float4x4 &transform)
{
    pos = transform.MulPos(pos);
    dir = transform.MulDir(dir);
}

void Line::Transform(const Quat &transform)
{
    pos = transform.Transform(pos);
    dir = transform.Transform(dir);
}

bool Line::Contains(const vec &point, float dist_th) const
{
    return ClosestPoint(point).DistanceSq(point) <= dist_th;
}

bool Line::Contains(const LineSegment &lineseg, float epsilon) const
{
    return Contains(lineseg.a, epsilon) && Contains(lineseg.b, epsilon);
}

bool Line::Equals(const Line &line, float epsilon) const
{
    if (!dir.IsNormalized() || !line.dir.IsNormalized())
        return false;  // TODO
    return Contains(line.pos, epsilon) && EqualAbs(Abs(dir.Dot(line.dir)), 1.f, epsilon);
}

float Line::Distance(const vec &point, float &d) const
{
    return ClosestPoint(point, d).Distance(point);
}

float Line::Distance(const Line &other, float &d, float &d2) const
{
    vec c = ClosestPoint(other, d, d2);
    return c.Distance(other.GetPoint(d2));
}

float Line::Distance(const LineSegment &other, float &d, float &d2) const
{
    vec c = ClosestPoint(other, d, d2);
    mathassert(d2 >= 0.f);
    mathassert(d2 <= 1.f);
    return c.Distance(other.GetPoint(d2));
}

vec Line::ClosestPoint(const vec &tar_pt, float &d) const
{
    d = Dot(tar_pt - pos, dir);
    return GetPoint(d);
}

vec Line::ClosestPoint(const Line &other, float &d, float &d2) const
{
    ClosestPointLineLine(pos, dir, other.pos, other.dir, d, d2);
    return GetPoint(d);
}

vec Line::ClosestPoint(const LineSegment &other, float &d, float &d2) const
{
    ClosestPointLineLine(pos, dir, other.a, other.b - other.a, d, d2);
    if (d2 < 0.f)
    {
        d2 = 0.f;
        return ClosestPoint(other.a, d);
    }
    else if (d2 > 1.f)
    {
        d2 = 1.f;
        return ClosestPoint(other.b, d);
    }
    else
        return GetPoint(d);
}

vec Line::ClosestPoint(const Triangle &triangle, float &d) const
{
    vec closestPointTriangle = triangle.ClosestPoint(*this);
    return ClosestPoint(closestPointTriangle, d);
}

vec Line::ClosestPoint(const Triangle &triangle, float &d, float2 &barycentric_coord_UV) const
{
    vec closestPointTriangle = triangle.ClosestPoint(*this);
    barycentric_coord_UV = triangle.BarycentricUV(closestPointTriangle);
    return ClosestPoint(closestPointTriangle, d);
}


bool Line::Intersects(const Triangle &triangle, float *d, vec *intersect_pt) const
{
    return triangle.Intersects(*this, d, intersect_pt);
}

bool Line::Intersects(const Plane &plane, float *d) const
{
    return plane.Intersects(*this, d);
}

bool Line::Intersects(const AABB &aabb) const
{
    return aabb.Intersects(*this);
}

bool Line::Intersects(const AABB &aabb, float &near, float &far) const
{
    return aabb.Intersects(*this, near, far);
}

bool Line::Intersects(const OBB &obb) const
{
    return obb.Intersects(*this);
}

bool Line::Intersects(const OBB &obb, float &near, float &far) const
{
    return obb.Intersects(*this, near, far);
}

bool Line::Intersects(const Polygon &polygon) const
{
    return polygon.Intersects(*this);
}

bool Line::Intersects(const Circle &disc) const
{
    return disc.Intersects(*this);
}

LineSegment Line::ToLineSegment(float d) const
{
    return LineSegment(pos, GetPoint(d));
}

LineSegment Line::ToLineSegment(float start, float end) const
{
    return LineSegment(GetPoint(start), GetPoint(end));
}

void Line::ProjectToAxis(const vec &direction, float &outmin, float &outmax) const
{
    // Most of the time, the projection of a line spans the whole 1D axis.
    // As a special case, if the line is perpendicular to the direction vector in question,
    // then the projection interval of this line is a single point.
    if (dir.IsPerpendicular(direction))
        outmin = outmax = Dot(direction, pos);
    else
    {
        outmin = -FLOAT_INF;
        outmax = FLOAT_INF;
    }
}

bool Line::AreCollinear(const vec &p1, const vec &p2, const vec &p3, float epsilon)
{
    return vec::AreCollinear(p1, p2, p3, epsilon);
}

void Line::ClosestPointLineLine(const vec &v0, const vec &v10, const vec &v2, const vec &v32, float &d, float &d2)
{
    d = d2 = 0;
    if (v10.IsZero() || v32.IsZero())
        return;
    vec v02 = v0 - v2;
    float d0232 = v02.Dot(v32);
    float d3210 = v32.Dot(v10);
    float d3232 = v32.Dot(v32);
    float d0210 = v02.Dot(v10);
    float d1010 = v10.Dot(v10);
    float denom = d1010*d3232 - d3210*d3210;
    if (denom != 0.f)
        d = (d0232*d3210 - d0210*d3232) / denom;
    else
        d = 0.f;
    d2 = (d0232 + d * d3210) / d3232;
}

Line operator *(const float3x3 &transform, const Line &l)
{
    return Line(transform * l.pos, transform * l.dir);
}

Line operator *(const float3x4 &transform, const Line &l)
{
    return Line(transform.MulPos(l.pos), transform.MulDir(l.dir));
}

Line operator *(const float4x4 &transform, const Line &l)
{
    return Line(transform.MulPos(l.pos), transform.MulDir(l.dir));
}

Line operator *(const Quat &transform, const Line &l)
{
    return Line(transform * l.pos, transform * l.dir);
}

#if defined(MATH_ENABLE_STL_SUPPORT)
std::string Line::ToString() const
{
    char str[256];
    sprintf(str, "Line(Pos:(%.2f, %.2f, %.2f) Dir:(%.3f, %.3f, %.3f))", pos.x, pos.y, pos.z, dir.x, dir.y, dir.z);
    return str;
}

std::string Line::SerializeToString() const
{
    char str[256];
    char *s = SerializeFloat(pos.x, str); *s = ','; ++s;
    s = SerializeFloat(pos.y, s); *s = ','; ++s;
    s = SerializeFloat(pos.z, s); *s = ','; ++s;
    s = SerializeFloat(dir.x, s); *s = ','; ++s;
    s = SerializeFloat(dir.y, s); *s = ','; ++s;
    s = SerializeFloat(dir.z, s);
    assert(s+1 - str < 256);
    MARK_UNUSED(s);
    return str;
}

std::string Line::SerializeToCodeString() const
{
    return "Line(" + pos.SerializeToCodeString() + "," + dir.SerializeToCodeString() + ")";
}
#endif

#if defined(MATH_ENABLE_STL_SUPPORT)

std::ostream &operator <<(std::ostream &o, const Line &line)
{
    o << line.ToString();
    return o;
}

#endif

Line Line::FromString(const char *str, const char **outEndStr)
{
    if (!str)
        return Line(vec::nan, vec::nan);
    Line l;
    MATH_SKIP_WORD(str, "Line(");
    MATH_SKIP_WORD(str, "Pos:(");
    l.pos = PointVecFromString(str, &str);
    MATH_SKIP_WORD(str, " Dir:(");
    l.dir = DirVecFromString(str, &str);
    if (outEndStr)
        *outEndStr = str;
    return l;
}

MATH_END_NAMESPACE
