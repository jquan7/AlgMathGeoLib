/* Copyright Jukka Jyl�nki

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License. */

/** @file LineSegment.cpp
	@author Jukka Jyl�nki
	@brief Implementation for the LineSegment geometry object. */
#include "LineSegment.h"
#include "../Math/MathFunc.h"
#include "AABB.h"
#include "Line.h"
#include "Plane.h"
#include "Polygon.h"
#include "../Math/float3x3.h"
#include "../Math/float3x4.h"
#include "../Math/float4x4.h"
#include "../Math/float4d.h"
#include "OBB.h"
#include "../Math/Quat.h"
#include "Triangle.h"
#include "Circle.h"

#ifdef MATH_ENABLE_STL_SUPPORT
#include <iostream>
#endif

MATH_BEGIN_NAMESPACE

LineSegment::LineSegment(const vec &a_, const vec &b_)
:a(a_), b(b_)
{
}

LineSegment::LineSegment(const Line &line, float d)
:a(line.pos), b(line.GetPoint(d))
{
}

vec LineSegment::GetPoint(float d) const
{
	return (1.f - d) * a + d * b;
}

vec LineSegment::CenterPoint() const
{
	return (a + b) * 0.5f;
}

void LineSegment::Reverse()
{
	std::swap(a, b);
}

vec LineSegment::Dir() const
{
	return (b - a).Normalized();
}

vec LineSegment::ExtremePoint(const vec &direction) const
{
	return Dot(direction, b-a) >= 0.f ? b : a;
}

vec LineSegment::ExtremePoint(const vec &direction, float &projectionDistance) const
{
	vec extremePoint = ExtremePoint(direction);
	projectionDistance = extremePoint.Dot(direction);
	return extremePoint;
}

void LineSegment::Translate(const vec &offset)
{
	a += offset;
	b += offset;
}

void LineSegment::Transform(const float3x3 &transform)
{
	a = transform * a;
	b = transform * b;
}

void LineSegment::Transform(const float3x4 &transform)
{
	a = transform.MulPos(a);
	b = transform.MulPos(b);
}

void LineSegment::Transform(const float4x4 &transform)
{
	a = transform.MulPos(a);
	b = transform.MulPos(b);
}

void LineSegment::Transform(const Quat &transform)
{
	a = transform * a;
	b = transform * b;
}

float LineSegment::Length() const
{
	return a.Distance(b);
}

float LineSegment::LengthSq() const
{
	return a.DistanceSq(b);
}

bool LineSegment::IsFinite() const
{
	return a.IsFinite() && b.IsFinite();
}

bool LineSegment::Contains(const vec &point, float dist_th) const
{
	return ClosestPoint(point).DistanceSq(point) <= dist_th;
}

bool LineSegment::Contains(const LineSegment &rhs, float dist_th) const
{
	return Contains(rhs.a, dist_th) && Contains(rhs.b, dist_th);
}

bool LineSegment::Equals(const LineSegment &rhs, float e) const
{
	return (a.Equals(rhs.a, e) && b.Equals(rhs.b, e)) || (a.Equals(rhs.b, e) && b.Equals(rhs.a, e));
}

vec LineSegment::ClosestPointD(const vec &point, double &d) const
{
	float4d dir = POINT_TO_FLOAT4D(b) - POINT_TO_FLOAT4D(a);
	d = Clamp01(dir.Dot(POINT_TO_FLOAT4D(point) - POINT_TO_FLOAT4D(a)) / dir.LengthSq());
	return (POINT_TO_FLOAT4D(a) + d * dir).ToPointVec();
}

vec LineSegment::ClosestPoint(const vec &point, float &d) const
{
	vec dir = b - a;
	d = Clamp01(Dot(point - a, dir) / dir.LengthSq());
	return a + d * dir;
}

vec LineSegment::ClosestPoint(const Line &other, float &d, float &d2) const
{
	Line::ClosestPointLineLine(other.pos, other.dir, a, b - a, d2, d);
	if (d < 0.f)
	{
		d = 0.f;
		other.ClosestPoint(a, d2);
		return a;
	}
	else if (d > 1.f)
	{
		d = 1.f;
		other.ClosestPoint(b, d2);
		return b;
	}
	else
		return GetPoint(d);
}

vec LineSegment::ClosestPoint(const LineSegment &other, float &d, float &d2) const
{
	vec dir = b - a;
	Line::ClosestPointLineLine(a, b - a, other.a, other.b - other.a, d, d2);
	if (d >= 0.f && d <= 1.f && d2 >= 0.f && d2 <= 1.f)
		return a + d * dir;
	else if (d >= 0.f && d <= 1.f) // Only d2 is out of bounds.
	{
		vec p;
		if (d2 < 0.f)
		{
			d2 = 0.f;
			p = other.a;
		}
		else
		{
			d2 = 1.f;
			p = other.b;
		}
		return ClosestPoint(p, d);
	}
	else if (d2 >= 0.f && d2 <= 1.f) // Only d is out of bounds.
	{
		vec p;
		if (d < 0.f)
		{
			d = 0.f;
			p = a;
		}
		else
		{
			d = 1.f;
			p = b;
		}

		other.ClosestPoint(p, d2);
		return p;
	}
	else // Both u and u2 are out of bounds.
	{
		vec p;
		if (d < 0.f)
		{
			p = a;
			d = 0.f;
		}
		else
		{
			p = b;
			d = 1.f;
		}

		vec p2;
		if (d2 < 0.f)
		{
			p2 = other.a;
			d2 = 0.f;
		}
		else
		{
			p2 = other.b;
			d2 = 1.f;
		}

		float T, T2;
		vec closestPoint = ClosestPoint(p2, T);
		vec closestPoint2 = other.ClosestPoint(p, T2);

		if (closestPoint.DistanceSq(p2) <= closestPoint2.DistanceSq(p))
		{
			d = T;
			return closestPoint;
		}
		else
		{
			d2 = T2;
			return p;
		}
	}
}

float LineSegment::Distance(const vec &point, float &d) const
{
	/// See Christer Ericson's Real-Time Collision Detection, p.130.
	vec closestPoint = ClosestPoint(point, d);
	return closestPoint.Distance(point);
}

float LineSegment::DistanceSq(const vec &point) const
{
	float d;
	/// See Christer Ericson's Real-Time Collision Detection, p.130.
	vec closestPoint = ClosestPoint(point, d);
	return closestPoint.DistanceSq(point);
}

double LineSegment::DistanceSqD(const vec &point) const
{
	double d;
	float4d pt = POINT_TO_FLOAT4D(point);
	/// See Christer Ericson's Real-Time Collision Detection, p.130.
	float4d closestPoint = POINT_TO_FLOAT4D(ClosestPointD(pt.ToPointVec(), d));
	return closestPoint.DistanceSq(POINT_TO_FLOAT4D(point));
}

float LineSegment::Distance(const Line &other, float &d, float &d2) const
{
	vec closestPoint2 = other.ClosestPoint(*this, d, d2);
	vec closestPoint = GetPoint(d2);
	return closestPoint.Distance(closestPoint2);
}

float LineSegment::Distance(const LineSegment &other, float &d, float &d2) const
{
	ClosestPoint(other, d, d2);
	return GetPoint(d).Distance(other.GetPoint(d2));
}

float LineSegment::DistanceSq(const LineSegment &other) const
{
	float d, d2;
	ClosestPoint(other, d, d2);
	return GetPoint(d).DistanceSq(other.GetPoint(d2));
}

float LineSegment::Distance(const Plane &other) const
{
	float aDist = other.SignedDistance(a);
	float bDist = other.SignedDistance(b);
	if (aDist * bDist < 0.f)
		return 0.f; // There was an intersection, so the distance is zero.
	return Min(Abs(aDist), Abs(bDist));
}

bool LineSegment::Intersects(const Plane &plane) const
{
	float d = plane.SignedDistance(a);
	float d2 = plane.SignedDistance(b);
	return d * d2 <= 0.f;
}

bool LineSegment::Intersects(const Plane &plane, float *d) const
{
	return plane.Intersects(*this, d);
}

bool LineSegment::Intersects(const Triangle &triangle, float *d, vec *intersect_pt) const
{
	return triangle.Intersects(*this, d, intersect_pt);
}

bool LineSegment::Intersects(const AABB &aabb) const
{
	return aabb.Intersects(*this);
}

bool LineSegment::Intersects(const AABB &aabb, float &near, float &far) const
{
	return aabb.Intersects(*this, near, far);
}

bool LineSegment::Intersects(const OBB &obb) const
{
	return obb.Intersects(*this);
}

bool LineSegment::Intersects(const OBB &obb, float &near, float &far) const
{
	return obb.Intersects(*this, near, far);
}

bool LineSegment::Intersects(const LineSegment &lineseg, float epsilon) const
{
	return Distance(lineseg) <= epsilon;
}

bool LineSegment::Intersects(const Polygon &polygon) const
{
	return polygon.Intersects(*this);
}

bool LineSegment::Intersects(const Circle &disc) const
{
	return disc.Intersects(*this);
}

Line LineSegment::ToLine() const
{
	return Line(a, Dir());
}

void LineSegment::ProjectToAxis(const vec &direction, float &outmin, float &outmax) const
{
	outmin = Dot(direction, a);
	outmax = Dot(direction, b);
	if (outmax < outmin)
		std::swap(outmin, outmax);
}

LineSegment operator *(const float3x3 &transform, const LineSegment &l)
{
	return LineSegment(transform * l.a, transform * l.b);
}

LineSegment operator *(const float3x4 &transform, const LineSegment &l)
{
	return LineSegment(transform.MulPos(l.a), transform.MulPos(l.b));
}

LineSegment operator *(const float4x4 &transform, const LineSegment &l)
{
	return LineSegment(transform.MulPos(l.a), transform.MulPos(l.b));
}

LineSegment operator *(const Quat &transform, const LineSegment &l)
{
	return LineSegment(transform * l.a, transform * l.b);
}

#if defined(MATH_ENABLE_STL_SUPPORT)
std::string LineSegment::ToString() const
{
	char str[256];
	sprintf(str, "LineSegment(a:(%.2f, %.2f, %.2f) b:(%.2f, %.2f, %.2f))",
		a.x, a.y, a.z, b.x, b.y, b.z);
	return str;
}

std::string LineSegment::SerializeToString() const
{
	char str[256];
	char *s = SerializeFloat(a.x, str); *s = ','; ++s;
	s = SerializeFloat(a.y, s); *s = ','; ++s;
	s = SerializeFloat(a.z, s); *s = ','; ++s;
	s = SerializeFloat(b.x, s); *s = ','; ++s;
	s = SerializeFloat(b.y, s); *s = ','; ++s;
	s = SerializeFloat(b.z, s);
	assert(s+1 - str < 256);
	MARK_UNUSED(s);
	return str;
}

std::string LineSegment::SerializeToCodeString() const
{
	return "LineSegment(" + a.SerializeToCodeString() + "," + b.SerializeToCodeString() + ")";
}
#endif

#if defined(MATH_ENABLE_STL_SUPPORT)

std::ostream &operator <<(std::ostream &o, const LineSegment &lineseg)
{
	o << lineseg.ToString();
	return o;
}

#endif

LineSegment LineSegment::FromString(const char *str, const char **outEndStr)
{
	assume(str);
	if (!str)
		return LineSegment(vec::nan, vec::nan);
	LineSegment l;
	MATH_SKIP_WORD(str, "LineSegment(");
	MATH_SKIP_WORD(str, "a:(");
	l.a = PointVecFromString(str, &str);
	MATH_SKIP_WORD(str, " b:(");
	l.b = PointVecFromString(str, &str);
	if (outEndStr)
		*outEndStr = str;
	return l;
}

MATH_END_NAMESPACE
