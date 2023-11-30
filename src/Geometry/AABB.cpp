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

/** @file AABB.cpp
	@author Jukka Jyl�nki
	@brief Implementation for the Axis-Aligned Bounding Box (AABB) geometry object. */
#include "AABB.h"
#include "../Math/MathFunc.h"
#ifdef MATH_ENABLE_STL_SUPPORT
#include <iostream>
#include <utility>
#endif
#include "LineSegment.h"
#include "Line.h"
#include "OBB.h"
#include "Plane.h"
#include "Polygon.h"
#include "../Math/float2.h"
#include "../Math/float3x3.h"
#include "../Math/float3x4.h"
#include "../Math/float4.h"
#include "../Math/float4x4.h"
#include "../Math/Quat.h"
#include "Triangle.h"

#include "../Math/float4x4_neon.h"

MATH_BEGIN_NAMESPACE

AABB::AABB(const vec &minPoint_, const vec &maxPoint_)
:minpt(minPoint_), maxpt(maxPoint_)
{
}

AABB::AABB(const OBB &obb)
{
	SetFrom(obb);
}

void AABB::SetNegativeInfinity()
{
	minpt.SetFromScalar(FLOAT_INF);
	maxpt.SetFromScalar(-FLOAT_INF);
}

void AABB::SetFromCenterAndSize(const vec &center, const vec &size)
{
	vec halfSize = 0.5f * size;
	minpt = center - halfSize;
	maxpt = center + halfSize;
}

void AABB::SetFrom(const OBB &obb)
{
	vec halfSize = Abs(obb.axis[0]*obb.r[0]) + Abs(obb.axis[1]*obb.r[1]) + Abs(obb.axis[2]*obb.r[2]);
	SetFromCenterAndSize(obb.pos, 2.f*halfSize);
}

void AABB::SetFrom(const vec *pts, int num)
{
	assume(pts || num == 0);
	SetNegativeInfinity();
	if (!pts)
		return;
	for(int i = 0; i < num; ++i)
		Enclose(pts[i]);
}

OBB AABB::ToOBB() const
{
	return OBB(*this);
}

bool AABB::IsFinite() const
{
	return minpt.IsFinite() && maxpt.IsFinite();
}

bool AABB::IsDegenerate() const
{
	return !(minpt.x < maxpt.x && minpt.y < maxpt.y && minpt.z < maxpt.z);
}

vec AABB::CenterPoint() const
{
	return (minpt + maxpt) * 0.5f;
}

vec AABB::PointInside(float x, float y, float z) const
{
	assume(0.f <= x && x <= 1.f);
	assume(0.f <= y && y <= 1.f);
	assume(0.f <= z && z <= 1.f);

	vec d = maxpt - minpt;
	return minpt + d.Mul(POINT_VEC(x, y, z));
}

LineSegment AABB::Edge(int edgeIndex) const
{
	assume(0 <= edgeIndex && edgeIndex <= 11);
	switch(edgeIndex)
	{
		default: // For release builds where assume() is disabled, return always the first option if out-of-bounds.
		/* For documentation, here's the segments that are returned:
		case 0: return LineSegment(CornerPoint(0), CornerPoint(1));
		case 1: return LineSegment(CornerPoint(0), CornerPoint(2));
		case 2: return LineSegment(CornerPoint(0), CornerPoint(4));
		case 3: return LineSegment(CornerPoint(1), CornerPoint(3));
		case 4: return LineSegment(CornerPoint(1), CornerPoint(5));
		case 5: return LineSegment(CornerPoint(2), CornerPoint(3));
		case 6: return LineSegment(CornerPoint(2), CornerPoint(6));
		case 7: return LineSegment(CornerPoint(3), CornerPoint(7));
		case 8: return LineSegment(CornerPoint(4), CornerPoint(5));
		case 9: return LineSegment(CornerPoint(4), CornerPoint(6));
		case 10: return LineSegment(CornerPoint(5), CornerPoint(7));
		case 11: return LineSegment(CornerPoint(6), CornerPoint(7));
		*/
		// Force-optimize to avoid calling to CornerPoint for another switch-case statement.
		case 0: return LineSegment(minpt, POINT_VEC(minpt.x, minpt.y, maxpt.z));
		case 1: return LineSegment(minpt, POINT_VEC(minpt.x, maxpt.y, minpt.z));
		case 2: return LineSegment(minpt, POINT_VEC(maxpt.x, minpt.y, minpt.z));
		case 3: return LineSegment(POINT_VEC(minpt.x, minpt.y, maxpt.z), POINT_VEC(minpt.x, maxpt.y, maxpt.z));
		case 4: return LineSegment(POINT_VEC(minpt.x, minpt.y, maxpt.z), POINT_VEC(maxpt.x, minpt.y, maxpt.z));
		case 5: return LineSegment(POINT_VEC(minpt.x, maxpt.y, minpt.z), POINT_VEC(minpt.x, maxpt.y, maxpt.z));
		case 6: return LineSegment(POINT_VEC(minpt.x, maxpt.y, minpt.z), POINT_VEC(maxpt.x, maxpt.y, minpt.z));
		case 7: return LineSegment(POINT_VEC(minpt.x, maxpt.y, maxpt.z), maxpt);
		case 8: return LineSegment(POINT_VEC(maxpt.x, minpt.y, minpt.z), POINT_VEC(maxpt.x, minpt.y, maxpt.z));
		case 9: return LineSegment(POINT_VEC(maxpt.x, minpt.y, minpt.z), POINT_VEC(maxpt.x, maxpt.y, minpt.z));
		case 10: return LineSegment(POINT_VEC(maxpt.x, minpt.y, maxpt.z), maxpt);
		case 11: return LineSegment(POINT_VEC(maxpt.x, maxpt.y, minpt.z), maxpt);
	}
}

vec AABB::CornerPoint(int cornerIndex) const
{
	assume(0 <= cornerIndex && cornerIndex <= 7);
	switch(cornerIndex)
	{
		default: // For release builds where assume() is disabled, return always the first option if out-of-bounds.
		case 0: return minpt;
		case 1: return POINT_VEC(minpt.x, minpt.y, maxpt.z);
		case 2: return POINT_VEC(minpt.x, maxpt.y, minpt.z);
		case 3: return POINT_VEC(minpt.x, maxpt.y, maxpt.z);
		case 4: return POINT_VEC(maxpt.x, minpt.y, minpt.z);
		case 5: return POINT_VEC(maxpt.x, minpt.y, maxpt.z);
		case 6: return POINT_VEC(maxpt.x, maxpt.y, minpt.z);
		case 7: return maxpt;
	}
}

vec AABB::ExtremePoint(const vec &direction) const
{
	return POINT_VEC((direction.x >= 0.f ? maxpt.x : minpt.x),
	                 (direction.y >= 0.f ? maxpt.y : minpt.y),
	                 (direction.z >= 0.f ? maxpt.z : minpt.z));
}

vec AABB::ExtremePoint(const vec &direction, float &project_dist) const
{
	vec extremePoint = ExtremePoint(direction);
	project_dist = extremePoint.Dot(direction);
	return extremePoint;
}

vec AABB::PointOnEdge(int edgeIndex, float u) const
{
	assume(0 <= edgeIndex && edgeIndex <= 11);
	assume(0 <= u && u <= 1.f);

	vec d = maxpt - minpt;
	switch(edgeIndex)
	{
	default: // For release builds where assume() is disabled, return always the first option if out-of-bounds.
	case 0: return POINT_VEC(minpt.x, minpt.y, minpt.z + u * d.z);
	case 1: return POINT_VEC(minpt.x, maxpt.y, minpt.z + u * d.z);
	case 2: return POINT_VEC(maxpt.x, minpt.y, minpt.z + u * d.z);
	case 3: return POINT_VEC(maxpt.x, maxpt.y, minpt.z + u * d.z);

	case 4: return POINT_VEC(minpt.x, minpt.y + u * d.y, minpt.z);
	case 5: return POINT_VEC(maxpt.x, minpt.y + u * d.y, minpt.z);
	case 6: return POINT_VEC(minpt.x, minpt.y + u * d.y, maxpt.z);
	case 7: return POINT_VEC(maxpt.x, minpt.y + u * d.y, maxpt.z);

	case 8: return POINT_VEC(minpt.x + u * d.x, minpt.y, minpt.z);
	case 9: return POINT_VEC(minpt.x + u * d.x, minpt.y, maxpt.z);
	case 10: return POINT_VEC(minpt.x + u * d.x, maxpt.y, minpt.z);
	case 11: return POINT_VEC(minpt.x + u * d.x, maxpt.y, maxpt.z);
	}
}

vec AABB::FaceCenterPoint(int faceIndex) const
{
	assume(0 <= faceIndex && faceIndex <= 5);

	vec center = (minpt + maxpt) * 0.5f;
	switch(faceIndex)
	{
	default: // For release builds where assume() is disabled, return always the first option if out-of-bounds.
	case 0: return POINT_VEC(minpt.x, center.y, center.z);
	case 1: return POINT_VEC(maxpt.x, center.y, center.z);
	case 2: return POINT_VEC(center.x, minpt.y, center.z);
	case 3: return POINT_VEC(center.x, maxpt.y, center.z);
	case 4: return POINT_VEC(center.x, center.y, minpt.z);
	case 5: return POINT_VEC(center.x, center.y, maxpt.z);
	}
}

vec AABB::FacePoint(int faceIndex, float u, float v) const
{
	assume(0 <= faceIndex && faceIndex <= 5);
	assume(0 <= u && u <= 1.f);
	assume(0 <= v && v <= 1.f);

	vec d = maxpt - minpt;
	switch(faceIndex)
	{
	default: // For release builds where assume() is disabled, return always the first option if out-of-bounds.
	case 0: return POINT_VEC(minpt.x, minpt.y + u * d.y, minpt.z + v * d.z);
	case 1: return POINT_VEC(maxpt.x, minpt.y + u * d.y, minpt.z + v * d.z);
	case 2: return POINT_VEC(minpt.x + u * d.x, minpt.y, minpt.z + v * d.z);
	case 3: return POINT_VEC(minpt.x + u * d.x, maxpt.y, minpt.z + v * d.z);
	case 4: return POINT_VEC(minpt.x + u * d.x, minpt.y + v * d.y, minpt.z);
	case 5: return POINT_VEC(minpt.x + u * d.x, minpt.y + v * d.y, maxpt.z);
	}
}

vec AABB::FaceNormal(int faceIndex) const
{
	assume(0 <= faceIndex && faceIndex <= 5);
	switch(faceIndex)
	{
	default: // For release builds where assume() is disabled, return always the first option if out-of-bounds.
	case 0: return DIR_VEC(-1,  0,  0);
	case 1: return DIR_VEC( 1,  0,  0);
	case 2: return DIR_VEC( 0, -1,  0);
	case 3: return DIR_VEC( 0,  1,  0);
	case 4: return DIR_VEC( 0,  0, -1);
	case 5: return DIR_VEC( 0,  0,  1);
	}
}

Plane AABB::FacePlane(int faceIndex) const
{
	assume(0 <= faceIndex && faceIndex <= 5);
	return Plane(FaceCenterPoint(faceIndex), FaceNormal(faceIndex));
}

void AABB::GetCornerPoints(vec *outPointArray) const
{
	assume(outPointArray);
	for(int i = 0; i < 8; ++i)
		outPointArray[i] = CornerPoint(i);
}

void AABB::GetFacePlanes(Plane *outPlaneArray) const
{
	assume(outPlaneArray);
	for(int i = 0; i < 6; ++i)
		outPlaneArray[i] = FacePlane(i);
}

AABB AABB::MinimalEnclosingAABB(const vec *pts, int num)
{
	AABB aabb;
	aabb.SetFrom(pts, num);
	return aabb;
}

void AABB::ExtremePointsAlongAABB(const vec *pts, int num, int &minx, int &maxx, int &miny, int &maxy, int &minz, int &maxz)
{
	assume(pts || num == 0);
	if (!pts)
		return;
	minx = maxx = miny = maxy = minz = maxz = 0;
	for(int i = 1; i < num; ++i)
	{
		if (pts[i].x < pts[minx].x) minx = i;
		if (pts[i].x > pts[maxx].x) maxx = i;
		if (pts[i].y < pts[miny].y) miny = i;
		if (pts[i].y > pts[maxy].y) maxy = i;
		if (pts[i].z < pts[minz].z) minz = i;
		if (pts[i].z > pts[maxz].z) maxz = i;
	}
}

AABB AABB::FromCenterAndSize(const vec &aabbCenterPos, const vec &aabbSize)
{
	vec halfSize = aabbSize * 0.5f;
	return AABB(aabbCenterPos - halfSize, aabbCenterPos + halfSize);
}

vec AABB::Size() const
{
	return maxpt - minpt;
}

vec AABB::HalfSize() const
{
	return Size() * 0.5f;
}

float AABB::Volume() const
{
	vec sz = Size();
	return sz.x * sz.y * sz.z;
}

float AABB::SurfaceArea() const
{
	vec size = Size();
	return 2.f * (size.x*size.y + size.x*size.z + size.y*size.z);
}

void AABB::Translate(const vec &offset)
{
	minpt += offset;
	maxpt += offset;
}

AABB AABB::Translated(const vec &offset) const
{
	return AABB(minpt + offset, maxpt + offset);
}

void AABB::Scale(const vec &centerPoint, float scaleFactor)
{
	minpt = (minpt - centerPoint) * scaleFactor + centerPoint;
	maxpt = (maxpt - centerPoint) * scaleFactor + centerPoint;
}

void AABB::Scale(const vec &centerPoint, const vec &scaleFactor)
{
	float3x4 transform = float3x4::Scale(DIR_TO_FLOAT3(scaleFactor), POINT_TO_FLOAT3(centerPoint)); ///TODO: mat
	minpt = POINT_VEC(transform.MulPos(POINT_TO_FLOAT3(minpt))); ///TODO: mat
	maxpt = POINT_VEC(transform.MulPos(POINT_TO_FLOAT3(maxpt))); ///TODO: mat
}

/// See Christer Ericson's Real-time Collision Detection, p. 87, or
/// James Arvo's "Transforming Axis-aligned Bounding Boxes" in Graphics Gems 1, pp. 548-550.
/// http://www.graphicsgems.org/
template<typename Matrix>
void AABBTransformAsAABB(AABB &aabb, Matrix &m)
{
	const vec centerPoint = (aabb.minpt + aabb.maxpt) * 0.5f;
	const vec halfSize = centerPoint - aabb.minpt;
	vec newCenter = m.MulPos(centerPoint);

	// The following is equal to taking the absolute value of the whole matrix m.
	vec newDir = DIR_VEC(Abs(m[0][0] * halfSize.x) + Abs(m[0][1] * halfSize.y) + Abs(m[0][2] * halfSize.z),
						 Abs(m[1][0] * halfSize.x) + Abs(m[1][1] * halfSize.y) + Abs(m[1][2] * halfSize.z),
						 Abs(m[2][0] * halfSize.x) + Abs(m[2][1] * halfSize.y) + Abs(m[2][2] * halfSize.z));
	aabb.minpt = newCenter - newDir;
	aabb.maxpt = newCenter + newDir;
}

#ifdef MATH_SIMD
void AABBTransformAsAABB_SIMD(AABB &aabb, const float4x4 &m)
{
	simd4f minPt = aabb.minpt;
	simd4f maxPt = aabb.maxpt;
	simd4f centerPoint = muls_ps(add_ps(minPt, maxPt), 0.5f);
	simd4f newCenter = mat4x4_mul_vec4(m.row, centerPoint);

	simd4f halfSize = sub_ps(centerPoint, minPt);
	simd4f x = abs_ps(mul_ps(m.row[0], halfSize));
	simd4f y = abs_ps(mul_ps(m.row[1], halfSize));
	simd4f z = abs_ps(mul_ps(m.row[2], halfSize));
	simd4f w = zero_ps();
	simd4f newDir = hadd4_ps(x, y, z, w);
	aabb.minpt = sub_ps(newCenter, newDir);
	aabb.maxpt = add_ps(newCenter, newDir);
}
#endif

void AABB::TransformAsAABB(const float3x3 &transform)
{
	assume(transform.IsColOrthogonal());
	assume(transform.HasUniformScale());

	AABBTransformAsAABB(*this, transform);
}

void AABB::TransformAsAABB(const float3x4 &transform)
{
	assume(transform.IsColOrthogonal());
	assume(transform.HasUniformScale());

	AABBTransformAsAABB(*this, transform);
}

void AABB::TransformAsAABB(const float4x4 &transform)
{
	assume(transform.IsColOrthogonal3());
	assume(transform.HasUniformScale());
	assume(transform.Row(3).Equals(0,0,0,1));

#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SIMD)
	AABBTransformAsAABB_SIMD(*this, transform);
#else
	AABBTransformAsAABB(*this, transform);
#endif
}

void AABB::TransformAsAABB(const Quat &transform)
{
	vec newCenter = transform.Transform(CenterPoint());
	vec newDir = Abs((transform.Transform(Size()) * 0.5f));
	minpt = newCenter - newDir;
	maxpt = newCenter + newDir;
}

OBB AABB::Transform(const float3x3 &transform) const
{
	OBB obb;
	obb.SetFrom(*this, transform);
	return obb;
}

OBB AABB::Transform(const float3x4 &transform) const
{
	OBB obb;
	obb.SetFrom(*this, transform);
	return obb;
}

OBB AABB::Transform(const float4x4 &transform) const
{
	OBB obb;
	obb.SetFrom(*this, transform);
	return obb;
}

OBB AABB::Transform(const Quat &transform) const
{
	OBB obb;
	obb.SetFrom(*this, transform);
	return obb;
}

vec AABB::ClosestPoint(const vec &tar_pt) const
{
	return tar_pt.Clamp(minpt, maxpt);
}

float AABB::Distance(const vec &point) const
{
	///@todo This function could be slightly optimized. See Christer Ericson's
	/// Real-Time Collision Detection, p.131.
	return ClosestPoint(point).Distance(point);
}

bool AABB::Contains(const vec &point) const
{
// Benchmarking this code is very difficult, since branch prediction makes the scalar version
// look very good. In isolation the scalar version might be better, however when joined with
// other SSE computation, the SIMD variants are probably more efficient because the data is
// already "hot" in the registers. Therefore favoring the SSE version over the scalar version
// when possible.

#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SIMD)
	// Benchmark 'AABBContains_positive': AABB::Contains(point) positive
	//    Best: 2.048 nsecs / 3.5128 ticks, Avg: 2.241 nsecs, Worst: 4.277 nsecs
	// Benchmark 'AABBContains_negative': AABB::Contains(point) negative
	//    Best: 2.048 nsecs / 3.467 ticks, Avg: 2.115 nsecs, Worst: 4.156 nsecs
	// Benchmark 'AABBContains_unpredictable': AABB::Contains(point) unpredictable
	//    Best: 2.590 nsecs / 4.4106 ticks, Avg: 2.978 nsecs, Worst: 6.084 nsecs
	simd4f a = cmplt_ps(point, minpt);
	simd4f b = cmpgt_ps(point, maxpt);
	a = or_ps(a, b);
	return allzero_ps(a) != 0;
#else
	// Benchmark 'AABBContains_positive': AABB::Contains(point) positive
	//    Best: 2.108 nsecs / 3.6022 ticks, Avg: 2.232 nsecs, Worst: 4.638 nsecs
	// Benchmark 'AABBContains_negative': AABB::Contains(point) negative
	//    Best: 1.988 nsecs / 3.361 ticks, Avg: 2.148 nsecs, Worst: 4.457 nsecs
	// Benchmark 'AABBContains_unpredictable': AABB::Contains(point) unpredictable
	//    Best: 3.554 nsecs / 6.0764 ticks, Avg: 3.803 nsecs, Worst: 6.264 nsecs
	return minpt.x <= point.x && point.x <= maxpt.x &&
	       minpt.y <= point.y && point.y <= maxpt.y &&
	       minpt.z <= point.z && point.z <= maxpt.z;
#endif
}

bool AABB::Contains(const LineSegment &lineseg) const
{
	return Contains(Min(lineseg.a, lineseg.b), Max(lineseg.a, lineseg.b));
}

bool AABB::Contains(const vec &aabbMinPoint, const vec &aabbMaxPoint) const
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SIMD)
	simd4f a = cmplt_ps(aabbMinPoint, minpt);
	simd4f b = cmpgt_ps(aabbMaxPoint, maxpt);
	a = or_ps(a, b);
	return allzero_ps(a) != 0;
#else
	return minpt.x <= aabbMinPoint.x && maxpt.x >= aabbMaxPoint.x &&
	       minpt.y <= aabbMinPoint.y && maxpt.y >= aabbMaxPoint.y &&
	       minpt.z <= aabbMinPoint.z && maxpt.z >= aabbMaxPoint.z;
#endif
}

bool AABB::Contains(const OBB &obb) const
{
	return Contains(obb.MinimalEnclosingAABB());
}

bool AABB::Contains(const Triangle &triangle) const
{
	return Contains(triangle.BoundingAABB());
}

bool AABB::Contains(const Polygon &polygon) const
{
	return Contains(polygon.MinimalEnclosingAABB());
}

bool AABB::IntersectLineAABB(const vec &linePos, const vec &lineDir, float &tNear, float &tFar) const
{
	// Never call the SSE version here. The SSE version does not output tNear and tFar, because
	// the memory stores have been profiled to make it slower than the CPP version. Therefore the SSE
	// version does not output tNear and tFar (profile shows it to be about 10x faster than the CPP version).
	return IntersectLineAABB_CPP(linePos, lineDir, tNear, tFar);
}

bool AABB::Intersects(const Line &line) const
{
	float tNear = -FLOAT_INF;
	float tFar = FLOAT_INF;

#ifdef MATH_SIMD
	return IntersectLineAABB_SSE(line.pos, line.dir, tNear, tFar);
#else
	return IntersectLineAABB_CPP(line.pos, line.dir, tNear, tFar);
#endif
}

bool AABB::Intersects(const LineSegment &lineseg) const
{
	vec dir = lineseg.b - lineseg.a;
	float len = dir.Length();
	if (len <= 1e-4f) // Degenerate line segment? Fall back to point-in-AABB test.
		return Contains(lineseg.a);

	float invLen = 1.f / len;
	dir *= invLen;
	float tNear = 0.f, tFar = len;
#ifdef MATH_SIMD
	return IntersectLineAABB_SSE(lineseg.a, dir, tNear, tFar);
#else
	return IntersectLineAABB_CPP(lineseg.a, dir, tNear, tFar);
#endif
}

bool AABB::IntersectLineAABB_CPP(const vec &linePos, const vec &lineDir, float &tNear, float &tFar) const
{
	assume2(lineDir.IsNormalized(), lineDir, lineDir.LengthSq());
	assume2(tNear <= tFar && "AABB::IntersectLineAABB: User gave a degenerate line as input for the intersection test!", tNear, tFar);
	// The user should have inputted values for tNear and tFar to specify the desired subrange [tNear, tFar] of the line
	// for this intersection test.
	// For a Line-AABB test, pass in
	//    tNear = -FLOAT_INF;
	//    tFar = FLOAT_INF;
	// For a LineSegment-AABB test, pass in
	//    tNear = 0.f;
	//    tFar = LineSegment.Length();

	// Test each cardinal plane (X, Y and Z) in turn.
	if (!EqualAbs(lineDir.x, 0.f))
	{
		float recipDir = RecipFast(lineDir.x);
		float t1 = (minpt.x - linePos.x) * recipDir;
		float t2 = (maxpt.x - linePos.x) * recipDir;

		// tNear tracks distance to intersect (enter) the AABB.
		// tFar tracks the distance to exit the AABB.
		if (t1 < t2)
			tNear = Max(t1, tNear), tFar = Min(t2, tFar);
		else // Swap t1 and t2.
			tNear = Max(t2, tNear), tFar = Min(t1, tFar);

		if (tNear > tFar)
			return false; // Box is missed since we "exit" before entering it.
	}
	else if (linePos.x < minpt.x || linePos.x > maxpt.x)
		return false; // The ray can't possibly enter the box, abort.

	if (!EqualAbs(lineDir.y, 0.f))
	{
		float recipDir = RecipFast(lineDir.y);
		float t1 = (minpt.y - linePos.y) * recipDir;
		float t2 = (maxpt.y - linePos.y) * recipDir;

		if (t1 < t2)
			tNear = Max(t1, tNear), tFar = Min(t2, tFar);
		else // Swap t1 and t2.
			tNear = Max(t2, tNear), tFar = Min(t1, tFar);

		if (tNear > tFar)
			return false; // Box is missed since we "exit" before entering it.
	}
	else if (linePos.y < minpt.y || linePos.y > maxpt.y)
		return false; // The ray can't possibly enter the box, abort.

	if (!EqualAbs(lineDir.z, 0.f)) // ray is parallel to plane in question
	{
		float recipDir = RecipFast(lineDir.z);
		float t1 = (minpt.z - linePos.z) * recipDir;
		float t2 = (maxpt.z - linePos.z) * recipDir;

		if (t1 < t2)
			tNear = Max(t1, tNear), tFar = Min(t2, tFar);
		else // Swap t1 and t2.
			tNear = Max(t2, tNear), tFar = Min(t1, tFar);
	}
	else if (linePos.z < minpt.z || linePos.z > maxpt.z)
		return false; // The ray can't possibly enter the box, abort.

	return tNear <= tFar;
}

#ifdef MATH_SIMD
bool AABB::IntersectLineAABB_SSE(const float4 &rayPos, const float4 &rayDir, float tNear, float tFar) const
{
	assume(rayDir.IsNormalized4());
	assume(tNear <= tFar && "AABB::IntersectLineAABB: User gave a degenerate line as input for the intersection test!");
	/* For reference, this is the C++ form of the vectorized SSE code below.

	float4 recipDir = rayDir.RecipFast4();
	float4 t1 = (aabbMinPoint - rayPos).Mul(recipDir);
	float4 t2 = (aabbMaxPoint - rayPos).Mul(recipDir);
	float4 near = t1.Min(t2);
	float4 far = t1.Max(t2);
	float4 rayDirAbs = rayDir.Abs();

	if (rayDirAbs.x > 1e-4f) // ray is parallel to plane in question
	{
		tNear = Max(near.x, tNear); // tNear tracks distance to intersect (enter) the AABB.
		tFar = Min(far.x, tFar); // tFar tracks the distance to exit the AABB.
	}
	else if (rayPos.x < aabbMinPoint.x || rayPos.x > aabbMaxPoint.x) // early-out if the ray can't possibly enter the box.
		return false;

	if (rayDirAbs.y > 1e-4f) // ray is parallel to plane in question
	{
		tNear = Max(near.y, tNear); // tNear tracks distance to intersect (enter) the AABB.
		tFar = Min(far.y, tFar); // tFar tracks the distance to exit the AABB.
	}
	else if (rayPos.y < aabbMinPoint.y || rayPos.y > aabbMaxPoint.y) // early-out if the ray can't possibly enter the box.
		return false;

	if (rayDirAbs.z > 1e-4f) // ray is parallel to plane in question
	{
		tNear = Max(near.z, tNear); // tNear tracks distance to intersect (enter) the AABB.
		tFar = Min(far.z, tFar); // tFar tracks the distance to exit the AABB.
	}
	else if (rayPos.z < aabbMinPoint.z || rayPos.z > aabbMaxPoint.z) // early-out if the ray can't possibly enter the box.
		return false;

	return tNear < tFar;
	*/

	simd4f recipDir = rcp_ps(rayDir.v);
	// Note: The above performs an approximate reciprocal (11 bits of precision).
	// For a full precision reciprocal, perform a div:
//	simd4f recipDir = div_ps(set1_ps(1.f), rayDir.v);

	simd4f t1 = mul_ps(sub_ps(minpt, rayPos.v), recipDir);
	simd4f t2 = mul_ps(sub_ps(maxpt, rayPos.v), recipDir);

	simd4f nearD = min_ps(t1, t2); // [0 n3 n2 n1]
	simd4f farD = max_ps(t1, t2);  // [0 f3 f2 f1]

	// Check if the ray direction is parallel to any of the cardinal axes, and if so,
	// mask those [near, far] ranges away from the hit test computations.
	simd4f rayDirAbs = abs_ps(rayDir.v);

	const simd4f epsilon = set1_ps(1e-4f);
	// zeroDirections[i] will be nonzero for each axis i the ray is parallel to.
	simd4f zeroDirections = cmple_ps(rayDirAbs, epsilon);

	const simd4f floatInf = set1_ps(FLOAT_INF);
	const simd4f floatNegInf = set1_ps(-FLOAT_INF);

	// If the ray is parallel to one of the axes, replace the slab range for that axis
	// with [-inf, inf] range instead. (which is a no-op in the comparisons below)
	nearD = cmov_ps(nearD, floatNegInf, zeroDirections);
	farD = cmov_ps(farD, floatInf, zeroDirections);

	// Next, we need to compute horizontally max(nearD[0], nearD[1], nearD[2]) and min(farD[0], farD[1], farD[2])
	// to see if there is an overlap in the hit ranges.
	simd4f v1 = axx_bxx_ps(nearD, farD); // [f1 f1 n1 n1]
	simd4f v2 = ayy_byy_ps(nearD, farD); // [f2 f2 n2 n2]
	simd4f v3 = azz_bzz_ps(nearD, farD); // [f3 f3 n3 n3]
	nearD = max_ps(v1, max_ps(v2, v3));
	farD = min_ps(v1, min_ps(v2, v3));
	farD = wwww_ps(farD); // Unpack the result from high offset in the register.
	nearD = max_ps(nearD, setx_ps(tNear));
	farD = min_ps(farD, setx_ps(tFar));

	// Finally, test if the ranges overlap.
	simd4f rangeIntersects = cmple_ps(nearD, farD); // Only x channel used, higher ones ignored.

	// To store out out the interval of intersection, uncomment the following:
	// These are disabled, since without these, the whole function runs without a single memory store,
	// which has been profiled to be very fast! Uncommenting these causes an order-of-magnitude slowdown.
	// For now, using the SSE version only where the tNear and tFar ranges are not interesting.
//	_mm_store_ss(&tNear, nearD);
//	_mm_store_ss(&tFar, farD);

	// To avoid false positives, need to have an additional rejection test for each cardinal axis the ray direction
	// is parallel to.
	simd4f out2 = cmplt_ps(rayPos.v, minpt);
	simd4f out3 = cmpgt_ps(rayPos.v, maxpt);
	out2 = or_ps(out2, out3);
	zeroDirections = and_ps(zeroDirections, out2);

	simd4f yOut = yyyy_ps(zeroDirections);
	simd4f zOut = zzzz_ps(zeroDirections);

	zeroDirections = or_ps(or_ps(zeroDirections, yOut), zOut);
	// Intersection occurs if the slab ranges had positive overlap and if the test was not rejected by the ray being
	// parallel to some cardinal axis.
	simd4f intersects = andnot_ps(zeroDirections, rangeIntersects);
	simd4f epsilonMasked = and_ps(epsilon, intersects);
	return comieq_ss(epsilon, epsilonMasked) != 0;
}
#endif

bool AABB::Intersects(const Line &line, float &near, float &far) const
{
	near = -FLOAT_INF;
	far = FLOAT_INF;
	return IntersectLineAABB(line.pos, line.dir, near, far);
}

bool AABB::Intersects(const LineSegment &lineseg, float &near, float &far) const
{
	vec dir = lineseg.b - lineseg.a;
	float len = dir.Length();
	if (len <= 1e-4f) // Degenerate line segment? Fall back to point-in-AABB test.
	{
		near = 0.f;
		far = 1.f;
		return Contains(lineseg.a);
	}
	float invLen = 1.f / len;
	dir *= invLen;
	near = 0.f;
	far = len;
	bool hit = IntersectLineAABB(lineseg.a, dir, near, far);
	near *= invLen;
	far *= invLen;
	return hit;
}

bool AABB::Intersects(const Plane &plane) const
{
	return plane.Intersects(*this);
}

bool AABB::Intersects(const AABB &aabb) const
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SIMD)
	// Benchmark 'AABBIntersectsAABB_positive': AABB::Intersects(AABB) positive
	//    Best: 2.229 nsecs / 3.848 ticks, Avg: 2.409 nsecs, Worst: 4.457 nsecs
	// Benchmark 'AABBIntersectsAABB_random': AABB::Intersects(AABB) random
	//    Best: 3.072 nsecs / 5.2904 ticks, Avg: 3.262 nsecs, Worst: 5.301 nsecs

	simd4f a = cmpge_ps(minpt.v, aabb.maxpt.v);
	simd4f b = cmpge_ps(aabb.minpt.v, maxpt.v);
	a = or_ps(a, b);
	return a_and_b_allzero_ps(a, set_ps_hex(0, 0xFFFFFFFFU, 0xFFFFFFFFU, 0xFFFFFFFFU)) != 0; // Mask off results from the W channel.
#else
	// Benchmark 'AABBIntersectsAABB_positive': AABB::Intersects(AABB) positive
	//    Best: 2.108 nsecs / 3.588 ticks, Avg: 2.310 nsecs, Worst: 5.481 nsecs
	// Benchmark 'AABBIntersectsAABB_random': AABB::Intersects(AABB) random
	//    Best: 7.529 nsecs / 12.8282 ticks, Avg: 8.892 nsecs, Worst: 16.323 nsecs

	// If any of the cardinal X,Y,Z axes is a separating axis, then
	// there is no intersection.
	return minpt.x < aabb.maxpt.x &&
	       minpt.y < aabb.maxpt.y &&
	       minpt.z < aabb.maxpt.z &&
	       aabb.minpt.x < maxpt.x &&
	       aabb.minpt.y < maxpt.y &&
	       aabb.minpt.z < maxpt.z;
#endif
}

bool AABB::Intersects(const OBB &obb) const
{
	return obb.Intersects(*this);
}

bool AABB::Intersects(const Triangle &triangle) const
{
	return triangle.Intersects(*this);
}

bool AABB::Intersects(const Polygon &polygon) const
{
	return polygon.Intersects(*this);
}

void AABB::ProjectToAxis(const vec &axis, float &dMin, float &dMax) const
{
	vec c = (minpt + maxpt) * 0.5f;
	vec e = maxpt - c;

#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SIMD)
	vec absAxis = axis.Abs();
	float r = Abs(e.Dot(absAxis));
#else
	// Compute the projection interval radius of the AABB onto L(t) = aabb.center + t * plane.normal;
	float r = Abs(e[0]*Abs(axis[0]) + e[1]*Abs(axis[1]) + e[2]*Abs(axis[2]));
#endif
	// Compute the distance of the box center from plane.
	float s = axis.Dot(c);
	dMin = s - r;
	dMax = s + r;
}

int AABB::UniqueFaceNormals(vec *out) const
{
	out[0] = DIR_VEC(1,0,0);
	out[1] = DIR_VEC(0,1,0);
	out[2] = DIR_VEC(0,0,1);
	return 3;
}

int AABB::UniqueEdgeDirections(vec *out) const
{
	out[0] = DIR_VEC(1,0,0);
	out[1] = DIR_VEC(0,1,0);
	out[2] = DIR_VEC(0,0,1);
	return 3;
}

void AABB::Enclose(const vec &point)
{
	minpt = Min(minpt, point);
	maxpt = Max(maxpt, point);
}

void AABB::Enclose(const LineSegment &lineseg)
{
	Enclose(Min(lineseg.a, lineseg.b), Max(lineseg.a, lineseg.b));
}

void AABB::Enclose(const vec &aabbMinPoint, const vec &aabbMaxPoint)
{
	minpt = Min(minpt, aabbMinPoint);
	maxpt = Max(maxpt, aabbMaxPoint);
}

void AABB::Enclose(const OBB &obb)
{
	vec absAxis0 = obb.axis[0].Abs();
	vec absAxis1 = obb.axis[1].Abs();
	vec absAxis2 = obb.axis[2].Abs();
	vec d = obb.r.x * absAxis0 + obb.r.y * absAxis1 + obb.r.z * absAxis2;
	Enclose(obb.pos - d, obb.pos + d);
}

void AABB::Enclose(const Triangle &triangle)
{
	Enclose(Min(triangle.a, triangle.b, triangle.c), Max(triangle.a, triangle.b, triangle.c));
}

void AABB::Enclose(const Polygon &polygon)
{
	Enclose(polygon.MinimalEnclosingAABB());
}

void AABB::Enclose(const vec *pts, int num)
{
	assume(pts || num == 0);
	if (!pts)
		return;
	for(int i = 0; i < num; ++i)
		Enclose(pts[i]);
}

void AABB::Triangulate(int numFacesX, int numFacesY, int numFacesZ,
                       vec *outPos, vec *outNormal, float2 *outUV,
                       bool ccwIsFrontFacing) const
{
	assume(numFacesX >= 1);
	assume(numFacesY >= 1);
	assume(numFacesZ >= 1);

	assume(outPos);
	if (!outPos)
		return;

	// Generate both X-Y planes.
	int i = 0;
	for(int face = 0; face < 6; ++face) // Faces run in the order -X, +X, -Y, +Y, -Z, +Z.
	{
		int numFacesU;
		int numFacesV;
		bool flip = (face == 1 || face == 2 || face == 5);
		if (ccwIsFrontFacing)
			flip = !flip;
		if (face == 0 || face == 1)
		{
			numFacesU = numFacesY;
			numFacesV = numFacesZ;
		}
		else if (face == 2 || face == 3)
		{
			numFacesU = numFacesX;
			numFacesV = numFacesZ;
		}
		else// if (face == 4 || face == 5)
		{
			numFacesU = numFacesX;
			numFacesV = numFacesY;
		}
		for(int x = 0; x < numFacesU; ++x)
			for(int y = 0; y < numFacesV; ++y)
			{
				float u = (float)x / (numFacesU);
				float v = (float)y / (numFacesV);
				float u2 = (float)(x+1) / (numFacesU);
				float v2 = (float)(y+1) / (numFacesV);

				outPos[i]   = FacePoint(face, u, v);
				outPos[i+1] = FacePoint(face, u, v2);
				outPos[i+2] = FacePoint(face, u2, v);
				if (flip)
					std::swap(outPos[i+1], outPos[i+2]);
				outPos[i+3] = outPos[i+2];
				outPos[i+4] = outPos[i+1];
				outPos[i+5] = FacePoint(face, u2, v2);

				if (outUV)
				{
					outUV[i]   = float2(u,v);
					outUV[i+1] = float2(u,v2);
					outUV[i+2] = float2(u2,v);
					if (flip)
						std::swap(outUV[i+1], outUV[i+2]);
					outUV[i+3] = outUV[i+2];
					outUV[i+4] = outUV[i+1];
					outUV[i+5] = float2(u2,v2);
				}

				if (outNormal)
					for(int j = 0; j < 6; ++j)
						outNormal[i+j] = FaceNormal(face);

				i += 6;
			}
	}
	assert(i == NumVerticesInTriangulation(numFacesX, numFacesY, numFacesZ));
}

void AABB::ToEdgeList(vec *outPos) const
{
	assume(outPos);
	if (!outPos)
		return;
	for(int i = 0; i < 12; ++i)
	{
		LineSegment edge = Edge(i);
		outPos[i*2] = edge.a;
		outPos[i*2+1] = edge.b;
	}
}

#ifdef MATH_ENABLE_STL_SUPPORT
std::string AABB::ToString() const
{
	char str[256];
	sprintf(str, "AABB(Min:(%.2f, %.2f, %.2f) Max:(%.2f, %.2f, %.2f))", minpt.x, minpt.y, minpt.z, maxpt.x, maxpt.y, maxpt.z);
	return str;
}

std::string AABB::SerializeToString() const
{
	char str[256];
	char *s = SerializeFloat(minpt.x, str); *s = ','; ++s;
	s = SerializeFloat(minpt.y, s); *s = ','; ++s;
	s = SerializeFloat(minpt.z, s); *s = ','; ++s;
	s = SerializeFloat(maxpt.x, s); *s = ','; ++s;
	s = SerializeFloat(maxpt.y, s); *s = ','; ++s;
	s = SerializeFloat(maxpt.z, s);
	assert(s+1 - str < 256);
	MARK_UNUSED(s);
	return str;
}

std::string AABB::SerializeToCodeString() const
{
	return "AABB(" + minpt.SerializeToCodeString() + "," + maxpt.SerializeToCodeString() + ")";
}

std::ostream &operator <<(std::ostream &o, const AABB &aabb)
{
	o << aabb.ToString();
	return o;
}

#endif

AABB AABB::Intersection(const AABB &aabb) const
{
	return AABB(Max(minpt, aabb.minpt), Min(maxpt, aabb.maxpt));
}

OBB operator *(const float3x3 &transform, const AABB &aabb)
{
	return aabb.Transform(transform);
}

OBB operator *(const float3x4 &transform, const AABB &aabb)
{
	return aabb.Transform(transform);
}

OBB operator *(const float4x4 &transform, const AABB &aabb)
{
	return aabb.Transform(transform);
}

OBB operator *(const Quat &transform, const AABB &aabb)
{
	return aabb.Transform(transform);
}

MATH_END_NAMESPACE
