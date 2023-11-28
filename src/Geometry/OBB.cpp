/* Copyright Jukka Jylänki

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License. */

/** @file OBB.cpp
	@author Jukka Jylänki
	@brief Implementation for the Oriented Bounding Box (OBB) geometry object. */
#include "OBB.h"
#ifdef MATH_ENABLE_STL_SUPPORT
#include <iostream>
#include <utility>
#endif
#include "../Math/MathFunc.h"
#include "AABB.h"
#include "LineSegment.h"
#include "Line.h"
#include "Plane.h"
#include "Polygon.h"
#include "../Math/float2.inl"
#include "../Math/float3x3.h"
#include "../Math/float3x4.h"
#include "../Math/float4.h"
#include "../Math/float4x4.h"
#include "../Math/Quat.h"
#include "Ray.h"
#include "Triangle.h"
#include <stdlib.h>
#include "../Time/Clock.h"

#include <set>
#ifdef MATH_CONTAINERLIB_SUPPORT
#include "Algorithm/Sort/Sort.h"
#endif

#ifdef MATH_GRAPHICSENGINE_INTEROP
#include "VertexBuffer.h"
#endif

#if defined(MATH_SIMD) && defined(MATH_AUTOMATIC_SSE)
#include "../Math/float4_neon.h"
#include "../Math/float4_sse.h"
#include "../Math/float4x4_sse.h"
#endif

#define MATH_ENCLOSINGOBB_DOUBLE_PRECISION

#ifdef MATH_ENCLOSINGOBB_DOUBLE_PRECISION
#include "../Math/float4d.h"
#endif

MATH_BEGIN_NAMESPACE

#ifdef MATH_ENCLOSINGOBB_DOUBLE_PRECISION
typedef float4d cv;
typedef double cs;
typedef std::vector<float4d> VecdArray;
#else
typedef vec cv;
typedef float cs;
typedef VecArray VecdArray;
#endif

OBB::OBB(const vec &pos, const vec &r, const vec &axis0, const vec &axis1, const vec &axis2)
:pos(pos), r(r)
{
	axis[0] = axis0;
	axis[1] = axis1;
	axis[2] = axis2;
}

OBB::OBB(const AABB &aabb)
{
	SetFrom(aabb);
}

void OBB::SetNegativeInfinity()
{
	pos = POINT_VEC_SCALAR(0.f);
	r.SetFromScalar(-FLOAT_INF);
	axis[0] = DIR_VEC(1,0,0);
	axis[1] = DIR_VEC(0, 1, 0);
	axis[2] = DIR_VEC(0, 0, 1);
}

void OBB::SetFrom(const AABB &aabb)
{
	pos = aabb.CenterPoint();
	r = aabb.HalfSize();
	axis[0] = DIR_VEC(1, 0, 0);
	axis[1] = DIR_VEC(0, 1, 0);
	axis[2] = DIR_VEC(0, 0, 1);
}

template<typename Matrix>
void OBBSetFrom(OBB &obb, const AABB &aabb, const Matrix &m)
{
	assume1(m.IsColOrthogonal(), m); // We cannot convert transform an AABB to OBB if it gets sheared in the process.
	assume(m.HasUniformScale()); // Nonuniform scale will produce shear as well.
	obb.pos = m.MulPos(aabb.CenterPoint());
	obb.r = aabb.HalfSize();
	obb.axis[0] = DIR_VEC(m.Col(0));
	obb.axis[1] = DIR_VEC(m.Col(1));
	obb.axis[2] = DIR_VEC(m.Col(2));
	// If the matrix m contains scaling, propagate the scaling from the axis vectors to the half-length vectors,
	// since we want to keep the axis vectors always normalized in our representation.
	float matrixScale = obb.axis[0].LengthSq();
	matrixScale = Sqrt(matrixScale);
	obb.r *= matrixScale;
	matrixScale = 1.f / matrixScale;
	obb.axis[0] *= matrixScale;
	obb.axis[1] *= matrixScale;
	obb.axis[2] *= matrixScale;

//	mathassert(vec::AreOrthogonal(obb.axis[0], obb.axis[1], obb.axis[2]));
//	mathassert(vec::AreOrthonormal(obb.axis[0], obb.axis[1], obb.axis[2]));
	///@todo Would like to simply do the above, but instead numerical stability requires to do the following:
	vec::Orthonormalize(obb.axis[0], obb.axis[1], obb.axis[2]);
}

void OBB::SetFrom(const AABB &aabb, const float3x3 &transform)
{
	assume(transform.IsColOrthogonal());
	OBBSetFrom(*this, aabb, transform);
}

void OBB::SetFrom(const AABB &aabb, const float3x4 &transform)
{
	OBBSetFrom(*this, aabb, transform);
}

void OBB::SetFrom(const AABB &aabb, const float4x4 &transform)
{
	assume(transform.Row(3).Equals(0,0,0,1));
	OBBSetFrom(*this, aabb, transform.Float3x4Part());
}

void OBB::SetFrom(const AABB &aabb, const Quat &transform)
{
	OBBSetFrom(*this, aabb, float3x3(transform));
}

AABB OBB::MinimalEnclosingAABB() const
{
	AABB aabb;
	aabb.SetFrom(*this);
	return aabb;
}

#if 0

AABB OBB::MaximalContainedAABB() const
{
#ifdef _MSC_VER
#pragma warning(OBB::MaximalContainedAABB not implemented!)
#else
#warning OBB::MaximalContainedAABB not implemented!
#endif
	assume(false && "OBB::MaximalContainedAABB not implemented!"); /// @todo Implement.
	return AABB();
}
#endif

bool OBB::IsFinite() const
{
	return pos.IsFinite() && r.IsFinite() && axis[0].IsFinite() && axis[1].IsFinite() && axis[2].IsFinite();
}

bool OBB::IsDegenerate() const
{
	return !(r.x > 0.f && r.y > 0.f && r.z > 0.f);
}

vec OBB::CenterPoint() const
{
	return pos;
}

vec OBB::PointInside(float x, float y, float z) const
{
	assume(0.f <= x && x <= 1.f);
	assume(0.f <= y && y <= 1.f);
	assume(0.f <= z && z <= 1.f);

	return pos + axis[0] * (2.f * r.x * x - r.x)
			   + axis[1] * (2.f * r.y * y - r.y)
			   + axis[2] * (2.f * r.z * z - r.z);
}

LineSegment OBB::Edge(int edgeIndex) const
{
	assume(0 <= edgeIndex && edgeIndex <= 11);
	switch(edgeIndex)
	{
		default: // For release builds where assume() is disabled, return always the first option if out-of-bounds.
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
	}
}

vec OBB::CornerPoint(int cornerIndex) const
{
	assume(0 <= cornerIndex && cornerIndex <= 7);
	switch(cornerIndex)
	{
		default: // For release builds where assume() is disabled, return always the first option if out-of-bounds.
		case 0: return pos - r.x * axis[0] - r.y * axis[1] - r.z * axis[2];
		case 1: return pos - r.x * axis[0] - r.y * axis[1] + r.z * axis[2];
		case 2: return pos - r.x * axis[0] + r.y * axis[1] - r.z * axis[2];
		case 3: return pos - r.x * axis[0] + r.y * axis[1] + r.z * axis[2];
		case 4: return pos + r.x * axis[0] - r.y * axis[1] - r.z * axis[2];
		case 5: return pos + r.x * axis[0] - r.y * axis[1] + r.z * axis[2];
		case 6: return pos + r.x * axis[0] + r.y * axis[1] - r.z * axis[2];
		case 7: return pos + r.x * axis[0] + r.y * axis[1] + r.z * axis[2];
	}
}

vec OBB::ExtremePoint(const vec &direction) const
{
	vec pt = pos;
	pt += axis[0] * (Dot(direction, axis[0]) >= 0.f ? r.x : -r.x);
	pt += axis[1] * (Dot(direction, axis[1]) >= 0.f ? r.y : -r.y);
	pt += axis[2] * (Dot(direction, axis[2]) >= 0.f ? r.z : -r.z);
	return pt;
}

vec OBB::ExtremePoint(const vec &direction, float &projectionDistance) const
{
	vec extremePoint = ExtremePoint(direction);
	projectionDistance = extremePoint.Dot(direction);
	return extremePoint;
}

void OBB::ProjectToAxis(const vec &direction, float &outMin, float &outMax) const
{
	float x = Abs(Dot(direction, axis[0]) * r.x);
	float y = Abs(Dot(direction, axis[1]) * r.y);
	float z = Abs(Dot(direction, axis[2]) * r.z);
	float pt = Dot(direction, pos);
	outMin = pt - x - y - z;
	outMax = pt + x + y + z;
}

int OBB::UniqueFaceNormals(vec *out) const
{
	out[0] = axis[0];
	out[1] = axis[1];
	out[2] = axis[2];
	return 3;
}

int OBB::UniqueEdgeDirections(vec *out) const
{
	out[0] = axis[0];
	out[1] = axis[1];
	out[2] = axis[2];
	return 3;
}

vec OBB::PointOnEdge(int edgeIndex, float u) const
{
	assume(0 <= edgeIndex && edgeIndex <= 11);
	assume(0 <= u && u <= 1.f);

	edgeIndex = Clamp(edgeIndex, 0, 11);
	vec d = axis[edgeIndex/4] * (2.f * u - 1.f) * r[edgeIndex/4];
	switch(edgeIndex)
	{
	default: // For release builds where assume() is disabled, return always the first option if out-of-bounds.
	case 0: return pos - r.y * axis[1] - r.z * axis[2] + d;
	case 1: return pos - r.y * axis[1] + r.z * axis[2] + d;
	case 2: return pos + r.y * axis[1] - r.z * axis[2] + d;
	case 3: return pos + r.y * axis[1] + r.z * axis[2] + d;

	case 4: return pos - r.x * axis[0] - r.z * axis[2] + d;
	case 5: return pos - r.x * axis[0] + r.z * axis[2] + d;
	case 6: return pos + r.x * axis[0] - r.z * axis[2] + d;
	case 7: return pos + r.x * axis[0] + r.z * axis[2] + d;

	case 8: return pos - r.x * axis[0] - r.y * axis[1] + d;
	case 9: return pos - r.x * axis[0] + r.y * axis[1] + d;
	case 10: return pos + r.x * axis[0] - r.y * axis[1] + d;
	case 11: return pos + r.x * axis[0] + r.y * axis[1] + d;
	}
}

vec OBB::FaceCenterPoint(int faceIndex) const
{
	assume(0 <= faceIndex && faceIndex <= 5);

	switch(faceIndex)
	{
	default: // For release builds where assume() is disabled, return always the first option if out-of-bounds.
	case 0: return pos - r.x * axis[0];
	case 1: return pos + r.x * axis[0];
	case 2: return pos - r.y * axis[1];
	case 3: return pos + r.y * axis[1];
	case 4: return pos - r.z * axis[2];
	case 5: return pos + r.z * axis[2];
	}
}

vec OBB::FacePoint(int faceIndex, float u, float v) const
{
	assume(0 <= faceIndex && faceIndex <= 5);
	assume(0 <= u && u <= 1.f);
	assume(0 <= v && v <= 1.f);

	int uIdx = faceIndex/2;
	int vIdx = (faceIndex/2 + 1) % 3;
	vec U = axis[uIdx] * (2.f * u - 1.f) * r[uIdx];
	vec V = axis[vIdx] * (2.f * v - 1.f) * r[vIdx];
	switch(faceIndex)
	{
	default: // For release builds where assume() is disabled, return always the first option if out-of-bounds.
	case 0: return pos - r.z * axis[2] + U + V;
	case 1: return pos + r.z * axis[2] + U + V;
	case 2: return pos - r.x * axis[0] + U + V;
	case 3: return pos + r.x * axis[0] + U + V;
	case 4: return pos - r.y * axis[1] + U + V;
	case 5: return pos + r.y * axis[1] + U + V;
	}
}

Plane OBB::FacePlane(int faceIndex) const
{
	assume(0 <= faceIndex && faceIndex <= 5);
	switch(faceIndex)
	{
	default: // For release builds where assume() is disabled, return always the first option if out-of-bounds.
	case 0: return Plane(FaceCenterPoint(0), -axis[0]);
	case 1: return Plane(FaceCenterPoint(1), axis[0]);
	case 2: return Plane(FaceCenterPoint(2), -axis[1]);
	case 3: return Plane(FaceCenterPoint(3), axis[1]);
	case 4: return Plane(FaceCenterPoint(4), -axis[2]);
	case 5: return Plane(FaceCenterPoint(5), axis[2]);
	}
}

void OBB::GetCornerPoints(vec *outPointArray) const
{
	assume(outPointArray);
	for(int i = 0; i < 8; ++i)
		outPointArray[i] = CornerPoint(i);
}

void OBB::GetFacePlanes(Plane *outPlaneArray) const
{
	assume(outPlaneArray);
	for(int i = 0; i < 6; ++i)
		outPlaneArray[i] = FacePlane(i);
}

/// See Christer Ericson's book Real-Time Collision Detection, page 83.
void OBB::ExtremePointsAlongDirection(const vec &dir, const vec *pointArray, int numPoints, int &idxSmallest, int &idxLargest, float &smallestD, float &largestD)
{
	assume(pointArray || numPoints == 0);

	idxSmallest = idxLargest = 0;

	smallestD = FLOAT_INF;
	largestD = -FLOAT_INF;
	for(int i = 0; i < numPoints; ++i)
	{
		float d = Dot(pointArray[i], dir);
		if (d < smallestD)
		{
			smallestD = d;
			idxSmallest = i;
		}
		if (d > largestD)
		{
			largestD = d;
			idxLargest = i;
		}
	}
}

OBB SmallestOBBVolumeOneEdgeFixed(const vec &edge, const vec *pointArray, int numPoints)
{
	vec u, v;
	edge.PerpendicularBasis(u, v);

	std::vector<float2> pts;
	pts.resize(numPoints);
	for(int k = 0; k < numPoints; ++k)
		pts[k] = float2(u.Dot(pointArray[k]), v.Dot(pointArray[k]));

	float2 rectCenter;
	float2 uDir;
	float2 vDir;
	float minU, maxU, minV, maxV;
	float2::MinAreaRectInPlace(&pts[0], (int)pts.size(), rectCenter, uDir, vDir, minU, maxU, minV, maxV);
	vec basisU = uDir.x * u + uDir.y * v;
	vec basisV = vDir.x * u + vDir.y * v;
	assume2(basisU.IsPerpendicular(basisV), basisU, basisV);
	return OBB::FixedOrientationEnclosingOBB(pointArray, numPoints, basisU, basisV);
}

float SmallestOBBVolumeJiggle(const vec &edge_, const vec *pointArray, int numPoints, std::vector<float2> &pts,
	vec &outEdgeA, vec &outEdgeB)
{
	assume(numPoints >= 3);
	vec edge = edge_;
	int numTimesNotImproved = 0;
	float bestVolume = FLOAT_INF;
	vec edgeFirstChoice, edgeSecondChoice;
	pts.resize(numPoints);
	while(numTimesNotImproved <= 1)
	{
		int e1, e2;
		OBB::ExtremePointsAlongDirection(edge, pointArray, numPoints, e1, e2);
		float edgeLength = Abs(Dot(pointArray[e1] - pointArray[e2], edge));

		vec u, v;
		edge.PerpendicularBasis(u, v);

		for(int k = 0; k < numPoints; ++k)
			pts[k] = float2(u.Dot(pointArray[k]), v.Dot(pointArray[k]));

		float2 rectCenter;
		float2 uDir;
		float2 vDir;
		float minU, maxU, minV, maxV;

		float2::MinAreaRectInPlace(&pts[0], (int)pts.size(), rectCenter, uDir, vDir, minU, maxU, minV, maxV);
		assume(uDir.IsNormalized());
		assume(vDir.IsNormalized());
		assume(uDir.IsPerpendicular(vDir));
		float2 c10 = (maxV - minV) * vDir;
		float2 c20 = (maxU - minU) * uDir;

		vec edge1BackIn3D = (c10.x*u + c10.y*v);
		vec edge2BackIn3D = (c20.x*u + c20.y*v);

		float volume = edge1BackIn3D.Length() * edge2BackIn3D.Length() * edgeLength;

		if (volume < bestVolume)
		{
			bestVolume = volume;
			edgeFirstChoice = (uDir.x*u + uDir.y*v);
			assume(edgeFirstChoice.IsPerpendicular(edge));
			float len = edgeFirstChoice.Normalize();
			assert(len > 0.f);

			edgeSecondChoice = (vDir.x*u + vDir.y*v);
			len = edgeSecondChoice.Normalize();
			assert(len > 0.f);

			assume3(edgeSecondChoice.IsPerpendicular(edge), edgeSecondChoice, edge, edgeSecondChoice.Dot(edge));
			assume3(edgeSecondChoice.IsPerpendicular(edgeFirstChoice), edgeSecondChoice, edgeFirstChoice, edgeSecondChoice.Dot(edgeFirstChoice));
			outEdgeA = edge;
			outEdgeB = edge = edgeFirstChoice;
			numTimesNotImproved = 0;
		}
		else
		{
			++numTimesNotImproved;
			edge = edgeSecondChoice;
		}
	}
	return bestVolume;
}

OBB SmallestOBBVolume2DPlanar(const vec *pointArray, int numPoints)
{
	vec commonPlaneNormal = vec::zero;
	for(int i = 2; i < numPoints; ++i)
	{
		vec normal = (pointArray[i] - pointArray[i-2]).Cross(pointArray[i] - pointArray[i-1]);
		if (normal.Normalize() > 0.f)
		{
			if (commonPlaneNormal.Dot(normal) >= 0)
				commonPlaneNormal += normal;
			else
				commonPlaneNormal -= normal;
			commonPlaneNormal.Normalize();
		}
	}
	std::vector<float2> pts;
	pts.resize(numPoints);
	vec edgeA, edgeB;
	SmallestOBBVolumeJiggle(commonPlaneNormal, pointArray, numPoints, pts, edgeA, edgeB);
	return OBB::FixedOrientationEnclosingOBB(pointArray, numPoints, edgeA, edgeB);
//	return SmallestOBBVolumeOneEdgeFixed(edgeA, pointArray, numPoints);
}

// Moves the floating point sign bit from src to dst.
#ifdef MATH_SSE
#define MoveSign(dst, src) \
	dst = s4f_x(xor_ps(setx_ps(dst), and_ps(setx_ps(src), simd4fSignBit))); \
	src = s4f_x(abs_ps(setx_ps(src)));
#else
#define MoveSign(dst, src) if (src < 0.f) { dst = -dst; src = -src; }
#endif

int ComputeBasis(const vec &f1a, const vec &f1b,
	const vec &f2a, const vec &f2b,
	const vec &f3a, const vec &f3b,
	vec *n1,
	vec *n2,
	vec *n3)
{
	const float eps = 1e-4f;
	const float angleEps = 1e-3f;

	{
		vec a = f1b;
		vec b = f1a-f1b;
		vec c = f2b;
		vec d = f2a-f2b;
		vec e = f3b;
		vec f = f3a-f3b;

		float g = a.Dot(c)*d.Dot(e) - a.Dot(d)*c.Dot(e);
		float h = a.Dot(c)*d.Dot(f) - a.Dot(d)*c.Dot(f);
		float i = b.Dot(c)*d.Dot(e) - b.Dot(d)*c.Dot(e);
		float j = b.Dot(c)*d.Dot(f) - b.Dot(d)*c.Dot(f);

		float k = g*b.Dot(e) - a.Dot(e)*i;
		float l = h*b.Dot(e) + g*b.Dot(f) - a.Dot(f)*i - a.Dot(e)*j;
		float m = h*b.Dot(f) - a.Dot(f)*j;

		float s = l*l - 4*m*k;

		if (Abs(m) < 1e-5f || Abs(s) < 1e-5f)
		{
			// The equation is linear instead.

			float v = -k / l;
			float t = -(g + h*v) / (i + j*v);
			float u = -(c.Dot(e) + c.Dot(f)*v) / (d.Dot(e) + d.Dot(f)*v);
			int nSolutions = 0;
			// If we happened to divide by zero above, the following checks handle them.
			if (v >= -eps && t >= -eps && u >= -eps && v <= 1.f + eps && t <= 1.f + eps && u <= 1.f + eps)
			{
				n1[0] = (a + b*t).Normalized();
				n2[0] = (c + d*u).Normalized();
				n3[0] = (e + f*v).Normalized();
				if (Abs(n1[0].Dot(n2[0])) < angleEps
					&& Abs(n1[0].Dot(n3[0])) < angleEps
					&& Abs(n2[0].Dot(n3[0])) < angleEps)
					return 1;
				else
					return 0;
			}
			return nSolutions;
		}

		if (s < 0.f)
			return 0; // Discriminant negative, no solutions for v.

		float sgnL = l < 0 ? -1.f : 1.f;
		float V1 = -(l + sgnL*Sqrt(s))/ (2.f*m);
		float V2 = k / (m*V1);

		float T1 = -(g + h*V1) / (i + j*V1);
		float T2 = -(g + h*V2) / (i + j*V2);

		float U1 = -(c.Dot(e) + c.Dot(f)*V1) / (d.Dot(e) + d.Dot(f)*V1);
		float U2 = -(c.Dot(e) + c.Dot(f)*V2) / (d.Dot(e) + d.Dot(f)*V2);

		int nSolutions = 0;
		if (V1 >= -eps && T1 >= -eps && U1 >= -eps && V1 <= 1.f + eps && T1 <= 1.f + eps && U1 <= 1.f + eps)
		{
			n1[nSolutions] = (a + b*T1).Normalized();
			n2[nSolutions] = (c + d*U1).Normalized();
			n3[nSolutions] = (e + f*V1).Normalized();

			if (Abs(n1[nSolutions].Dot(n2[nSolutions])) < angleEps
				&& Abs(n1[nSolutions].Dot(n3[nSolutions])) < angleEps
				&& Abs(n2[nSolutions].Dot(n3[nSolutions])) < angleEps)
				++nSolutions;
		}
		if (V2 >= -eps && T2 >= -eps && U2 >= -eps && V2 <= 1.f + eps && T2 <= 1.f + eps && U2 <= 1.f + eps)
		{
			n1[nSolutions] = (a + b*T2).Normalized();
			n2[nSolutions] = (c + d*U2).Normalized();
			n3[nSolutions] = (e + f*V2).Normalized();
			if (Abs(n1[nSolutions].Dot(n2[nSolutions])) < angleEps
				&& Abs(n1[nSolutions].Dot(n3[nSolutions])) < angleEps
				&& Abs(n2[nSolutions].Dot(n3[nSolutions])) < angleEps)
				++nSolutions;
		}
		if (s < 1e-4f && nSolutions == 2)
			 nSolutions = 1;

		return nSolutions;
	}
}

// A heuristic(?) that checks of the face normals of two edges are not suitably oriented.
// This is used to skip certain configurations.
static bool AreEdgesBad(const vec &f1a, const vec &f1b, const vec &f2a, const vec &f2b)
{
	MARK_UNUSED(f1a);
	MARK_UNUSED(f1b);
	MARK_UNUSED(f2a);
	MARK_UNUSED(f2b);
	return false;
	// Currently disabled. It's not completely certain if there's a form of this heuristic that
	// might be perfect, needs more tweaking.
#if 0
	float a1 = Abs(f1a.Dot(f1b));
	float a2 = Abs(f2a.Dot(f2b));
	float b1 = Abs(f1a.Dot(f2b));
	float b2 = Abs(f2a.Dot(f1b));
	const float limitEpsilon = 1e-4f;

	if ((a1 > 1.f - limitEpsilon && a2 < limitEpsilon) || (a2 > 1.f - limitEpsilon && a1 < limitEpsilon))
		return true;
	if ((b1 > 1.f - limitEpsilon && b2 < limitEpsilon) || (b2 > 1.f - limitEpsilon && b1 < limitEpsilon))
		return true;

	if ((a1 > 1.f - limitEpsilon || a1 < limitEpsilon) && (a2 > 1.f - limitEpsilon || a2 < limitEpsilon))
		return true;
	if ((b1 > 1.f - limitEpsilon || b1 < limitEpsilon) && (b2 > 1.f - limitEpsilon || b2 < limitEpsilon))
		return true;

	return false;
#endif
}

static bool AreEdgesCompatibleForOBB(const vec &f1a, const vec &f1b, const vec &f2a, const vec &f2b)
{
	const vec f1a_f1b = f1a-f1b;
	const vec f2a_f2b = f2a-f2b;
	float a = f1b.Dot(f2b);
	float b = (f1a_f1b).Dot(f2b);
	float c = (f2a_f2b).Dot(f1b);
	float d = (f1a_f1b).Dot(f2a_f2b);

	/*
		n1 = f1a*t + f1b*(1-t) = f1b + (f1a-f1b)*t
		n2 = f2a*u + f2b*(1-u) = f2b + (f2a-f2b)*u

		n1.n2 = 0
			f1b.f2b + t*(f1a-f1b).f2b + u*(f2a-f2b).f1b + t*u*(f1a-f1b).(f2a-f2b) = 0
			a + t*b + u*c + t*u*d = 0

		Does this equation have a solution within t & u \in [0,1]?

		// The function f(t,u) = a + t*b + u*c + t*u*d is continuous and bilinear
		// with respect to t and u, so test the four corners to get the minimum
		// and maximum of the function. If minimum <= 0 and
		// maximum >= 0, we know it must have a zero inside t,u \in [0,1].

		t=0: f(t,u)=a+uc   => min: a, max: a+c
		u=0: f(t,u)=a+tb   => min: a, max: a+b
		t=1: f(t,u)=a+uc + b+ud  => min: a+b, max: a+b+c+d
		u=1: f(t,u)=a+tb + c+td  => min: a+c, max: a+b+c+d
	*/
	float ab = a+b;
	float ac = a+c;
	float abcd = ab+c+d;
	float minVal = Min(a, ab, ac, abcd);
	float maxVal = Max(a, ab, ac, abcd);
	return minVal <= 0.f && maxVal >= 0.f;
}

// Enable this to add extra runtime checks to sanity test that the generated OBB is actually valid.
//#define OBB_ASSERT_VALIDITY

// Enable this to add internal debug prints for tracking internal behavior.
//#define OBB_DEBUG_PRINT

// Enable this to print out detailed profiling info.
//#define ENABLE_TIMING

#ifdef ENABLE_TIMING
#define TIMING_TICK(...) __VA_ARGS__
#define TIMING LOGI
#else
#define TIMING(...) ((void)0)
#define TIMING_TICK(...) ((void)0)
#endif

namespace
{
	struct hash_edge
	{
		size_t operator()(const std::pair<int, int> &e) const
		{
			return (e.first << 16) ^ e.second;
		}
	};
}

bool AreCompatibleOpposingEdges(const vec &f1a, const vec &f1b, const vec &f2a, const vec &f2b, vec &outN)
{
	/*
		n1 = f1a*t + f1b*(1-t)
		n2 = f2a*u + f2b*(1-u)
		n1 = -c*n2, where c > 0
		f1a*t + f1b*(1-t) = -c*f2a*u - c*f2b*(1-u)
		f1a*t - f1b*t + cu*f2a + c*f2b - cu*f2b = -f1b
		c*f2b + t*(f1a-f1b) + cu*(f2a-f2b) = -f1b

		M * v = -f1b, where

		M = [ f2b, (f1a-f1b), (f2a-f2b) ] column vectors
		v = [c, t, cu]
	*/

	const float tooCloseToFaceEpsilon = 1e-4f;

	float3x3 A;
	A.SetCol(0, f2b.xyz()); // c
	A.SetCol(1, (f1a - f1b).xyz()); // t
	A.SetCol(2, (f2a - f2b).xyz()); // r = c*u
	float3 x;
	bool success = A.SolveAxb(-f1b.xyz(), x);
	float c = x[0];
	float t = x[1];
	float cu = x[2];
	if (!success || c <= 0.f || t < 0.f || t > 1.f)
		return false;
	float u = cu / c;
	if (t < tooCloseToFaceEpsilon || t > 1.f - tooCloseToFaceEpsilon
		|| u < tooCloseToFaceEpsilon || u > 1.f - tooCloseToFaceEpsilon)
		return false;
	if (cu < 0.f || cu > c)
		return false;
	outN = f1b + (f1a-f1b)*t;
	return true;
}

bool SortedArrayContains(const std::vector<int> &arr, int i)
{
	size_t left = 0;
	size_t right = arr.size() - 1;
	if (arr[left] == i || arr[right] == i)
		return true;
	if (arr[left] > i || arr[right] < i)
		return false;

	while(left < right)
	{
		size_t middle = (left + right + 1) >> 1;
		if (arr[middle] < i)
			left = i;
		else if (arr[middle] > i)
			right = i;
		else
			return true;
	}
	return false;
}

OBB OBB::FixedOrientationEnclosingOBB(const vec *pointArray, int numPoints, const vec &dir0, const vec &dir1)
{
	assume(dir0.IsNormalized());
	assume(dir1.IsNormalized());
	assume(dir0.IsPerpendicular(dir1));

	int d0, d1;
	float mind0, maxd0, mind1, maxd1, mind2, maxd2;
	OBB::ExtremePointsAlongDirection(dir0, pointArray, numPoints, d0, d1, mind0, maxd0);
	OBB::ExtremePointsAlongDirection(dir1, pointArray, numPoints, d0, d1, mind1, maxd1);
	vec edgeC = dir0.Cross(dir1).Normalized();
	OBB::ExtremePointsAlongDirection(edgeC, pointArray, numPoints, d0, d1, mind2, maxd2);
	float rd0 = (maxd0 - mind0) * 0.5f;
	float rd1 = (maxd1 - mind1) * 0.5f;
	float rd2 = (maxd2 - mind2) * 0.5f;
	OBB minOBB;
	minOBB.pos = (mind0 + rd0) * dir0 + (mind1 + rd1) * dir1 + (mind2 + rd2) * edgeC;
#ifdef MATH_VEC_IS_FLOAT4
	minOBB.pos.w = 1.f;
#endif
	minOBB.axis[0] = dir0;
	minOBB.axis[1] = dir1;
	minOBB.axis[2] = edgeC;
	minOBB.r = DIR_VEC(rd0, rd1, rd2);

	return minOBB;
}

vec OBB::Size() const
{
	return r * 2.f;
}

vec OBB::HalfSize() const
{
	return r;
}

vec OBB::Diagonal() const
{
	return 2.f * HalfDiagonal();
}

vec OBB::HalfDiagonal() const
{
	return axis[0] * r[0] + axis[1] * r[1] + axis[2] * r[2];
}

float3x4 OBB::WorldToLocal() const
{
	float3x4 m = LocalToWorld();
	m.InverseOrthonormal();
	return m;
}

float3x4 OBB::LocalToWorld() const
{
	// To produce a normalized local->world matrix, do the following.
	/*
	float3x4 m;
	vec x = axis[0] * r.x;
	vec y = axis[1] * r.y;
	vec z = axis[2] * r.z;
	m.SetCol(0, 2.f * x);
	m.SetCol(1, 2.f * y);
	m.SetCol(2, 2.f * z);
	m.SetCol(3, pos - x - y - z);
	return m;
	*/

	assume2(axis[0].IsNormalized(), axis[0], axis[0].LengthSq());
	assume2(axis[1].IsNormalized(), axis[1], axis[1].LengthSq());
	assume2(axis[2].IsNormalized(), axis[2], axis[2].LengthSq());
	float3x4 m; ///\todo sse-matrix
	m.SetCol(0, axis[0].ptr());
	m.SetCol(1, axis[1].ptr());
	m.SetCol(2, axis[2].ptr());
	vec p = pos - axis[0] * r.x - axis[1] * r.y - axis[2] * r.z;
	m.SetCol(3, p.ptr());
	assume1(m.Row3(0).IsNormalized(), m.Row3(0));
	assume1(m.Row3(1).IsNormalized(), m.Row3(1));
	assume1(m.Row3(2).IsNormalized(), m.Row3(2));
	assume3(m.Col(0).IsPerpendicular(m.Col(1)), m.Col(0), m.Col(1), m.Col(0).Dot(m.Col(1)));
	assume3(m.Col(0).IsPerpendicular(m.Col(2)), m.Col(0), m.Col(2), m.Col(0).Dot(m.Col(2)));
	assume3(m.Col(1).IsPerpendicular(m.Col(2)), m.Col(1), m.Col(2), m.Col(1).Dot(m.Col(2)));
	return m;
}

/// The implementation of this function is from Christer Ericson's Real-Time Collision Detection, p.133.
vec OBB::ClosestPoint(const vec &targetPoint) const
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SIMD)
	// Best: 8.833 nsecs / 24 ticks, Avg: 9.044 nsecs, Worst: 9.217 nsecs
	simd4f d = sub_ps(targetPoint.v, pos.v);
	simd4f x = xxxx_ps(r.v);
	simd4f closestPoint = pos.v;
	closestPoint = madd_ps(max_ps(min_ps(dot4_ps(d, axis[0].v), x), neg_ps(x)), axis[0].v, closestPoint);
	simd4f y = yyyy_ps(r.v);
	closestPoint = madd_ps(max_ps(min_ps(dot4_ps(d, axis[1].v), y), neg_ps(y)), axis[1].v, closestPoint);
	simd4f z = zzzz_ps(r.v);
	closestPoint = madd_ps(max_ps(min_ps(dot4_ps(d, axis[2].v), z), neg_ps(z)), axis[2].v, closestPoint);
	return closestPoint;
#else
	// Best: 33.412 nsecs / 89.952 ticks, Avg: 33.804 nsecs, Worst: 34.180 nsecs
	vec d = targetPoint - pos;
	vec closestPoint = pos; // Start at the center point of the OBB.
	for(int i = 0; i < 3; ++i) // Project the target onto the OBB axes and walk towards that point.
		closestPoint += Clamp(Dot(d, axis[i]), -r[i], r[i]) * axis[i];

	return closestPoint;
#endif
}

float OBB::Volume() const
{
	vec size = Size();
	return size.x*size.y*size.z;
}

float OBB::SurfaceArea() const
{
	const vec size = Size();
	return 2.f * (size.x*size.y + size.x*size.z + size.y*size.z);
}

void OBB::Translate(const vec &offset)
{
	pos += offset;
}

void OBB::Scale(const vec &centerPoint, float scaleFactor)
{
	return Scale(centerPoint, DIR_VEC_SCALAR(scaleFactor));
}

void OBB::Scale(const vec &centerPoint, const vec &scaleFactor)
{
	///@bug This scales in global axes, not local axes.
	float3x4 transform = float3x4::Scale(scaleFactor, centerPoint);
	Transform(transform);
}

template<typename Matrix>
void OBBTransform(OBB &o, const Matrix &transform)
{
	o.pos = transform.MulPos(o.pos);
	o.axis[0] = transform.MulDir(o.r.x * o.axis[0]);
	o.axis[1] = transform.MulDir(o.r.y * o.axis[1]);
	o.axis[2] = transform.MulDir(o.r.z * o.axis[2]);
	o.r.x = o.axis[0].Normalize();
	o.r.y = o.axis[1].Normalize();
	o.r.z = o.axis[2].Normalize();
}

void OBB::Transform(const float3x3 &transform)
{
	assume(transform.IsColOrthogonal());
	OBBTransform(*this, transform);
}

void OBB::Transform(const float3x4 &transform)
{
	assume(transform.IsColOrthogonal());
	OBBTransform(*this, transform);
}

void OBB::Transform(const float4x4 &transform)
{
	assume(transform.IsColOrthogonal3());
	OBBTransform(*this, transform);
}

void OBB::Transform(const Quat &transform)
{
	OBBTransform(*this, transform.ToFloat3x3());
}

float OBB::Distance(const vec &point) const
{
	///@todo This code can be optimized a bit. See Christer Ericson's Real-Time Collision Detection,
	/// p.134.
	vec closestPoint = ClosestPoint(point);
	return point.Distance(closestPoint);
}

bool OBB::Contains(const vec &point) const
{
#if defined(MATH_SSE) && defined(MATH_AUTOMATIC_SSE)
// Best: 9.985 nsecs / 26.816 ticks, Avg: 10.112 nsecs, Worst: 11.137 nsecs
	simd4f pt = sub_ps(point.v, pos.v);
	simd4f s1 = mul_ps(pt, axis[0].v);
	simd4f s2 = mul_ps(pt, axis[1].v);
	simd4f s3 = mul_ps(pt, axis[2].v);
	s1 = abs_ps(sum_xyzw_ps(s1));
	s2 = abs_ps(sum_xyzw_ps(s2));
	s3 = abs_ps(sum_xyzw_ps(s3));

	s1 = _mm_sub_ss(s1, r.v);
	s2 = _mm_sub_ss(s2, yyyy_ps(r.v));
	simd4f s12 = _mm_max_ss(s1, s2);
	s3 = _mm_sub_ss(s3, zzzz_ps(r.v));
	s3 = _mm_max_ss(s12, s3);
	return _mm_cvtss_f32(s3) <= 0.f; // Note: This might be micro-optimized further out by switching to a signature "float OBB::SignedDistance(point)" instead.
#else
// Best: 14.978 nsecs / 39.944 ticks, Avg: 15.350 nsecs, Worst: 39.941 nsecs
	vec pt = point - pos;
	return Abs(Dot(pt, axis[0])) <= r[0] &&
	       Abs(Dot(pt, axis[1])) <= r[1] &&
	       Abs(Dot(pt, axis[2])) <= r[2];
#endif
}

bool OBB::Contains(const LineSegment &lineSegment) const
{
	return Contains(lineSegment.a) && Contains(lineSegment.b);
}

bool OBB::Contains(const AABB &aabb) const
{
	// Since both AABB and OBB are convex objects, this OBB contains the AABB
	// if and only if it contains all its corner points.
	for(int i = 0; i < 8; ++i)
	if (!Contains(aabb.CornerPoint(i)))
			return false;

	return true;
}

bool OBB::Contains(const OBB &obb) const
{
	for(int i = 0; i < 8; ++i)
		if (!Contains(obb.CornerPoint(i)))
			return false;

	return true;
}

bool OBB::Contains(const Triangle &triangle) const
{
	return Contains(triangle.a) && Contains(triangle.b) && Contains(triangle.c);
}

bool OBB::Contains(const Polygon &polygon) const
{
	for(int i = 0; i < polygon.NumVertices(); ++i)
		if (!Contains(polygon.Vertex(i)))
			return false;
	return true;
}

bool OBB::Intersects(const AABB &aabb) const
{
	return Intersects(OBB(aabb));
}

void OBB::Enclose(const vec &point)
{
	vec p = point - pos;
	for(int i = 0; i < 3; ++i)
	{
		assume2(EqualAbs(axis[i].Length(), 1.f), axis[i], axis[i].Length());
		float dist = p.Dot(axis[i]);
		float distanceFromOBB = Abs(dist) - r[i];
		if (distanceFromOBB > 0.f)
		{
			r[i] += distanceFromOBB * 0.5f;
			if (dist > 0.f) ///\todo Optimize out this comparison!
				pos += axis[i] * distanceFromOBB * 0.5f;
			else
				pos -= axis[i] * distanceFromOBB * 0.5f;

			p = point-pos; ///\todo Can we omit this? (redundant since axis[i] are orthonormal?)

			mathassert(EqualAbs(Abs(p.Dot(axis[i])), r[i], 1e-1f));
		}
	}
	// Should now contain the point.
	assume2(Distance(point) <= 1e-3f, point, Distance(point));
}

void OBB::Triangulate(int x, int y, int z, vec *outPos, vec *outNormal, float2 *outUV, bool ccwIsFrontFacing) const
{
	AABB aabb(POINT_VEC_SCALAR(0), r*2.f);
	aabb.Triangulate(x, y, z, outPos, outNormal, outUV, ccwIsFrontFacing);
	float3x4 localToWorld = LocalToWorld();
	assume(localToWorld.HasUnitaryScale()); // Transforming of normals will fail otherwise.
	localToWorld.BatchTransformPos(outPos, NumVerticesInTriangulation(x,y,z), sizeof(vec));
	localToWorld.BatchTransformDir(outNormal, NumVerticesInTriangulation(x,y,z), sizeof(vec));
}

void OBB::ToEdgeList(vec *outPos) const
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

bool OBB::Intersects(const OBB &b, float epsilon) const
{
	assume(pos.IsFinite());
	assume(b.pos.IsFinite());
	assume(vec::AreOrthogonal(axis[0], axis[1], axis[2]));
	assume(vec::AreOrthogonal(b.axis[0], b.axis[1], b.axis[2]));

#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SSE)
	// SSE4.1:
	// Benchmark 'OBBIntersectsOBB_Random': OBB::Intersects(OBB) Random
	//    Best: 23.913 nsecs / 40.645 ticks, Avg: 26.490 nsecs, Worst: 43.729 nsecs
	// Benchmark 'OBBIntersectsOBB_Positive': OBB::Intersects(OBB) Positive
	//    Best: 42.585 nsecs / 72.413 ticks, Avg: 44.373 nsecs, Worst: 70.774 nsecs

	simd4f bLocalToWorld[3];
	mat3x4_transpose((const simd4f*)b.axis, bLocalToWorld);

	// Generate a rotation matrix that transforms from world space to this OBB's coordinate space.
	simd4f R[3];
	mat3x4_mul_sse(R, (const simd4f*)axis, bLocalToWorld);

	// Express the translation vector in a's coordinate frame.
	simd4f t = mat3x4_mul_sse((const simd4f*)axis, sub_ps(b.pos, pos));

	// This trashes the w component, which should technically be zero, but this does not matter since
	// AbsR will only be used with direction vectors.
	const vec epsilonxyz = set1_ps(epsilon);
	simd4f AbsR[3];
	AbsR[0] = add_ps(abs_ps(R[0]), epsilonxyz);
	AbsR[1] = add_ps(abs_ps(R[1]), epsilonxyz);
	AbsR[2] = add_ps(abs_ps(R[2]), epsilonxyz);

	// Test the three major axes of this OBB.
	simd4f res = cmpgt_ps(abs_ps(t), add_ps(r, mat3x4_mul_sse(AbsR, b.r)));
	if (!allzero_ps(res)) return false;

	// Test the three major axes of the OBB b.
	simd4f transpR[3];
	mat3x4_transpose(R, transpR);
	vec l = abs_ps(mat3x4_mul_sse(transpR, t));
	simd4f transpAbsR[3];
	mat3x4_transpose(AbsR, transpAbsR);
	vec s = mat3x4_mul_sse(transpAbsR, r);
	res = cmpgt_ps(l, add_ps(s, b.r));
	if (!allzero_ps(res)) return false;

	// Test the 9 different cross-axes.

#define xxxw_ps(x) shuffle1_ps((x), _MM_SHUFFLE(3,0,0,0))
#define yyyw_ps(x) shuffle1_ps((x), _MM_SHUFFLE(3,1,1,1))
#define zzzw_ps(x) shuffle1_ps((x), _MM_SHUFFLE(3,2,2,2))
#define yxxw_ps(x) shuffle1_ps((x), _MM_SHUFFLE(3,0,0,1))
#define zzyw_ps(x) shuffle1_ps((x), _MM_SHUFFLE(3,1,2,2))

	// A.x <cross> B.x
	// A.x <cross> B.y
	// A.x <cross> B.z
	simd4f ra = madd_ps(yyyw_ps(r), AbsR[2], mul_ps(zzzw_ps(r), AbsR[1]));
	simd4f rb = madd_ps(yxxw_ps(b.r), zzyw_ps(AbsR[0]), mul_ps(zzyw_ps(b.r), yxxw_ps(AbsR[0])));
	simd4f lhs = msub_ps(zzzw_ps(t), R[1], mul_ps(yyyw_ps(t), R[2]));
	res = cmpgt_ps(abs_ps(lhs), add_ps(ra, rb));
	if (!allzero_ps(res)) return false;

	// A.y <cross> B.x
	// A.y <cross> B.y
	// A.y <cross> B.z
	ra = madd_ps(xxxw_ps(r), AbsR[2], mul_ps(zzzw_ps(r), AbsR[0]));
	rb = madd_ps(yxxw_ps(b.r), zzyw_ps(AbsR[1]), mul_ps(zzyw_ps(b.r), yxxw_ps(AbsR[1])));
	lhs = msub_ps(xxxw_ps(t), R[2], mul_ps(zzzw_ps(t), R[0]));
	res = cmpgt_ps(abs_ps(lhs), add_ps(ra, rb));
	if (!allzero_ps(res)) return false;

	// A.z <cross> B.x
	// A.z <cross> B.y
	// A.z <cross> B.z
	ra = madd_ps(xxxw_ps(r), AbsR[1], mul_ps(yyyw_ps(r), AbsR[0]));
	rb = madd_ps(yxxw_ps(b.r), zzyw_ps(AbsR[2]), mul_ps(zzyw_ps(b.r), yxxw_ps(AbsR[2])));
	lhs = msub_ps(yyyw_ps(t), R[0], mul_ps(xxxw_ps(t), R[1]));
	res = cmpgt_ps(abs_ps(lhs), add_ps(ra, rb));
	return allzero_ps(res) != 0;

#else
	// Benchmark 'OBBIntersectsOBB_Random': OBB::Intersects(OBB) Random
	//    Best: 100.830 nsecs / 171.37 ticks, Avg: 110.533 nsecs, Worst: 155.582 nsecs
	// Benchmark 'OBBIntersectsOBB_Positive': OBB::Intersects(OBB) Positive
	//    Best: 95.771 nsecs / 162.739 ticks, Avg: 107.935 nsecs, Worst: 173.110 nsecs

	// Generate a rotation matrix that transforms from world space to this OBB's coordinate space.
	float3x3 R;
	for(int i = 0; i < 3; ++i)
		for(int j = 0; j < 3; ++j)
			R[i][j] = Dot(axis[i], b.axis[j]);

	vec t = b.pos - pos;
	// Express the translation vector in a's coordinate frame.
	t = DIR_VEC(Dot(t, axis[0]), Dot(t, axis[1]), Dot(t, axis[2]));

	float3x3 AbsR;
	for(int i = 0; i < 3; ++i)
		for(int j = 0; j < 3; ++j)
			AbsR[i][j] = Abs(R[i][j]) + epsilon;

	// Test the three major axes of this OBB.
	for(int i = 0; i < 3; ++i)
	{
		float ra = r[i];
		float rb = b.r.x * AbsR[i][0] + b.r.y * AbsR[i][1] + b.r.z * AbsR[i][2];
		if (Abs(t[i]) > ra + rb)
			return false;
	}

	// Test the three major axes of the OBB b.
	for(int i = 0; i < 3; ++i)
	{
		float ra = r[0] * AbsR[0][i] + r[1] * AbsR[1][i] + r[2] * AbsR[2][i];
		float rb = b.r[i];
		if (Abs(t.x * R[0][i] + t.y * R[1][i] + t.z * R[2][i]) > ra + rb)
			return false;
	}

	// Test the 9 different cross-axes.

	// A.x <cross> B.x
	float ra = r.y * AbsR[2][0] + r.z * AbsR[1][0];
	float rb = b.r.y * AbsR[0][2] + b.r.z * AbsR[0][1];
	if (Abs(t.z * R[1][0] - t.y * R[2][0]) > ra + rb)
		return false;

	// A.x < cross> B.y
	ra = r.y * AbsR[2][1] + r.z * AbsR[1][1];
	rb = b.r.x * AbsR[0][2] + b.r.z * AbsR[0][0];
	if (Abs(t.z * R[1][1] - t.y * R[2][1]) > ra + rb)
		return false;

	// A.x <cross> B.z
	ra = r.y * AbsR[2][2] + r.z * AbsR[1][2];
	rb = b.r.x * AbsR[0][1] + b.r.y * AbsR[0][0];
	if (Abs(t.z * R[1][2] - t.y * R[2][2]) > ra + rb)
		return false;

	// A.y <cross> B.x
	ra = r.x * AbsR[2][0] + r.z * AbsR[0][0];
	rb = b.r.y * AbsR[1][2] + b.r.z * AbsR[1][1];
	if (Abs(t.x * R[2][0] - t.z * R[0][0]) > ra + rb)
		return false;

	// A.y <cross> B.y
	ra = r.x * AbsR[2][1] + r.z * AbsR[0][1];
	rb = b.r.x * AbsR[1][2] + b.r.z * AbsR[1][0];
	if (Abs(t.x * R[2][1] - t.z * R[0][1]) > ra + rb)
		return false;

	// A.y <cross> B.z
	ra = r.x * AbsR[2][2] + r.z * AbsR[0][2];
	rb = b.r.x * AbsR[1][1] + b.r.y * AbsR[1][0];
	if (Abs(t.x * R[2][2] - t.z * R[0][2]) > ra + rb)
		return false;

	// A.z <cross> B.x
	ra = r.x * AbsR[1][0] + r.y * AbsR[0][0];
	rb = b.r.y * AbsR[2][2] + b.r.z * AbsR[2][1];
	if (Abs(t.y * R[0][0] - t.x * R[1][0]) > ra + rb)
		return false;

	// A.z <cross> B.y
	ra = r.x * AbsR[1][1] + r.y * AbsR[0][1];
	rb = b.r.x * AbsR[2][2] + b.r.z * AbsR[2][0];
	if (Abs(t.y * R[0][1] - t.x * R[1][1]) > ra + rb)
		return false;

	// A.z <cross> B.z
	ra = r.x * AbsR[1][2] + r.y * AbsR[0][2];
	rb = b.r.x * AbsR[2][1] + b.r.y * AbsR[2][0];
	if (Abs(t.y * R[0][2] - t.x * R[1][2]) > ra + rb)
		return false;

	// No separating axis exists, so the two OBB don't intersect.
	return true;
#endif
}

/// The implementation of OBB-Plane intersection test follows Christer Ericson's Real-Time Collision Detection, p. 163. [groupSyntax]
bool OBB::Intersects(const Plane &p) const
{
	// Compute the projection interval radius of this OBB onto L(t) = this->pos + x * p.normal;
	float t = r[0] * Abs(Dot(p.normal, axis[0])) +
			  r[1] * Abs(Dot(p.normal, axis[1])) +
			  r[2] * Abs(Dot(p.normal, axis[2]));
	// Compute the distance of this OBB center from the plane.
	float s = Dot(p.normal, pos) - p.d;
	return Abs(s) <= t;
}

bool OBB::Intersects(const Ray &ray) const
{
	AABB aabb(POINT_VEC_SCALAR(0.f), Size());
	Ray localRay = WorldToLocal() * ray;
	return aabb.Intersects(localRay);
}

bool OBB::Intersects(const Ray &ray, float &dNear, float &dFar) const
{
	AABB aabb(POINT_VEC_SCALAR(0.f), Size());
	Ray localRay = WorldToLocal() * ray;
	return aabb.Intersects(localRay, dNear, dFar);
}

bool OBB::Intersects(const Line &line) const
{
	AABB aabb(POINT_VEC_SCALAR(0.f), Size());
	Line l = WorldToLocal() * line;
	return aabb.Intersects(l);
}

bool OBB::Intersects(const Line &line, float &dNear, float &dFar) const
{
	AABB aabb(POINT_VEC_SCALAR(0.f), Size());
	Line l = WorldToLocal() * line;
	return aabb.Intersects(l, dNear, dFar);
}

bool OBB::Intersects(const LineSegment &lineSegment) const
{
	AABB aabb(POINT_VEC_SCALAR(0.f), Size());
	LineSegment l = WorldToLocal() * lineSegment;
	return aabb.Intersects(l);
}

bool OBB::Intersects(const LineSegment &lineSegment, float &dNear, float &dFar) const
{
	AABB aabb(POINT_VEC_SCALAR(0.f), Size());
	LineSegment l = WorldToLocal() * lineSegment;
	return aabb.Intersects(l, dNear, dFar);
}

bool OBB::Intersects(const Triangle &triangle) const
{
	AABB aabb(POINT_VEC_SCALAR(0.f), Size());
	Triangle t = WorldToLocal() * triangle;
	return t.Intersects(aabb);
}

bool OBB::Intersects(const Polygon &polygon) const
{
	return polygon.Intersects(*this);
}

#if defined(MATH_ENABLE_STL_SUPPORT) || defined(MATH_CONTAINERLIB_SUPPORT)
StringT OBB::ToString() const
{
	char str[256];
	sprintf(str, "OBB(Pos:(%.2f, %.2f, %.2f) Halfsize:(%.2f, %.2f, %.2f) X:(%.2f, %.2f, %.2f) Y:(%.2f, %.2f, %.2f) Z:(%.2f, %.2f, %.2f))",
		pos.x, pos.y, pos.z, r.x, r.y, r.z, axis[0].x, axis[0].y, axis[0].z, axis[1].x, axis[1].y, axis[1].z, axis[2].x, axis[2].y, axis[2].z);
	return str;
}

StringT OBB::SerializeToString() const
{
	StringT s = pos.xyz().SerializeToString() + " "
	              + r.xyz().SerializeToString() + " "
	              + axis[0].xyz().SerializeToString() + " "
	              + axis[1].xyz().SerializeToString() + " "
	              + axis[2].xyz().SerializeToString();
	return s;
}

StringT OBB::SerializeToCodeString() const
{
	return "OBB(" + pos.SerializeToCodeString() + ","
	              + r.SerializeToCodeString() + ","
	              + axis[0].SerializeToCodeString() + ","
	              + axis[1].SerializeToCodeString() + ","
	              + axis[2].SerializeToCodeString() + ")";
}
#endif

#if defined(MATH_ENABLE_STL_SUPPORT)

std::ostream &operator <<(std::ostream &o, const OBB &obb)
{
	o << obb.ToString();
	return o;
}

#endif

OBB OBB::FromString(const char *str, const char **outEndStr)
{
	assume(str);
	if (!str)
		return OBB(vec::nan, vec::nan, vec::nan, vec::nan, vec::nan);
	OBB o;
	MATH_SKIP_WORD(str, "OBB(");
	MATH_SKIP_WORD(str, "Pos:(");
	o.pos = PointVecFromString(str, &str);
	MATH_SKIP_WORD(str, " Halfsize:(");
	o.r = DirVecFromString(str, &str);
	MATH_SKIP_WORD(str, " X:(");
	o.axis[0] = DirVecFromString(str, &str);
	MATH_SKIP_WORD(str, " Y:(");
	o.axis[1] = DirVecFromString(str, &str);
	MATH_SKIP_WORD(str, " Z:(");
	o.axis[2] = DirVecFromString(str, &str);
	if (outEndStr)
		*outEndStr = str;
	return o;
}

#ifdef MATH_GRAPHICSENGINE_INTEROP
void OBB::Triangulate(VertexBuffer &vb, int x, int y, int z, bool ccwIsFrontFacing) const
{
	Array<vec> position;
	Array<vec> normal;
	Array<float2> uv;
	int numVertices = (x*y+y*z+x*z)*2*6;
	position.Resize_unspecified(numVertices);
	normal.Resize_unspecified(numVertices);
	uv.Resize_unspecified(numVertices);
	Triangulate(x,y,z, &position[0], &normal[0], &uv[0], ccwIsFrontFacing);
	int startIndex = vb.AppendVertices(numVertices);
	for(int i = 0; i < (int)position.size(); ++i)
	{
		vb.Set(startIndex+i, VDPosition, POINT_TO_FLOAT4(position[i]));
		if (vb.Declaration()->TypeOffset(VDNormal) >= 0)
			vb.Set(startIndex+i, VDNormal, DIR_TO_FLOAT4(normal[i]));
		if (vb.Declaration()->TypeOffset(VDUV) >= 0)
			vb.SetFloat2(startIndex+i, VDUV, 0, uv[i]);
	}
}

void OBB::ToLineList(VertexBuffer &vb) const
{
	if (vb.Declaration()->HasType(VDNormal))
	{
		// FacePlane() returns plane normals in order -x, +x, -y, +y, -z, +z
		// Cornerpoint() returns points in order
		// 0: ---, 1: --+, 2: -+-, 3: -++, 4: +--, 5: +-+, 6: ++-, 7: +++. (corresponding the XYZ axis directions).

		// Vertices corresponding to these six faces:
		int verts[6][4] =
		{
			{ 0, 1, 3, 2 }, // -x
			{ 7, 6, 4, 5 }, // +x
			{ 0, 1, 5, 4 }, // -y
			{ 7, 6, 2, 3 }, // +y
			{ 0, 2, 6, 4 }, // -z
			{ 7, 5, 1, 3 }  // +z
		};
		int si = vb.AppendVertices(2*4*6);
		for(int face = 0; face < 6; ++face)
		{
			float4 faceNormal = DIR_TO_FLOAT4(FacePlane(face).normal);
			int v0 = verts[face][3];
			for(int v1i = 0; v1i < 4; ++v1i)
			{
				int v1 = verts[face][v1i];
				vb.Set(si, VDPosition, POINT_TO_FLOAT4(CornerPoint(v0)));
				vb.Set(si++, VDNormal, faceNormal);
				vb.Set(si, VDPosition, POINT_TO_FLOAT4(CornerPoint(v1)));
				vb.Set(si++, VDNormal, faceNormal);
				v0 = v1;
			}
		}
	}
	else
	{
		Array<vec> position;
		position.Resize_unspecified(NumVerticesInEdgeList());
		ToEdgeList(&position[0]);
		int startIndex = vb.AppendVertices((int)position.size());
		for(int i = 0; i < (int)position.size(); ++i)
			vb.Set(startIndex+i, VDPosition, POINT_TO_FLOAT4(position[i]));
	}
}

#endif

OBB operator *(const float3x3 &transform, const OBB &obb)
{
	OBB o(obb);
	o.Transform(transform);
	return o;
}

OBB operator *(const float3x4 &transform, const OBB &obb)
{
	OBB o(obb);
	o.Transform(transform);
	return o;
}

OBB operator *(const float4x4 &transform, const OBB &obb)
{
	OBB o(obb);
	o.Transform(transform);
	return o;
}

OBB operator *(const Quat &transform, const OBB &obb)
{
	OBB o(obb);
	o.Transform(transform);
	return o;
}

MATH_END_NAMESPACE
