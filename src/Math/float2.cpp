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

/** @file float2.cpp
	@author Jukka Jylänki
	@brief */
#include "float2.h"
#include "float2.inl"
#include "float3.h"
#include "float4.h"
#include "float3x3.h"
#include "float3x4.h"
#include "float4x4.h"
#include "MathFunc.h"
#include "assume.h"
#include <string.h>
#include <stdlib.h>
#include <locale.h>

#ifdef MATH_ENABLE_STL_SUPPORT
#include "myassert.h"
#include <iostream>
#include <utility>
#include <algorithm>
#endif

MATH_BEGIN_NAMESPACE

using namespace std;

float2::float2(float x_, float y_)
:x(x_), y(y_)
{
}

float2::float2(float scalar)
:x(scalar), y(scalar)
{
}

float2::float2(const float *data)
{
	assume(data);
	x = data[0];
	y = data[1];
}

float2 float2::Swizzled(int i, int j) const
{
	return float2(At(i), At(j));
}

float3 float2::Swizzled(int i, int j, int k) const
{
	return float3(At(i), At(j), At(k));
}

float4 float2::Swizzled(int i, int j, int k, int l) const
{
	return float4(At(i), At(j), At(k), At(l));
}

float float2::LengthSq() const
{
	return x*x + y*y;
}

float float2::Length() const
{
	return Sqrt(LengthSq());
}

void float2::SetFromPolarCoordinates(float theta, float length)
{
	float sin, cos;
	SinCos(theta, sin, cos);
	x = cos * length;
	y = sin * length;
}

float2 float2::FromPolarCoordinates(float theta, float length)
{
	float2 euclidean;
	euclidean.SetFromPolarCoordinates(theta, length);
	return euclidean;
}

float2 float2::ToPolarCoordinates() const
{
	float radius = Length();
	if (radius > 1e-4f)
		return float2(atan2(y, x), radius);
	else
		return float2::zero;
}

float float2::AimedAngle() const
{
	assume(!IsZero());
	return atan2(y, x);
}

float float2::Normalize()
{
	assume(IsFinite());
	float lengthSq = LengthSq();
	if (lengthSq > 1e-6f)
	{
		float length = Sqrt(lengthSq);
		*this *= 1.f / length;
		return length;
	}
	else
	{
		Set(1.f, 0.f); // We will always produce a normalized vector.
		return 0; // But signal failure, so user knows we have generated an arbitrary normalization.
	}
}

float2 float2::Normalized() const
{
	float2 copy = *this;
	float oldLength = copy.Normalize();
	assume(oldLength > 0.f && "float2::Normalized() failed!");
	MARK_UNUSED(oldLength);
	return copy;
}

float float2::ScaleToLength(float newLength)
{
	float length = LengthSq();
	if (length < 1e-6f)
		return 0.f;

	length = Sqrt(length);
	float scalar = newLength / length;
	x *= scalar;
	y *= scalar;
	return length;
}

float2 float2::ScaledToLength(float newLength) const
{
	assume(!IsZero());

	float2 v = *this;
	v.ScaleToLength(newLength);
	return v;
}

bool float2::IsNormalized(float epsilonSq) const
{
	return MATH_NS::Abs(LengthSq()-1.f) <= epsilonSq;
}

bool float2::IsZero(float epsilonSq) const
{
	return LengthSq() <= epsilonSq;
}

bool float2::IsFinite() const
{
	return MATH_NS::IsFinite(x) && MATH_NS::IsFinite(y);
}

bool float2::IsPerpendicular(const float2 &other, float epsilonSq) const
{
	float dot = Dot(other);
	return dot*dot <= epsilonSq * LengthSq() * other.LengthSq();
}

bool float2::Equals(const float2 &rhs, float epsilon) const
{
	return EqualAbs(x, rhs.x, epsilon) && EqualAbs(y, rhs.y, epsilon);
}

bool float2::Equals(float x_, float y_, float epsilon) const
{
	return EqualAbs(x, x_, epsilon) && EqualAbs(y, y_, epsilon);
}

#if defined(MATH_ENABLE_STL_SUPPORT)
std::string float2::ToString() const
{
	char str[256];
	sprintf(str, "(%f, %f)", x, y);
	return str;
}

std::string float2::SerializeToString() const
{
	char str[256];
	char *s = SerializeFloat(x, str); *s = ','; ++s;
	s = SerializeFloat(y, s);
	assert(s+1 - str < 256);
	MARK_UNUSED(s);
	return str;
}

std::string float2::SerializeToCodeString() const
{
	return "float2(" + SerializeToString() + ")";
}
#endif

float float2::SumOfElements() const
{
	return x + y;
}

float float2::ProductOfElements() const
{
	return x * y;
}

float float2::AverageOfElements() const
{
	return (x + y) * 0.5f;
}

float float2::MinElement() const
{
	return MATH_NS::Min(x, y);
}

int float2::MinElementIndex() const
{
	return (x <= y) ? 0 : 1;
}

float float2::MaxElement() const
{
	return  MATH_NS::Max(x, y);
}

int float2::MaxElementIndex() const
{
	return (x > y) ? 0 : 1;
}

float2 float2::Abs() const
{
	return float2(MATH_NS::Abs(x), MATH_NS::Abs(y));
}

float2 float2::Neg() const
{
	return float2(-x, -y);
}

float2 float2::Recip() const
{
	return float2(1.f/x, 1.f/y);
}

float2 float2::Min(float floor) const
{
	return float2(MATH_NS::Min(x, floor),  MATH_NS::Min(y, floor));
}

float2 float2::Min(const float2 &floor) const
{
	return float2(MATH_NS::Min(x, floor.x),  MATH_NS::Min(y, floor.y));
}

float2 float2::Max(float ceil) const
{
	return float2(MATH_NS::Max(x, ceil),  MATH_NS::Max(y, ceil));
}

float2 float2::Max(const float2 &ceil) const
{
	return float2(MATH_NS::Max(x, ceil.x),  MATH_NS::Max(y, ceil.y));
}

float2 float2::Clamp(const float2 &floor, const float2 &ceil) const
{
	return float2(MATH_NS::Clamp(x, floor.x, ceil.x),  MATH_NS::Clamp(y, floor.y, ceil.y));
}

float2 float2::Clamp(float floor, float ceil) const
{
	return float2(MATH_NS::Clamp(x, floor, ceil),  MATH_NS::Clamp(y, floor, ceil));
}

float2 float2::Clamp01() const
{
	return Clamp(0.f, 1.f);
}

float float2::DistanceSq(const float2 &rhs) const
{
	float dx = x - rhs.x;
	float dy = y - rhs.y;
	return dx*dx + dy*dy;
}

float float2::Distance(const float2 &rhs) const
{
	return Sqrt(DistanceSq(rhs));
}

float float2::Dot(const float2 &rhs) const
{
	return x * rhs.x + y * rhs.y;
}

float2 float2::Perp() const
{
	return float2(y, -x);
}

float float2::PerpDot(const float2 &rhs) const
{
	return x * rhs.y - y * rhs.x;
}

float2 float2::ProjectTo(const float2 &direction) const
{
	assume(!direction.IsZero());
	return direction * this->Dot(direction) / direction.LengthSq();
}

float2 float2::ProjectToNorm(const float2 &direction) const
{
	assume(direction.IsNormalized());
	return direction * this->Dot(direction);
}

float float2::AngleBetween(const float2 &other) const
{
	return acos(Dot(other)) / Sqrt(LengthSq() * other.LengthSq());
}

float float2::AngleBetweenNorm(const float2 &other) const
{
	assume(this->IsNormalized());
	assume(other.IsNormalized());
	return acos(Dot(other));
}

float2 float2::Lerp(const float2 &b, float t) const
{
	assume(0.f <= t && t <= 1.f);
	return (1.f - t) * *this + t * b;
}

float2 float2::Lerp(const float2 &a, const float2 &b, float t)
{
	return a.Lerp(b, t);
}

void float2::Decompose(const float2 &direction, float2 &parallel, float2 &perpendicular) const
{
	assume(direction.IsNormalized());
	parallel = this->Dot(direction) * direction;
	perpendicular = *this - parallel;
}

void float2::Orthogonalize(const float2 &a, float2 &b)
{
	assume(!a.IsZero());
	b -= a.Dot(b) / a.Length() * a;
}

bool float2::AreOrthogonal(const float2 &a, const float2 &b, float epsilon)
{
	return a.IsPerpendicular(b, epsilon);
}


void float2::Orthonormalize(float2 &a, float2 &b)
{
	assume(!a.IsZero());
	a.Normalize();
	b -= a.Dot(b) * a;
}

float2 float2::FromScalar(float scalar)
{
	return float2(scalar, scalar);
}

void float2::SetFromScalar(float scalar)
{
	x = scalar;
	y = scalar;
}

void float2::Set(float x_, float y_)
{
	x = x_;
	y = y_;
}

void float2::Rotate90CW()
{
	float oldX = x;
	x = y;
	y = -oldX;
}

float2 float2::Rotated90CW() const
{
	return float2(y, -x);
}

void float2::Rotate90CCW()
{
	float oldX = x;
	x = -y;
	y = oldX;
}

float2 float2::Rotated90CCW() const
{
	return float2(-y, x);
}

bool float2::OrientedCCW(const float2 &a, const float2 &b, const float2 &c)
{
	// Compute the determinant
	// | ax ay 1 |
	// | bx by 1 |
	// | cx cy 1 |
	// See Christer Ericson, Real-Time Collision Detection, p.32.
	return (a.x-c.x)*(b.y-c.y) - (a.y-c.y)*(b.x-c.x) >= 0.f;
}

float2 float2::operator +(const float2 &rhs) const
{
	return float2(x + rhs.x, y + rhs.y);
}

float2 float2::operator -(const float2 &rhs) const
{
	return float2(x - rhs.x, y - rhs.y);
}

float2 float2::operator -() const
{
	return float2(-x, -y);
}

float2 float2::operator *(float scalar) const
{
	return float2(x * scalar, y * scalar);
}

float2 operator *(float scalar, const float2 &rhs)
{
	return float2(scalar * rhs.x, scalar * rhs.y);
}

float2 float2::operator /(float scalar) const
{
	float invScalar = 1.f / scalar;
	return float2(x * invScalar, y * invScalar);
}

float2 &float2::operator =(const float2 &rhs)
{
	x = rhs.x;
	y = rhs.y;

	return *this;
}

float2 &float2::operator +=(const float2 &rhs)
{
	x += rhs.x;
	y += rhs.y;

	return *this;
}

float2 &float2::operator -=(const float2 &rhs)
{
	x -= rhs.x;
	y -= rhs.y;

	return *this;
}

float2 &float2::operator *=(float scalar)
{
	x *= scalar;
	y *= scalar;

	return *this;
}

float2 float2::Add(float s) const
{
	return float2(x + s, y + s);
}

float2 float2::Sub(float s) const
{
	return float2(x - s, y - s);
}

float2 float2::SubLeft(float s) const
{
	return float2(s - x, s - y);
}

float2 float2::DivLeft(float s) const
{
	return float2(s / x, s / y);
}

float2 float2::Mul(const float2 &rhs) const
{
	return float2(x * rhs.x, y * rhs.y);
}

float2 float2::Div(const float2 &rhs) const
{
	return float2(x / rhs.x, y / rhs.y);
}

float2 &float2::operator /=(float scalar)
{
	float invScalar = 1.f / scalar;
	x *= invScalar;
	y *= invScalar;

	return *this;
}

#ifdef MATH_ENABLE_STL_SUPPORT
std::ostream &operator <<(std::ostream &out, const float2 &rhs)
{
	std::string str = rhs.ToString();
	out << str;
	return out;
}
#endif

float2 Mul2D(const float3x3 &transform, const float2 &v) { return transform.Transform(v.x, v.y, 0.f).xy(); }
float2 MulPos2D(const float3x4 &transform, const float2 &v) { return transform.Transform(float4(v.x, v.y, 0.f, 1.f)).xy(); }
float2 MulPos2D(const float4x4 &transform, const float2 &v) { return transform.Transform(float4(v.x, v.y, 0.f, 1.f)).xy(); }
float2 MulDir2D(const float3x4 &transform, const float2 &v) { return transform.Transform(float4(v.x, v.y, 0.f, 0.f)).xy(); }
float2 MulDir2D(const float4x4 &transform, const float2 &v) { return transform.Transform(float4(v.x, v.y, 0.f, 0.f)).xy(); }

const float2 float2::zero = float2(0, 0);
const float2 float2::one = float2(1, 1);
const float2 float2::unitX = float2(1, 0);
const float2 float2::unitY = float2(0, 1);
const float2 float2::nan = float2(FLOAT_NAN, FLOAT_NAN);
const float2 float2::inf = float2(FLOAT_INF, FLOAT_INF);

MATH_END_NAMESPACE
