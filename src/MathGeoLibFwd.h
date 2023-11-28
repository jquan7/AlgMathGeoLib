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

/** @file MathFwd.h
	@author Jukka Jyl�nki
	@brief */
#pragma once

#include "MathBuildConfig.h"
#include "Math/MathNamespace.h"

#ifdef MATH_CONTAINERLIB_SUPPORT
#include "Memory/TypeTraits.h"
#endif

#include <stddef.h>

// Very annoying to have to do this, but <iosfwd> doesn't have a fwddecl for std::vector,
// and forward-declaring it manually is not allowed, see http://stackoverflow.com/questions/307343/forward-declare-an-stl-container
#include <vector>

#define NAMELESS_UNION_BEGIN union {
#define NAMELESS_UNION_END };

#if !defined(MATH_ENABLE_STL_SUPPORT) && !defined(assert)
#include <stdio.h>
#define assert(x) do { if (!(x)) { printf("Error: assert(%s) failed!\n", #x); } } while(0)
#endif

MATH_BEGIN_NAMESPACE

class float2;
class float3;
class float4;
class float3x3;
class float3x4;
class float4x4;
class Quat;

class TranslateOp;
class ScaleOp;

class AABB;
class Circle;
class Circle2D;
class Cone;
class Cylinder;
class Ellipsoid;
class Line;
class LineSegment;
class OBB;
class Plane;
class Polygon;
class Polynomial;
class Quat;
class TranslateOp;
class Torus;
class ScaleOp;
class Triangle;

class AABB2D;
class LineSegment2D;
class Triangle2D;

struct float4_storage;

#define IS16ALIGNED(x) ((((uintptr_t)(x)) & 0xF) == 0)
#define IS32ALIGNED(x) ((((uintptr_t)(x)) & 0x1F) == 0)
#define IS64ALIGNED(x) ((((uintptr_t)(x)) & 0x3F) == 0)

#ifdef MATH_SIMD

#ifdef MATH_AVX
#define ALIGN_MAT ALIGN32
#define MAT_ALIGNMENT 32
#define IS_MAT_ALIGNED(x) IS32ALIGNED(x)
#else
#define ALIGN_MAT ALIGN16
#define MAT_ALIGNMENT 16
#define IS_MAT_ALIGNED(x) IS16ALIGNED(x)
#endif

#define ALIGN16 __attribute__((aligned(16)))
#define ALIGN32 __attribute__((aligned(32)))
#define ALIGN64 __attribute__((aligned(64)))

#else

#define ALIGN16
#define ALIGN32
#define ALIGN64
#define ALIGN_MAT
#define IS_MAT_ALIGNED(x) true

#endif

#ifdef MATH_AUTOMATIC_SSE

#ifndef MATH_VEC_IS_FLOAT4
#define MATH_VEC_IS_FLOAT4
#endif

typedef ALIGN16 float4 vec2d;
typedef float4_storage vec2d_storage;

typedef ALIGN16 float4 vec;
typedef float4_storage vec_storage;

#else

typedef float2 vec2d;
typedef float2 vec2d_storage;

typedef float3 vec;
typedef float3 vec_storage;

#endif

template<class T, size_t Alignment>
struct AlignedAllocator;

struct Triangle_storage;
struct LineSegment_storage;

typedef std::vector<Triangle> TriangleArray;
typedef std::vector<LineSegment> LineSegmentArray;
typedef std::vector<float4> Float4Array;
#ifdef MATH_AUTOMATIC_SSE
typedef std::vector<float4> VecArray;
#endif

#if !defined(MATH_AUTOMATIC_SSE)
typedef std::vector<float3> VecArray;
#endif

MATH_END_NAMESPACE

#ifdef MATH_GRAPHICSENGINE_INTEROP
class VertexBuffer;
#endif

#ifdef MATH_ENABLE_STL_SUPPORT
#include <iosfwd>
#endif

#if defined(_M_X64) || defined(__x86_64__)
// Are we targeting a 64-bit build?
#define MATH_64BIT
#endif

#ifdef MATH_CONTAINERLIB_SUPPORT
REGISTER_POD(MATH_NS::float2);
REGISTER_POD(MATH_NS::float3);
REGISTER_POD(MATH_NS::float4);
REGISTER_POD(MATH_NS::float3x3);
REGISTER_POD(MATH_NS::float3x4);
REGISTER_POD(MATH_NS::float4x4);
REGISTER_POD(MATH_NS::Quat);
REGISTER_POD(MATH_NS::TranslateOp);
REGISTER_POD(MATH_NS::ScaleOp);
REGISTER_POD(MATH_NS::AABB);
REGISTER_POD(MATH_NS::Circle);
REGISTER_POD(MATH_NS::Cone);
REGISTER_POD(MATH_NS::Cylinder);
REGISTER_POD(MATH_NS::Ellipsoid);
REGISTER_POD(MATH_NS::Line);
REGISTER_POD(MATH_NS::LineSegment);
REGISTER_POD(MATH_NS::OBB);
REGISTER_POD(MATH_NS::Plane);
REGISTER_POD(MATH_NS::Polygon);
REGISTER_POD(MATH_NS::Polynomial);
REGISTER_POD(MATH_NS::Torus);
REGISTER_POD(MATH_NS::Triangle);
REGISTER_POD(MATH_NS::float4_storage);
REGISTER_POD(MATH_NS::Triangle_storage);
REGISTER_POD(MATH_NS::LineSegment_storage);
#endif
