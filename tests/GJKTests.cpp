#include <stdio.h>
#include <stdlib.h>

#include "../src/MathGeoLib.h"
#include "../src/Math/myassert.h"
#include "TestRunner.h"
#include "../src/Algorithm/GJK.h"
#include "ObjectGenerators.h"

MATH_IGNORE_UNUSED_VARS_WARNING

UNIQUE_TEST(TrickyGJKSphereSphereIntersect)
{
	Sphere a = Sphere(POINT_VEC(-14.740263f, 8.2647991f, 64.282227f), 7.6029987f);
	Sphere b = Sphere(POINT_VEC(-23.840866f, 9.4233770f, 66.399742f), 1.9519407f);
	assert(GJKIntersect(a, b));
	assert(GJKIntersect(b, a));
}

UNIQUE_TEST(TrickyGJKSphereSphereIntersect2)
{
	Sphere a = Sphere(POINT_VEC(57.166256f, 99.426201f, 75.735786f), 2.3808355f);
	Sphere b = Sphere(POINT_VEC(54.087727f, 94.719139f, 75.188812f), 3.3114955f);
	assert(GJKIntersect(a, b));
	assert(GJKIntersect(b, a));
}

UNIQUE_TEST(TrickyGJKAABBAABBIntersect)
{
	AABB a(POINT_VEC_SCALAR(-10.f), POINT_VEC_SCALAR(10.f));
	AABB b(POINT_VEC_SCALAR(8.f), POINT_VEC_SCALAR(20.f));
	assert(GJKIntersect(a, b));
	assert(GJKIntersect(b, a));
}

UNIQUE_TEST(GJKAABBSphereIntersectCase)
{
	AABB a(POINT_VEC(37.1478767f,-71.9611969f,-51.1293259f),POINT_VEC(42.8975906f,-67.3180618f,-44.9161682f));
	Sphere b(POINT_VEC(41.9271927f,-71.1957016f,-56.7100677f),7.20756626f);
	assert(GJKIntersect(a, b));
}

UNIQUE_TEST(GJKAABBSphereIntersectCase2)
{
	AABB a(POINT_VEC(-6.17850494f,-1.09283221f,-85.2101898f),POINT_VEC(-3.21846271f,-0.564386666f,-84.2947693f));
	Sphere b(POINT_VEC(-2.29130507f,-1.87549007f,-83.2377472f),2.09292078f);
	assert(GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKAABBShereNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));

	AABB a = RandomAABBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Sphere b = RandomSphereInHalfspace(p, 10.f);
	assert(!GJKIntersect(a, b));
	assert(!GJKIntersect(b, a));
}

RANDOMIZED_TEST(GJKAABBAABBIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	AABB a = RandomAABBContainingPoint(pt, 10.f);
	AABB b = RandomAABBContainingPoint(pt, 10.f);
	assert(GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKAABBOBBIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	AABB a = RandomAABBContainingPoint(pt, 10.f);
	OBB b = RandomOBBContainingPoint(pt, 10.f);
	assert(GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKAABBLineSegmentIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	AABB a = RandomAABBContainingPoint(pt, 10.f);
	LineSegment b = RandomLineSegmentContainingPoint(pt);
	try
	{
		assert(GJKIntersect(a, b));
	} catch(...)
	{
		LOGI("a: %s", a.SerializeToCodeString().c_str());
		LOGI("b: %s", b.SerializeToCodeString().c_str());
		throw;
	}
}

RANDOMIZED_TEST(GJKAABBSphereIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	AABB a = RandomAABBContainingPoint(pt, 10.f);
	Sphere b = RandomSphereContainingPoint(pt, SCALE);
	assert(GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKAABBTriangleIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	AABB a = RandomAABBContainingPoint(pt, 10.f);
	Triangle b = RandomTriangleContainingPoint(pt);
	assert(GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKOBBOBBIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	OBB a = RandomOBBContainingPoint(pt, 10.f);
	OBB b = RandomOBBContainingPoint(pt, 10.f);
	assert(GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKOBBLineSegmentIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	OBB a = RandomOBBContainingPoint(pt, 10.f);
	LineSegment b = RandomLineSegmentContainingPoint(pt);
	assert(GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKOBBSphereIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	OBB a = RandomOBBContainingPoint(pt, 10.f);
	Sphere b = RandomSphereContainingPoint(pt, SCALE);
	assert(GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKOBBTriangleIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	OBB a = RandomOBBContainingPoint(pt, 10.f);
	Triangle b = RandomTriangleContainingPoint(pt);
	assert(GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKSphereSphereIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Sphere a = RandomSphereContainingPoint(pt, 10.f);
	Sphere b = RandomSphereContainingPoint(pt, 10.f);
	assert(GJKIntersect(a, b));
}

/*
UNIQUE_TEST(GJKSphereLineSegmentIntersectCase)
{
	Sphere a(POINT_VEC(52.4970627f,45.4888649f,-7.32828188f),9.61045837f);
	LineSegment b(POINT_VEC(90.447998f,30.0441036f,-46.333149f),POINT_VEC(35.6951218f,38.615181f,4.10816383f));
	assert(GJKIntersect(a, b));
}
*/

RANDOMIZED_TEST(GJKSphereLineSegmentIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Sphere a = RandomSphereContainingPoint(pt, 10.f);
	LineSegment b = RandomLineSegmentContainingPoint(pt);
	try
	{
		assert(GJKIntersect(a, b));
	} catch(...)
	{
		LOGI("%s", a.SerializeToCodeString().c_str());
		LOGI("%s", b.SerializeToCodeString().c_str());
		throw;
	}
}

RANDOMIZED_TEST(GJKTriangleLineSegmentIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Triangle a = RandomTriangleContainingPoint(pt);
	LineSegment b = RandomLineSegmentContainingPoint(pt);
	assert(GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKTriangleTriangleIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Triangle a = RandomTriangleContainingPoint(pt);
	Triangle b = RandomTriangleContainingPoint(pt);
	assert(GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKAABBAABBNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));

	AABB a = RandomAABBInHalfspace(p, 10.f);
	p.ReverseNormal();
	AABB b = RandomAABBInHalfspace(p, 10.f);
	assert(!GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKAABBOBBNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	AABB a = RandomAABBInHalfspace(p, 10.f);
	p.ReverseNormal();
	OBB b = RandomOBBInHalfspace(p, 10.f);
	assert(!GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKAABBLineSegmentNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	AABB a = RandomAABBInHalfspace(p, 10.f);
	p.ReverseNormal();
	LineSegment b = RandomLineSegmentInHalfspace(p);
	assert(!GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKAABBSphereNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	AABB a = RandomAABBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Sphere b = RandomSphereInHalfspace(p, SCALE);
	assert(!GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKAABBTriangleNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	AABB a = RandomAABBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Triangle b = RandomTriangleInHalfspace(p);
	assert(!GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKOBBOBBNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	OBB a = RandomOBBInHalfspace(p, 10.f);
	p.ReverseNormal();
	OBB b = RandomOBBInHalfspace(p, 10.f);
	assert(!GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKOBBLineSegmentNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	OBB a = RandomOBBInHalfspace(p, 10.f);
	p.ReverseNormal();
	LineSegment b = RandomLineSegmentInHalfspace(p);
	assert(!GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKOBBSphereNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	OBB a = RandomOBBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Sphere b = RandomSphereInHalfspace(p, SCALE);
	assert(!GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKOBBTriangleNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	OBB a = RandomOBBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Triangle b = RandomTriangleInHalfspace(p);
	assert(!GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKSphereSphereNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Sphere a = RandomSphereInHalfspace(p, 10.f);
	p.ReverseNormal();
	Sphere b = RandomSphereInHalfspace(p, 10.f);
	assert(!GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKSphereLineSegmentNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Sphere a = RandomSphereInHalfspace(p, 10.f);
	p.ReverseNormal();
	LineSegment b = RandomLineSegmentInHalfspace(p);
	assert(!GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKSphereTriangleNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Sphere a = RandomSphereInHalfspace(p, 10.f);
	p.ReverseNormal();
	Triangle b = RandomTriangleInHalfspace(p);
	assert(!GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKTriangleLineSegmentNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Triangle a = RandomTriangleInHalfspace(p);
	p.ReverseNormal();
	LineSegment b = RandomLineSegmentInHalfspace(p);
	assert(!GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKTriangleTriangleNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Triangle a = RandomTriangleInHalfspace(p);
	p.ReverseNormal();
	Triangle b = RandomTriangleInHalfspace(p);
	assert(!GJKIntersect(a, b));
}
