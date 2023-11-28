#include <stdio.h>
#include <stdlib.h>

#include "../src/MathGeoLib.h"
#include "../src/Math/myassert.h"
#include "TestRunner.h"
#include "../src/Algorithm/GJK.h"
#include "../src/Algorithm/SAT.h"

MATH_IGNORE_UNUSED_VARS_WARNING

AABB RandomAABBInHalfspace(const Plane &plane, float maxSideLength)
{
	float w = rng.Float(1e-3f, maxSideLength);
	float h = rng.Float(1e-3f, maxSideLength);
	float d = rng.Float(1e-3f, maxSideLength);

	AABB a(DIR_VEC(0, 0, 0), DIR_VEC(w, h, d)); // Store with w == 0, the translate below will set the origin and w=1.

	vec origin = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	a.Translate(origin);

	vec aabbExtremePoint = a.ExtremePoint(-plane.normal);
	float distance = plane.Distance(aabbExtremePoint);
	a.Translate((distance + GUARDBAND) * plane.normal);

	assert(!a.IsDegenerate());
	assert(a.IsFinite());
	assert(!a.Intersects(plane));
//	assert(a.SignedDistance(plane) > 0.f);
	aabbExtremePoint = a.ExtremePoint(-plane.normal);
	assert(plane.SignedDistance(aabbExtremePoint) > 0.f);
	assert(plane.SignedDistance(a) > 0.f);
	return a;
}

OBB RandomOBBInHalfspace(const Plane &plane, float maxSideLength)
{
	AABB a = RandomAABBInHalfspace(plane, maxSideLength);
	float3x4 rot = float3x4::RandomRotation(rng);
	float3x4 tm = float3x4::Translate(a.CenterPoint()) * rot * float3x4::Translate(-a.CenterPoint());
	OBB o = a.Transform(tm);

	vec obbExtremePoint = o.ExtremePoint(-plane.normal);
	float distance = plane.Distance(obbExtremePoint);
	o.Translate((distance + GUARDBAND) * plane.normal);

	assert1(!o.IsDegenerate(), o);
	assert(o.IsFinite());
	assert(!o.Intersects(plane));
//	assert(o.SignedDistance(plane) > 0.f);
	obbExtremePoint = o.ExtremePoint(-plane.normal);
	assert(plane.SignedDistance(obbExtremePoint) > 0.f);
	assert(plane.SignedDistance(o) > 0.f);

	return o;
}

Sphere RandomSphereInHalfspace(const Plane &plane, float maxRadius)
{
	Sphere s(vec::zero, rng.Float(0.001f, maxRadius));
	s.Translate(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)));

	vec extremePoint = s.ExtremePoint(-plane.normal);
	float distance = plane.Distance(extremePoint);
	s.Translate((distance + GUARDBAND) * plane.normal);

	assert(s.IsFinite());
	assert(!s.IsDegenerate());
	assert(!s.Intersects(plane));
//	assert(s.SignedDistance(plane) > 0.f);
	extremePoint = s.ExtremePoint(-plane.normal);
	assert(plane.SignedDistance(extremePoint) > 0.f);
	assert(plane.SignedDistance(s) > 0.f);
	return s;
}

Line RandomLineInHalfspace(const Plane &plane)
{
	vec linePos = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	if (plane.SignedDistance(linePos) < 0.f)
		linePos = plane.Mirror(linePos);
	linePos += plane.normal * 1e-2f;
	assert(plane.SignedDistance(linePos) >= 1e-3f);

	vec dir = plane.normal.RandomPerpendicular(rng);
	Line l(linePos, dir);
	assert(l.IsFinite());
	assert2(!plane.Intersects(l), plane.SerializeToCodeString(), l.SerializeToCodeString());
	assert1(plane.SignedDistance(l) > 0.f, plane.SignedDistance(l));
	return l;
}

Ray RandomRayInHalfspace(const Plane &plane)
{
	if (rng.Int(0, 10) == 0)
		return RandomLineInHalfspace(plane).ToRay();

	vec rayPos = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	if (plane.SignedDistance(rayPos) < 0.f)
		rayPos = plane.Mirror(rayPos);
	rayPos += plane.normal * 1e-2f;
	assert(plane.SignedDistance(rayPos) >= 1e-3f);

	vec dir = vec::RandomDir(rng);
	if (dir.Dot(plane.normal) < 0.f)
		dir = -dir;
	Ray r(rayPos, dir);
	assert(r.IsFinite());
	assert(plane.SignedDistance(r.GetPoint(SCALE*10.f)) > 0.f);
	assert(!plane.Intersects(r));
	assert(plane.SignedDistance(r) > 0.f);
	return r;
}

LineSegment RandomLineSegmentInHalfspace(const Plane &plane)
{
	float f = rng.Float(0.f, SCALE);
	LineSegment ls = RandomRayInHalfspace(plane).ToLineSegment(0.f, f);
	assert(ls.IsFinite());
	assert(plane.SignedDistance(ls) > 0.f);

	return ls;
}

Plane RandomPlaneInHalfspace(Plane &plane)
{
	Plane p2;
	p2.normal = plane.normal;
	p2.d = rng.Float(plane.d + 1e-2f, plane.d + 1e-2f + SCALE);
	assert(!p2.IsDegenerate());
	return p2;
}

Triangle RandomTriangleInHalfspace(const Plane &plane)
{
	vec a = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	vec b = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	vec c = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Triangle t(a,b,c);

	assert1(t.IsFinite(), t);
	assert1(!t.IsDegenerate(), t);

	vec extremePoint = t.ExtremePoint(-plane.normal);
	float distance = plane.Distance(extremePoint);
	t.Translate((distance + GUARDBAND) * plane.normal);

	assert1(t.IsFinite(), t);
	assert1(!t.IsDegenerate(), t);
	assert2(!t.Intersects(plane), t, plane);
//	assert(t.SignedDistance(plane) > 0.f);
	extremePoint = t.ExtremePoint(-plane.normal);
	assert(plane.SignedDistance(extremePoint) > 0.f);
	assert(plane.SignedDistance(t) > 0.f);

	return t;
}

RANDOMIZED_TEST(AABBAABBNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));

	AABB a = RandomAABBInHalfspace(p, 10.f);
	p.ReverseNormal();
	AABB b = RandomAABBInHalfspace(p, 10.f);
	assert2(!a.Intersects(b), a, b);
	assert(!b.Intersects(a));


	assert(!SATIntersect(a, b));

//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(AABBOBBNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	AABB a = RandomAABBInHalfspace(p, 10.f);
	p.ReverseNormal();
	OBB b = RandomOBBInHalfspace(p, 10.f);
	assert2(!a.Intersects(b), a, b);
	assert(!b.Intersects(a));
	assert(!SATIntersect(a, b));

//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

UNIQUE_TEST(AABBLineNoIntersectCase)
{
	Plane p(DIR_VEC(-0.72379446f,0.652315021f,-0.224959269f),27.2405319f);
	Line l(POINT_VEC(-16.0996895f,22.3687477f,-5.59284782f),DIR_VEC(-0.616314948f,-0.757768512f,-0.214342773f));
	assert(!p.Intersects(l));
}

RANDOMIZED_TEST(AABBLineNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	AABB a = RandomAABBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Line b = RandomLineInHalfspace(p);
	if (a.Intersects(b))
	{
		LOGI("AABB: %s", a.ToString().c_str());
		LOGI("Line: %s", b.ToString().c_str());
	}
	assert2(!a.Intersects(b), a, b);
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(AABBRayNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	AABB a = RandomAABBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Ray b = RandomRayInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(AABBLineSegmentNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	AABB a = RandomAABBInHalfspace(p, 10.f);
	p.ReverseNormal();
	LineSegment b = RandomLineSegmentInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
	assert(!b.Intersects(a));


//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(AABBPlaneNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	AABB a = RandomAABBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Plane b = RandomPlaneInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(AABBSphereNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	AABB a = RandomAABBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Sphere b = RandomSphereInHalfspace(p, SCALE);
	assert2(!a.Intersects(b), a, b);
	assert(!b.Intersects(a));
	assert(a.Distance(b) > 0.f);
	assert(b.Distance(a) > 0.f);


//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(AABBTriangleNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	AABB a = RandomAABBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Triangle b = RandomTriangleInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
	assert(!b.Intersects(a));


	assert(!SATIntersect(a, b));

//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

UNIQUE_TEST(AABBPolygonNoIntersectCase)
{
	AABB a(POINT_VEC(-1.07812309f,50.0289841f,-42.3423996f),POINT_VEC(4.804286f,50.5800514f,-42.3384552f));
	Polygon b;
	b.p.push_back(POINT_VEC(5.48830986f,-108.176254f,84.0496674f));
	b.p.push_back(POINT_VEC(5.48830986f,-69.97966f,84.0496674f));
	b.p.push_back(POINT_VEC(24.5866089f,-58.1762543f,114.95137f));
	b.p.push_back(POINT_VEC(36.3900108f,-89.0779572f,134.049667f));
	b.p.push_back(POINT_VEC(24.5866089f,-119.97966f,114.95137f));
	assert(!a.Intersects(b));
}

RANDOMIZED_TEST(OBBOBBNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	OBB a = RandomOBBInHalfspace(p, 10.f);
	p.ReverseNormal();
	OBB b = RandomOBBInHalfspace(p, 10.f);
	assert2(!a.Intersects(b), a, b);
	assert(!b.Intersects(a));


	assert(!SATIntersect(a, b));

//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

extern int xxxxx;

BENCHMARK(OBBOBBNoIntersect, "OBB-OBB No Intersection")
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	OBB a = RandomOBBInHalfspace(p, 10.f);
	p.ReverseNormal();
	OBB b = RandomOBBInHalfspace(p, 10.f);
	if (!a.Intersects(b))
		++xxxxx;
	if (!b.Intersects(a))
		++xxxxx;
}
BENCHMARK_END;

BENCHMARK(OBBOBBNoIntersect_SAT, "OBB-OBB SAT No Intersection")
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	OBB a = RandomOBBInHalfspace(p, 10.f);
	p.ReverseNormal();
	OBB b = RandomOBBInHalfspace(p, 10.f);
	if (!SATIntersect(a, b))
		++xxxxx;
	if (!SATIntersect(b, a))
		++xxxxx;
}
BENCHMARK_END;

BENCHMARK(OBBOBBNoIntersect_GJK, "OBB-OBB GJK No Intersection")
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	OBB a = RandomOBBInHalfspace(p, 10.f);
	p.ReverseNormal();
	OBB b = RandomOBBInHalfspace(p, 10.f);
	if (!GJKIntersect(a, b))
		++xxxxx;
	if (!GJKIntersect(b, a))
		++xxxxx;
}
BENCHMARK_END;

RANDOMIZED_TEST(OBBLineNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	OBB a = RandomOBBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Line b = RandomLineInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(OBBRayNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	OBB a = RandomOBBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Ray b = RandomRayInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(OBBLineSegmentNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	OBB a = RandomOBBInHalfspace(p, 10.f);
	p.ReverseNormal();
	LineSegment b = RandomLineSegmentInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
	assert(!b.Intersects(a));


//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(OBBPlaneNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	OBB a = RandomOBBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Plane b = RandomPlaneInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(OBBSphereNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	OBB a = RandomOBBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Sphere b = RandomSphereInHalfspace(p, SCALE);
	assert2(!a.Intersects(b), a, b);
	assert(!b.Intersects(a));
	assert(a.Distance(b) > 0.f);
	assert(b.Distance(a) > 0.f);


//	assert(a.Contains(a.ClosestPoint(b)));
////	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(OBBTriangleNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	OBB a = RandomOBBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Triangle b = RandomTriangleInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
	assert(!b.Intersects(a));


	assert(!SATIntersect(a, b));

//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(SphereSphereNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Sphere a = RandomSphereInHalfspace(p, 10.f);
	p.ReverseNormal();
	Sphere b = RandomSphereInHalfspace(p, 10.f);
	assert2(!a.Intersects(b), a, b);
	assert(!b.Intersects(a));
	assert(a.Distance(b) > 0.f);
	assert(b.Distance(a) > 0.f);


//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(SphereLineNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Sphere a = RandomSphereInHalfspace(p, 10.f);
	p.ReverseNormal();
	Line b = RandomLineInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
	assert(!b.Intersects(a));
	assert(a.Distance(b) > 0.f);
	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(SphereRayNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Sphere a = RandomSphereInHalfspace(p, 10.f);
	p.ReverseNormal();
	Ray b = RandomRayInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
	assert(!b.Intersects(a));
	assert(a.Distance(b) > 0.f);
	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(SphereLineSegmentNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Sphere a = RandomSphereInHalfspace(p, 10.f);
	p.ReverseNormal();
	LineSegment b = RandomLineSegmentInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
	assert(!b.Intersects(a));
	assert(a.Distance(b) > 0.f);
	assert(b.Distance(a) > 0.f);


//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(SpherePlaneNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Sphere a = RandomSphereInHalfspace(p, 10.f);
	p.ReverseNormal();
	Plane b = RandomPlaneInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
	assert(!b.Intersects(a));
	assert(a.Distance(b) > 0.f);
	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(SphereTriangleNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Sphere a = RandomSphereInHalfspace(p, 10.f);
	p.ReverseNormal();
	Triangle b = RandomTriangleInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
	assert(!b.Intersects(a));
	assert(a.Distance(b) > 0.f);
	assert(b.Distance(a) > 0.f);


//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

extern int xxxxx;

UNIQUE_TEST(PlaneLineNoIntersectCase)
{
	Plane p(DIR_VEC(-0.746312618f,0.586626351f,-0.31446299f),-35.7190437f);
	Line l(POINT_VEC(45.1519928f,46.7459641f,92.9752197f),DIR_VEC(0.631202042f,0.773685277f,-0.0547275133f));
	assert(!p.Intersects(l));
}

UNIQUE_TEST(PlaneLineNoIntersectCase2)
{
	Plane p(DIR_VEC(0.344275832f,-0.882686555f,0.31990397f),-56.7400818f);
	Line l(POINT_VEC(36.2179184f,88.9618607f,29.178812f),DIR_VEC(0.775070965f,0.459497392f,0.433736295f));
	assert(!p.Intersects(l));
}

RANDOMIZED_TEST(TriangleLineNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Triangle a = RandomTriangleInHalfspace(p);
	p.ReverseNormal();
	Line b = RandomLineInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
//	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
	assert4(a.Contains(a.ClosestPoint(b)), a.SerializeToCodeString(), b.SerializeToCodeString(), a.ClosestPoint(b).SerializeToCodeString(), a.Distance(a.ClosestPoint(b)));
	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(TriangleRayNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Triangle a = RandomTriangleInHalfspace(p);
	p.ReverseNormal();
	Ray b = RandomRayInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(TriangleLineSegmentNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Triangle a = RandomTriangleInHalfspace(p);
	p.ReverseNormal();
	LineSegment b = RandomLineSegmentInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
//	assert(!b.Intersects(a));


//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
	assert4(a.Contains(a.ClosestPoint(b)), a.SerializeToCodeString(), b.SerializeToCodeString(), a.ClosestPoint(b).SerializeToCodeString(), a.Distance(a.ClosestPoint(b)));
	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(TrianglePlaneNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Triangle a = RandomTriangleInHalfspace(p);
	p.ReverseNormal();
	Plane b = RandomPlaneInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

UNIQUE_TEST(TrickyTriangleTriangleSATNoIntersect)
{
	Triangle a, b;
	a.a = POINT_VEC(-19.234608f, 175.11060f, -65.702095f);
	a.b = POINT_VEC(-41.996265f, 86.449524f, 61.047047f);
	a.c = POINT_VEC(-40.166401f, 83.222511f, -7.1153078f);
	b.a = POINT_VEC(-23.087940f, 13.051629f, -21.682280f);
	b.b = POINT_VEC(-90.890396f, -61.371635f, -44.296501f);
	b.c = POINT_VEC(85.991585f, 38.734276f, 29.707987f);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));


	assert(!SATIntersect(a, b));

}

RANDOMIZED_TEST(TriangleTriangleNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Triangle a = RandomTriangleInHalfspace(p);
	p.ReverseNormal();
	Triangle b = RandomTriangleInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
	assert(!b.Intersects(a));


	assert(!SATIntersect(a, b));

//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
	assert4(a.Contains(a.ClosestPoint(b)), a.SerializeToCodeString(), b.SerializeToCodeString(), a.ClosestPoint(b).SerializeToCodeString(), a.Distance(a.ClosestPoint(b)));
	assert(!b.Contains(a.ClosestPoint(b)));
	assert(!a.Contains(b.ClosestPoint(a)));
	assert(b.Contains(b.ClosestPoint(a)));
}




RANDOMIZED_TEST(PlaneLineNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Plane a = RandomPlaneInHalfspace(p);
	p.ReverseNormal();
	Line b = RandomLineInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
///	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

UNIQUE_TEST(TrickyPlaneRayClosestPoint)
{
	Plane p(DIR_VEC(0.50561136f,-0.753886223f,0.419538707f),83.6198273f);
	Ray r(POINT_VEC(-34.2622185f,97.5630875f,14.6553354f),DIR_VEC(-0.433765054f,-0.636341155f,-0.637901187f));
	vec cp = p.ClosestPoint(r);
	assert(p.Contains(cp));
}

RANDOMIZED_TEST(PlaneRayNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Plane a = RandomPlaneInHalfspace(p);
	p.ReverseNormal();
	Ray b = RandomRayInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
	assert3(a.Contains(a.ClosestPoint(b)), a.SerializeToCodeString(), b.SerializeToCodeString(), a.ClosestPoint(b).SerializeToCodeString());
	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(PlaneLineSegmentNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Plane a = RandomPlaneInHalfspace(p);
	p.ReverseNormal();
	LineSegment b = RandomLineSegmentInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
	assert(!b.Intersects(a));
	assert(a.Distance(b) > 0.f);
	assert(b.Distance(a) > 0.f);
	assert4(a.Contains(a.ClosestPoint(b)), a.SerializeToCodeString(), b.SerializeToCodeString(), a.ClosestPoint(b).SerializeToCodeString(), a.Distance(a.ClosestPoint(b)));
	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(PlanePlaneNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Plane a = RandomPlaneInHalfspace(p);
	p.ReverseNormal();
	Plane b = RandomPlaneInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}
