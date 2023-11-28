#include <stdio.h>
#include <stdlib.h>

#include "../src/MathGeoLib.h"
#include "../src/Math/myassert.h"
#include "TestRunner.h"
#include "../src/Algorithm/GJK.h"
#include "../src/Algorithm/SAT.h"

MATH_IGNORE_UNUSED_VARS_WARNING

UNIQUE_TEST(AABBLineNoIntersectCase)
{
	Plane p(DIR_VEC(-0.72379446f,0.652315021f,-0.224959269f),27.2405319f);
	Line l(POINT_VEC(-16.0996895f,22.3687477f,-5.59284782f),DIR_VEC(-0.616314948f,-0.757768512f,-0.214342773f));
	assert(!p.Intersects(l));
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

UNIQUE_TEST(TrickyPlaneRayClosestPoint)
{
	Plane p(DIR_VEC(0.50561136f,-0.753886223f,0.419538707f),83.6198273f);
	Ray r(POINT_VEC(-34.2622185f,97.5630875f,14.6553354f),DIR_VEC(-0.433765054f,-0.636341155f,-0.637901187f));
	vec cp = p.ClosestPoint(r);
	assert(p.Contains(cp));
}

