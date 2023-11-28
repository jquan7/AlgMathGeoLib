#include <stdio.h>
#include <stdlib.h>

#include "../src/MathGeoLib.h"
#include "../src/Math/myassert.h"
#include "TestRunner.h"
#include "../src/Algorithm/GJK.h"
#include "../src/Algorithm/SAT.h"

MATH_IGNORE_UNUSED_VARS_WARNING

UNIQUE_TEST(Polygon_Contains_PointCase)
{
	Polygon p;

	p.p.push_back(POINT_VEC(1.f, 0.f, 1.f));
	p.p.push_back(POINT_VEC(1.f, 0.f, 0.f));
	p.p.push_back(POINT_VEC(0.f, 0.f, 0.f));
	p.p.push_back(POINT_VEC(0.f, 0.f, 1.f));
	vec pt = POINT_VEC(0.5f, 0.f, 0.0007f);

	assert(p.Contains(pt));
}

UNIQUE_TEST(TrickyAABBLineSegmentIntersectGJK)
{
	AABB a;
	a.minPoint = POINT_VEC(-60.895836f, 18.743414f, -17.829493f);
	a.maxPoint = POINT_VEC(-60.294441f, 23.510536f, -10.694467f);
	LineSegment b;
	b.a = POINT_VEC(-61.331539f, 16.955204f, -18.561975f);
	b.b = POINT_VEC(-53.097103f, 40.628937f, 30.422394f);
	assert(a.Intersects(b));
	assert(b.Intersects(a));


}

UNIQUE_TEST(AABBPolygonIntersectCase)
{
	AABB a(POINT_VEC(-6.86657524f,-87.7668533f,-40.2900276f),
		POINT_VEC(2.77308559f,-79.9921722f,-34.0750961f));
	Polygon b;
	b.p.push_back(POINT_VEC(-43.6267548f,-19.4667053f,-35.8499298f));
	b.p.push_back(POINT_VEC(56.3732452f,-19.4667053f,-35.8499298f));
	b.p.push_back(POINT_VEC(56.3732452f,-119.466698f,-35.8499298f));
	b.p.push_back(POINT_VEC(-43.6267548f,-119.466698f,-35.8499298f));
	assert(a.Intersects(b));
}

UNIQUE_TEST(AABBPolygonIntersectCase2)
{
	AABB a(POINT_VEC(-23.4495525f,-25.5456314f,82.2886734f),
		POINT_VEC(-14.9134054f,-22.428545f,87.317482f));
	Polygon b;
	b.p.push_back(POINT_VEC(74.7439194f,-20.989212f,181.39743f));
	b.p.push_back(POINT_VEC(-25.2560806f,-20.989212f,81.3974304f));
	b.p.push_back(POINT_VEC(74.7439194f,-120.989212f,81.3974304f));
	assert(a.Intersects(b));
}

UNIQUE_TEST(PolygonLineIntersectCase)
{
	Polygon p;
	p.p.push_back(POINT_VEC(-19.0257339f,93.0070877f,20.232048f));
	p.p.push_back(POINT_VEC(-19.0257339f,131.203674f,20.232048f));
	p.p.push_back(POINT_VEC(0.0725631714f,143.00708f,51.1337509f));
	p.p.push_back(POINT_VEC(11.875967f,112.105385f,70.232048f));
	p.p.push_back(POINT_VEC(0.0725631714f,81.2036819f,51.1337509f));

	Line l(POINT_VEC(-51.2448387f,66.6799698f,-31.887619f),DIR_VEC(-0.494226635f,-0.341286302f,-0.799539685f));

	assert(p.Intersects(l));
}

UNIQUE_TEST(PolygonPolygonIntersectCase)
{
	for(int i = 0; i < 2; ++i)
	{
		Triangle t1, t2;
		switch(i)
		{
		case 0:
			t1 = Triangle(POINT_VEC(0,0,0),
					POINT_VEC(1,0,0),
					POINT_VEC(0,1,0));
			t2 = Triangle(POINT_VEC(0.5f,   0, 1e-6f),
					POINT_VEC(1.5f,   0, 1e-6f),
					POINT_VEC(   0, 1.f, 1e-6f));
			break;
		case 1:
			t1 = Triangle(POINT_VEC(-18.8999062f,7.59376526f,25.2815018f),
				POINT_VEC(12.0017948f,26.6920624f,75.2815018f),
				POINT_VEC(-49.801609f,26.6920624f,75.2815018f));

			t2 = Triangle(POINT_VEC(-44.9369545f,11.4193268f,35.2969398f),
				POINT_VEC(-14.0352535f,30.5176239f,85.296936f),
				POINT_VEC(-75.8386536f,30.5176239f,85.296936f));
			break;
		}

		assert(t1.Intersects(t1));
		assert(t2.Intersects(t2));
		assert(t1.Intersects(t2));

		Polygon a = t1.ToPolygon();
		assert(a.Intersects(a));
		Polygon b = t2.ToPolygon();

		assert(a.Intersects(b));
	}
}

UNIQUE_TEST(PolygonPolygonIntersectCase2)
{
	Polygon a;
	a.p.push_back(POINT_VEC(40.6926041f,-36.1174965f, 0.f));
	a.p.push_back(POINT_VEC(40.6926041f,-9.93014526f, 0.f));
	a.p.push_back(POINT_VEC(43.8726807f,-9.93014526f, 0.f));
	a.p.push_back(POINT_VEC(43.8726807f,-36.1174965f, 0.f));

	Polygon b;
	b.p.push_back(POINT_VEC(70.9185791f,-46.9780273f, 0.f));
	b.p.push_back(POINT_VEC(70.9185791f, 53.0219727f, 0.f));
	b.p.push_back(POINT_VEC(-29.0814209f,53.0219727f, 0.f));
	b.p.push_back(POINT_VEC(-29.0814209f,-46.9780273f, 0.f));

	assert(a.Intersects(b));
}

UNIQUE_TEST(PolygonContainsPointCase)
{
	Polygon a;
	a.p.push_back(POINT_VEC(-27.6082363f,-17.8272648f,116.150414f));
	a.p.push_back(POINT_VEC(15.0997639f,-67.2276688f,12.971736f));
	a.p.push_back(POINT_VEC(15.062994f,-67.2823105f,12.9826784f));
	a.p.push_back(POINT_VEC(-27.6450062f,-17.8819065f,116.161354f));

	vec pt = POINT_VEC(12.1201611f,-63.8624725f,20.105011f);
	assert(a.Contains(pt, 1e-2f));
}

UNIQUE_TEST(TriangleLineSegmentIntersectCase)
{
	Triangle a(POINT_VEC(-45.7166939f,-104.675713f,17.1150723f),POINT_VEC(-20.9888325f,-89.1524963f,-31.5042286f),POINT_VEC(-1.45244789f,-76.914505f,-69.9231262f));
	LineSegment b(POINT_VEC(-19.0950012f,-134.222748f,-33.7456589f),POINT_VEC(-52.5003471f,-49.3652039f,28.5405655f));

	assert2(a.Intersects(b), a.SerializeToCodeString(), b.SerializeToCodeString());
	vec cp = a.ClosestPoint(b);
	assert(a.Contains(cp));
//	assert(b.Contains(cp));
}

UNIQUE_TEST(PlaneRayIntersectCase)
{
	Plane p(DIR_VEC(-0.25385046f,-0.518036366f,-0.816822112f),91.5489655f);
	Ray r(POINT_VEC(-70.5785141f,-19.6609783f,-77.6785507f),DIR_VEC(0.916250288f,0.141897082f,-0.374634057f));
	assert(p.Intersects(r));
}

UNIQUE_TEST(PlanePlaneIntersectCase)
{
	Plane a(DIR_VEC(-9.31284958e-005f,0.896122217f,-0.44380734f).Normalized(),-63.5531387f);
	Plane b(DIR_VEC(0.0797545761f,-0.9964259f,0.0185146127f).Normalized(),45.0416794f);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
}

TEST(PolygonContains2D)
{
	float xmin = 0.f, xmax = 10.f, ymin = 0.f, ymax = 10.f, z = 2.f;

	vec point = POINT_VEC((xmax-xmin)/2,(ymax-ymin)/2,z);
	Polygon pol;
	pol.p.push_back(POINT_VEC(xmin, ymin, z));
	pol.p.push_back(POINT_VEC(xmax, ymin, z));
	pol.p.push_back(POINT_VEC(xmax, ymax, z));
	pol.p.push_back(POINT_VEC(xmin, ymax, z));

	assert(pol.Contains(point));
}

#if 0
UNIQUE_TEST(PolygonContainsPointCase2)
{
	Polygon p;
	p.p.push_back(POINT_VEC(0,0,0));
	p.p.push_back(POINT_VEC(2.f,0,0));
	p.p.push_back(POINT_VEC(2.f,0.0646286f,0));
	p.p.push_back(POINT_VEC(0,0.0646286f,0));

	vec pt = POINT_VEC(1.f,0.0645294f,0);

	assert(p.Contains(pt));
}
#endif
