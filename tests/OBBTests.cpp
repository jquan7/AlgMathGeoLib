#include <stdio.h>
#include <stdlib.h>

#include "../src/MathGeoLib.h"
#include "../src/Math/myassert.h"
#include "TestRunner.h"
#include "TestData.h"
#include "../src/Algorithm/GJK.h"
#include "../src/Algorithm/SAT.h"

MATH_IGNORE_UNUSED_VARS_WARNING

MATH_BEGIN_NAMESPACE

using namespace TestData;

UNIQUE_TEST(OBB_ClosestPoint_Point)
{
	vec pt = POINT_VEC_SCALAR(0.f);
	OBB o(pt, DIR_VEC(1.f, 1.f, 1.f), vec::unitX, vec::unitY, vec::unitZ);
	assert(o.ClosestPoint(pt).Equals(pt));
	assert(o.ClosestPoint(POINT_VEC(5.f, 0.f, 0.f)).Equals(POINT_VEC(1.f, 0.f, 0.f)));
	assert(o.ClosestPoint(POINT_VEC(5.f, 5.f, 5.f)).Equals(POINT_VEC(1.f, 1.f, 1.f)));
	assert(o.ClosestPoint(POINT_VEC(-5.f, -5.f, -5.f)).Equals(POINT_VEC(-1.f, -1.f, -1.f)));
}

// Programmatically found test case of two OBBs which by construction should be disjoint.
// Verifies fix to bug https://github.com/juj/MathGeoLib/pull/50
UNIQUE_TEST(OBB_NoIntersect_OBB_Case1)
{
	OBB a;
	a.pos = POINT_VEC(-58.54f, -57.58f, 87.16f);
	a.r = DIR_VEC(0.49f, 1.86f, 2.29f);
	a.axis[0] = DIR_VEC(0.82f, -0.50f, -0.29f);
	a.axis[1] = DIR_VEC(-0.49f, -0.87f, 0.10f);
	a.axis[2] = DIR_VEC(-0.31f, 0.06f, -0.95f);

	OBB b;
	b.pos = POINT_VEC(-59.53f, -50.44f, 83.27f);
	b.r = DIR_VEC(3.62f, 3.07f, 5.00f);
	b.axis[0] = DIR_VEC(-0.41f, 0.81f, -0.42f);
	b.axis[1] = DIR_VEC(0.44f, -0.23f, -0.87f);
	b.axis[2] = DIR_VEC(-0.80f, -0.54f, -0.26f);

	assert(!a.Intersects(b));
}

BENCHMARK(OBBIntersectsOBB_Random, "OBB::Intersects(OBB) Random")
{
	if (obb[i].Intersects(obb[i+1]))
		++dummyResultInt;
}
BENCHMARK_END

BENCHMARK(OBBIntersectsOBB_Positive, "OBB::Intersects(OBB) Positive")
{
	if (obb[i].Intersects(obb[i]))
		++dummyResultInt;
}
BENCHMARK_END

BENCHMARK(OBBIntersectsOBB_GJK, "OBB::Intersects(OBB)_GJK")
{
	if (GJKIntersect(obb[i], obb[i+1]))
		++dummyResultInt;
}
BENCHMARK_END

BENCHMARK(OBBIntersectsOBB_SAT, "OBB::Intersects(OBB)_SAT")
{
	if (SATIntersect(obb[i], obb[i+1]))
		++dummyResultInt;
}
BENCHMARK_END

BENCHMARK(OBBContains, "OBB::Contains(point)")
{
	uf[i] = obb[i].Contains(ve[i]) ? 1.f : 0.f;
}
BENCHMARK_END

BENCHMARK(OBBClosestPoint, "OBB::ClosestPoint(point)")
{
	dummyResultVec += obb[i].ClosestPoint(ve[i]);
}
BENCHMARK_END


MATH_END_NAMESPACE
