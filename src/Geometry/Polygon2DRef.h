#pragma once

#include "../MathBuildConfig.h"

#ifdef MATH_ENABLE_STL_SUPPORT
#include <sstream>
#endif
#include "../Math/float2.h"
#include "../Math/MathConstants.h"
#include "../Math/myassert.h"

/// Polygon2DRef represents a Polygon2D, but by referencing an existing set of points, rather than allocating/owning its own set of points.
struct Polygon2DRef
{
public:
	const vec2d *points;
	int num;

	FORCE_INLINE vec2d AnyPointFast() const
	{
		assert(points);
		assert(num > 0);
		return points[0];
	}

	vec2d ExtremePoint(const vec2d &direction, float &project_dist) const
	{
		vec2d mostExtreme = vec2d::nan;
		project_dist = -FLOAT_INF;
		for(int i = 0; i < num; ++i)
		{
			float d = Dot(direction, points[i]);
			if (d > project_dist)
			{
				project_dist = d;
				mostExtreme = points[i];
			}
		}
		return mostExtreme;
	}

#if defined(MATH_ENABLE_STL_SUPPORT)
	std::string SerializeToString() const
	{
		std::stringstream ss;
		ss << "(";
		for(int i = 0; i < num; ++i)
			ss << "(" << points[i].SerializeToString() + (i+1 != num ? ")," : ")");
		ss << ")";
		return ss.str();
	}
#endif
};

