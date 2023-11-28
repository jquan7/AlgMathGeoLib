#pragma once

#include "../src/MathGeoLibFwd.h"

AABB RandomAABBContainingPoint(const vec &pt, float maxSideLength);
OBB RandomOBBContainingPoint(const vec &pt, float maxSideLength);
Line RandomLineContainingPoint(const vec &pt);
Ray RandomRayContainingPoint(const vec &pt);
LineSegment RandomLineSegmentContainingPoint(const vec &pt);
Plane RandomPlaneContainingPoint(const vec &pt);
Triangle RandomTriangleContainingPoint(const vec &pt);
Circle2D RandomCircle2DContainingPoint(LCG &lcg, const float2 &pt, float maxRadius);

OBB RandomOBBInHalfspace(const Plane &plane, float maxSideLength);
Line RandomLineInHalfspace(const Plane &plane);
LineSegment RandomLineSegmentInHalfspace(const Plane &plane);
Plane RandomPlaneInHalfspace(Plane &plane);
Triangle RandomTriangleInHalfspace(const Plane &plane);
Polygon RandomPolygonInHalfspace(const Plane &plane);
