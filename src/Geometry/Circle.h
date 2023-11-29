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

/** @file Circle.h
	@author Jukka Jyl�nki
	@brief The Circle geometry object. */
#pragma once

#include "../MathGeoLibFwd.h"
#include "../Math/float3.h"

#ifdef MATH_ENABLE_STL_SUPPORT
#include <vector>
#endif

MATH_BEGIN_NAMESPACE

/// A two-dimensional circle in 3D space.
/** This class represents both a hollow circle (only edge) and a solid circle (disc). */
class Circle
{
public:
	/// The center position of this circle.
	vec pos;

	/// The normal direction of this circle. [similarOverload: pos]
	/** A circle is a two-dimensional object in 3D space. This normal vector (together with the pos member)
		specifies the plane in which this circle lies in.
		This vector is always normalized. If you assign to this member directly, be sure to only assign normalized vectors. */
	vec normal;

	/// The radius of the circle. [similarOverload: pos]
	/** This parameter must be strictly positive to specify a non-degenerate circle. If zero is specified, this circle
		is considered to be degenerate.
		@see Circle::Circle(). */
	float r;

	/// The default constructor does not initialize any members of this class.
	/** This means that the values of the members pos, normal and r are all undefined after creating a new circle using
		this default constructor. Remember to assign to them before use.
		@see pos, normal, r. */
	Circle() {}

	/// Constructs a new circle by explicitly specifying the member variables.
	/** @param center The center point of the circle.
		@param normal The direction vector that specifies the orientation of this circle. This vector must be normalized,
			this constructor will not normalize the vector for you (for performance reasons).
		@param radius The radius of the circle.
		@see pos, normal, r. */
	Circle(const vec &center, const vec &normal, float radius);

	/// Returns a normalized direction vector to the 'U direction' of the circle.
	/** This vector lies on the plane of this circle.
		The U direction specifies the first basis vector of a local space of this circle. */
	vec BasisU() const;

	/// Returns a normalized direction vector to the 'V direction' of the circle.
	/** This vector lies on the plane of this circle.
		The U direction specifies the second basis vector of a local space of this circle. */
	vec BasisV() const;

	/// Returns a point at the edge of this circle.
	/** @param angleRadians The direction of the point to get. A full circle is generated by the range [0, 2*pi],
			but it is ok to pass in values outside this range.
		@note This function is equivalent to calling GetPoint(float angleRadians, float d) with a value of d == 1.
		@return A point in world space at the edge of this circle. */
	vec GetPoint(float angleRadians) const;

	/// Returns a point inside this circle.
	/** @param angleRadians The direction of the point to get. A full circle is generated by the range [0, 2*pi],
			but it is ok to pass in values outside this range.
		@param d A value in the range [0,1] that specifies the normalzied distance of the point from the center of the circle.
			A value of 0 returns the center point of this circle. A value of 1 returns a point at the edge of this circle.
			The range of d is not restricted, so it is ok to pass in values larger than 1 to generate a point lying completely
			outside this circle. */
	vec GetPoint(float angleRadians, float d) const;

	/// Returns the center point of this circle.
	/** This point is also the center of mass for this circle. The functions CenterPoint() and Centroid() are equivalent.
		@see pos. */
	vec CenterPoint() const { return pos; }
	vec Centroid() const { return pos; } ///< [similarOverload: CenterPoint]

	/// Computes an extreme point of this Circle/Disc in the given direction.
	/** An extreme point is a farthest point of this Circle/Disc in the given direction. Given a direction,
		this point is not necessarily unique.
		@param direction The direction vector of the direction to find the extreme point. This vector may
			be unnormalized, but may not be null.
		@return An extreme point of this Circle/Disc in the given direction. The returned point is always at
			the edge of this Circle. */
	vec ExtremePoint(const vec &direction) const;

	/// Computes the plane this circle is contained in.
	/** All the points of this circle lie inside this plane.
		@see class Plane. */
	Plane ContainingPlane() const;

	/// Translates this Circle in world space.
	/** @param offset The amount of displacement to apply to this Circle, in world space coordinates.
		@see Transform(). */
	void Translate(const vec &offset);

	/// Applies a transformation to this Circle.
	/** @param transform The transformation to apply to this Circle. This transformation must be
		affine, and must contain an orthogonal set of column vectors (may not contain shear or projection).
		The transformation can only contain uniform scale, and may not contain mirroring.
		@see Translate(), Scale(), classes float3x3, float3x4, float4x4, Quat. */
	void Transform(const float3x3 &transform);
	void Transform(const float3x4 &transform);
	void Transform(const float4x4 &transform);
	void Transform(const Quat &transform);

	/// Tests if the given point is contained at the edge of this circle.
	/** @param point The target point to test.
		@param maxDistance The epsilon threshold to test the distance against. This effectively turns the circle into a torus
			for this test.
		@see DistanceToEdge(), DistanceToDisc(), ClosestPointToEdge(), ClosestPointToDisc().
		@todo Implement DiscContains(float3/LineSegment/Triangle). */
	bool EdgeContains(const vec &point, float maxDistance = 1e-6f) const;

	// Returns true if the given point lies inside this filled circle.
	// @param maxDistance The epsilon threshold to test the distance against.
//	bool DiscContains(const vec &point, float maxDistance = 1e-6f) const;
//	bool DiscContains(const LineSegment &lineseg, float maxDistance = 1e-6f) const;

	/// Computes the distance of the given object to the edge of this circle.
	/** @return The distance of the given point to the edge of this circle. If the point is contained on this circle,
			the value 0 is returned.
		@see DistanceToDisc(), ClosestPointToEdge(), ClosestPointToDisc(). */
	float DistanceToEdge(const vec &point) const;
//	float DistanceToEdge(const LineSegment &lineseg, float *d, vec *closestPoint) const;
//	float DistanceToEdge(const Line &line, float *d, vec *closestPoint) const;

	/// Computes the distance of the given object to this disc (filled circle).
	/** If the point is contained inside this disc, the value 0 is returned.
		@see DistanceToEdge(), ClosestPointToEdge(), ClosestPointToDisc(). */
	float DistanceToDisc(const vec &point) const;
/*
	float DistanceToDisc(const LineSegment &lineseg, float *d, vec *closestPoint) const;
	float DistanceToDisc(const Line &line, float *d, vec *closestPoint) const;
*/
	/// Computes the closest point on the edge of this circle to the given point.
	/** @see DistanceToEdge(), DistanceToDisc(), ClosestPointToDisc(). */
	vec ClosestPointToEdge(const vec &point) const;
//	vec ClosestPointToEdge(const LineSegment &lineseg, float *d) const;
//	vec ClosestPointToEdge(const Line &line, float *d) const;

	/// Computes the closest point on the disc of this circle to the given object.
	/** @see DistanceToEdge(), DistanceToDisc(), ClosestPointToEdge(). */
	vec ClosestPointToDisc(const vec &point) const;

	/// Tests this circle for an intersection against the given plane.
	/** @note For Circle-Plane intersection, there is no need to differentiate between a hollow or a filled circle (disc).
		@return The number of intersection points found for this circle and the given plane.
		@see IntersectsDisc(). */
	int Intersects(const Plane &plane, vec *pt1, vec *pt2) const;
	int Intersects(const Plane &plane) const;

	/// Tests this disc for an intersection against the given object.
	/** @see Intersects(). */
	bool IntersectsDisc(const Line &line) const;
	bool IntersectsDisc(const LineSegment &lineseg) const;

#if defined(MATH_ENABLE_STL_SUPPORT)
	/// Tests if this circle intersects the faces of the given OBB.
	/** @param obb The bounding box to test against. This box is treated as "hollow", i.e. only the faces of the OBB are considered to be
			a part of the OBB.
		@return A vector that contains all the detected points of intersection for this circle and the given OBB. If the circle is fully
			contained inside the OBB, or is fully outside the OBB, no intersection occurs, and the returned vector has zero elements.
		@see Intersects(), IntersectsDisc(). */
	VecArray IntersectsFaces(const OBB &obb) const;

	VecArray IntersectsFaces(const AABB &aabb) const;

	/// Returns a human-readable representation of this circle. Most useful for debugging purposes.
	/** The returned string specifies the center position, normal direction and the radius of this circle. */
	std::string ToString() const;
#endif
};

Circle operator *(const float3x3 &transform, const Circle &circle);
Circle operator *(const float3x4 &transform, const Circle &circle);
Circle operator *(const float4x4 &transform, const Circle &circle);
Circle operator *(const Quat &transform, const Circle &circle);

#ifdef MATH_ENABLE_STL_SUPPORT
std::ostream &operator <<(std::ostream &o, const Circle &circle);
#endif

MATH_END_NAMESPACE
