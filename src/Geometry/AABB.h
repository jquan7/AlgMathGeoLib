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

/** @file AABB.h
	@author Jukka Jyl�nki
	@brief The Axis-Aligned Bounding Box (AABB) geometry object. */
#pragma once

#include "../MathGeoLibFwd.h"
#include "../Math/float3.h"
#include "../Math/SSEMath.h"

#ifdef MATH_AUTOMATIC_SSE
#include "../Math/float4.h"
#endif

MATH_BEGIN_NAMESPACE
/// A 3D axis-aligned bounding box.
/** This data structure can be used to represent coarse bounds of objects, in situations where detailed triangle-level
	computations can be avoided. In physics systems, bounding boxes are used as an efficient early-out test for geometry
	intersection queries.

	The 'axis-aligned' part in the name means that the local axes of this bounding box are restricted to align with the
	axes of the world space coordinate system. This makes computations involving AABB's very fast, since AABB's cannot
	be arbitrarily oriented in the space with respect to each other.

	If you need to represent a box in 3D space with arbitrary orientation, see the class OBB. */
class ALIGN16 AABB
{
public:

	/// Specifies the minimum extent of this AABB in the world space x, y and z axes.
	vec minpt;
	/// Specifies the maximum extent of this AABB in the world space x, y and z axes. [similarOverload: minpt]
	vec maxpt;

	/// The default constructor does not initialize any members of this class.
	/** This means that the values of the members minpt and maxpt are undefined after creating a new AABB using this
		default constructor. Remember to assign to them before use.
		@see minpt, maxpt. */
	AABB() {}

	/// Constructs this AABB by specifying the minimum and maximum extending corners of the box.
	/** @see minpt, maxpt. */
	AABB(const vec &minpt, const vec &maxpt);

	/// Constructs this AABB to enclose the given OBB.
	/** This constructor computes the optimal minimum volume AABB that encloses the given OBB.
		@note Since an AABB cannot generally represent an OBB, this conversion is not exact, but the returned AABB
			specifies a larger volume.
		@see class OBB. */
	explicit AABB(const OBB &obb);

	FORCE_INLINE static int NumFaces() { return 6; }
	FORCE_INLINE static int NumEdges() { return 12; }
	FORCE_INLINE static int NumVertices() { return 8; }

	/// Returns the minimum world-space coordinate along the given axis.
	float MinX() const { return minpt.x; }
	float MinY() const { return minpt.y; } ///< [similarOverload: MinX]
	float MinZ() const { return minpt.z; } ///< [similarOverload: MinX]
	/// Returns the maximum world-space coordinate along the given axis.
	float MaxX() const { return maxpt.x; }
	float MaxY() const { return maxpt.y; } ///< [similarOverload: MaxX]
	float MaxZ() const { return maxpt.z; } ///< [similarOverload: MaxX]

	/// Sets this structure to a degenerate AABB that does not have any volume.
	/** This function is useful for initializing the AABB to "null" before a loop of calls to Enclose(),
		which incrementally expands the bounds of this AABB to enclose the given objects.
		@see Enclose(). */
	void SetNegativeInfinity();

	/// Sets this AABB by specifying its center and size.
	/** @param center The center point of this AABB.
		@param size A vector that specifies the size of this AABB in x, y and z directions.
		@see SetFrom(), FromCenterAndSize(). */
	void SetFromCenterAndSize(const vec &center, const vec &size);

	/// Sets this AABB to enclose the given OBB.
	/** This function computes the minimal axis-aligned bounding box for the given oriented bounding box. If the orientation
		of the OBB is not aligned with the world axes, this conversion is not exact and loosens the volume of the bounding box.
		@param obb The oriented bounding box to convert into this AABB.
		@see SetCenter(), class OBB. */
	void SetFrom(const OBB &obb);

	/// Sets this AABB to enclose the given set of points.
	/** @param pointArray A pointer to an array of points to enclose inside an AABB.
		@param numPoints The number of elements in the pointArray list.
		@see MinimalEnclosingAABB(). */
	void SetFrom(const vec *pointArray, int numPoints);

	/// Converts this AABB to an OBB.
	/** This function returns an OBB representation of this AABB. This conversion is exact, meaning that the returned
		OBB represents the same set of points than this AABB.
		@see class OBB, ToPBVolume(). */
	OBB ToOBB() const;

	/// Tests if this AABB is finite.
	/** @return True if the member variables of this AABB are valid floats and do not contain NaNs or infs, and false otherwise.
		@see IsDegenerate(), minpt, maxpt. */
	bool IsFinite() const;

	/// Tests if this AABB is degenerate.
	/** @return True if this AABB does not span a strictly positive volume.
		@see IsFinite(), Volume(), minpt, maxpt. */
	bool IsDegenerate() const;

	/// @return The center point of this AABB.
	vec CenterPoint() const;
	/// [similarOverload: CenterPoint]
	vec Centroid() const { return CenterPoint(); }

	/// Quickly returns an arbitrary point inside this AABB. Used in GJK intersection test.
	vec AnyPointFast() const { return minpt; }

	/// Generates a point inside this AABB.
	/** @param x A normalized value between [0,1]. This specifies the point position along the world x axis.
		@param y A normalized value between [0,1]. This specifies the point position along the world y axis.
		@param z A normalized value between [0,1]. This specifies the point position along the world z axis.
		@return A point inside this AABB at point specified by given parameters.
		@see Edge(), CornerPoint(), PointOnEdge(), FaceCenterPoint(), FacePoint(). */
	vec PointInside(float x, float y, float z) const;

	/// Returns an edge of this AABB.
	/** @param edgeIndex The index of the edge line segment to get, in the range [0, 11].
		@todo Specify which index generates which edge.
		@see PointInside(), CornerPoint(), PointOnEdge(), FaceCenterPoint(), FacePoint(). */
	LineSegment Edge(int edgeIndex) const;

	/// Returns a corner point of this AABB.
	/** This function generates one of the eight corner points of this AABB.
		@param cornerIndex The index of the corner point to generate, in the range [0, 7].
			The points are returned in the order 0: ---, 1: --+, 2: -+-, 3: -++, 4: +--, 5: +-+, 6: ++-, 7: +++. (corresponding the XYZ axis directions).
		@todo Draw which index generates which corner point.
		@see PointInside(), Edge(), PointOnEdge(), FaceCenterPoint(), FacePoint(), GetCornerPoints(). */
	vec CornerPoint(int cornerIndex) const;

	/// Computes an extreme point of this AABB in the given direction.
	/** An extreme point is a farthest point of this AABB in the given direction. Given a direction,
		this point is not necessarily unique.
		@param direction The direction vector of the direction to find the extreme point. This vector may
			be unnormalized, but may not be null.
		@return An extreme point of this AABB in the given direction. The returned point is always a
			corner point of this AABB.
		@see CornerPoint(). */
	vec ExtremePoint(const vec &direction) const;
	vec ExtremePoint(const vec &direction, float &projectionDistance) const;

	/// Returns a point on an edge of this AABB.
	/** @param edgeIndex The index of the edge to generate a point to, in the range [0, 11]. @todo Document which index generates which one.
		@param u A normalized value between [0,1]. This specifies the relative distance of the point along the edge.
		@see PointInside(), CornerPoint(), CornerPoint(), FaceCenterPoint(), FacePoint(). */
	vec PointOnEdge(int edgeIndex, float u) const;

	/// Returns the point at the center of the given face of this AABB.
	/** @param faceIndex The index of the AABB face to generate the point at. The valid range is [0, 5].
			This index corresponds to the planes in the order (-X, +X, -Y, +Y, -Z, +Z).
		@see PointInside(), CornerPoint(), PointOnEdge(), PointOnEdge(), FacePoint(). */
	vec FaceCenterPoint(int faceIndex) const;

	/// Generates a point at the surface of the given face of this AABB.
	/** @param faceIndex The index of the AABB face to generate the point at. The valid range is [0, 5].
			This index corresponds to the planes in the order (-X, +X, -Y, +Y, -Z, +Z).
		@param u A normalized value between [0, 1].
		@param v A normalized value between [0, 1].
		@see PointInside(), CornerPoint(), PointOnEdge(), PointOnEdge(), FaceCenterPoint(). */
	vec FacePoint(int faceIndex, float u, float v) const;

	/// Returns the surface normal direction vector the given face points towards.
	/** @param faceIndex The index of the AABB face to generate the point at. The valid range is [0, 5].
			This index corresponds to the planes in the order (-X, +X, -Y, +Y, -Z, +Z).
		@see FacePoint(), FacePlane(). */
	vec FaceNormal(int faceIndex) const;

	/// Computes the plane equation of the given face of this AABB.
	/** @param faceIndex The index of the AABB face. The valid range is [0, 5].
			This index corresponds to the planes in the order (-X, +X, -Y, +Y, -Z, +Z).
		@return The plane equation the specified face lies on. The normal of this plane points outwards from this AABB.
		@see FacePoint(), FaceNormal(), GetFacePlanes(). */
	Plane FacePlane(int faceIndex) const;

	/// Fills an array with all the eight corner points of this AABB.
	/** @param outPointArray [out] The array to write the points to. Must have space for 8 elements.
		@see CornerPoint(). */
	void GetCornerPoints(vec *outPointArray) const;

	/// Fills an array with all the six planes of this AABB.
	/** @param outPlaneArray [out] The array to write the planes to. Must have space for 6 elements.
		@see FacePlane(). */
	void GetFacePlanes(Plane *outPlaneArray) const;

	/// Generates an AABB that encloses the given point set.
	/** This function finds the smallest AABB that contains the given set of points.
		@param pointArray A pointer to an array of points to enclose inside an AABB.
		@param numPoints The number of elements in the pointArray list.
		@see SetFrom(). */
	static AABB MinimalEnclosingAABB(const vec *pointArray, int numPoints);
	AABB MinimalEnclosingAABB() const { return *this; }

	/// Finds the most extremal points along the three world axes simultaneously.
	/** @param pointArray A pointer to an array of points to process.
		@param numPoints The number of elements in the pointArray list.
		@param minx [out] Receives the point that has the smallest x coordinate.
		@param maxx [out] Receives the point that has the largest x coordinate.
		@param miny [out] Receives the point that has the smallest y coordinate.
		@param maxy [out] Receives the point that has the largest y coordinate.
		@param minz [out] Receives the point that has the smallest z coordinate.
		@param maxz [out] Receives the point that has the largest z coordinate. */
	static void ExtremePointsAlongAABB(const vec *pointArray, int numPoints, int &minx, int &maxx, int &miny, int &maxy, int &minz, int &maxz);

	/// Creates a new AABB given is center position and size along the X, Y and Z axes.
	/** @see SetCenter(). */
	static AABB FromCenterAndSize(const vec &aabbCenterPos, const vec &aabbSize);

	/// Returns the side lengths of this AABB in x, y and z directions.
	/** The returned vector is equal to the diagonal vector of this AABB, i.e. it spans from the
		minimum corner of the AABB to the maximum corner of the AABB.
		@see HalfSize(), Diagonal(). */
	vec Size() const;

	/// [similarOverload: Size]
	/** Returns Size()/2.
		@see Size(), HalfDiagonal(). */
	vec HalfSize() const;

	/// Returns the diameter vector of this AABB.
	/** @note For AABB, Diagonal() and Size() are the same concept. These functions are provided for symmetry
		with the OBB class.
		@see Size(), HalfDiagonal(). */
	vec Diagonal() const { return Size(); }

	/// [similarOverload: Diagonal]
	/** Returns Diagonal()/2.
		@see Diagonal(), HalfSize(). */
	vec HalfDiagonal() const { return HalfSize(); }

	/// Computes the volume of this AABB.
	/** @see SurfaceArea(), IsDegenerate(). */
	float Volume() const;

	/// Computes the surface area of the faces of this AABB.
	/** @see Volume(). */
	float SurfaceArea() const;

	/// Translates this AABB in world space.
	/** @param offset The amount of displacement to apply to this AABB, in world space coordinates.
		@see Scale(), Transform(). */
	void Translate(const vec &offset);
	AABB Translated(const vec &offset) const;

	/// Applies a uniform scale to this AABB.
	/** This function scales this AABB structure in-place, using the given center point as the origin
		for the scaling operation.
		@param centerPoint Specifies the center of the scaling operation, in world space.
		@param scaleFactor The uniform scale factor to apply to each world space axis.
		@see Translate(), Transform(). */
	void Scale(const vec &centerPoint, float scaleFactor);

	/// Applies a non-uniform scale to this AABB.
	/** This function scales this AABB structure in-place, using the given center point as the origin
		for the scaling operation.
		@param centerPoint Specifies the center of the scaling operation, in world space.
		@param scaleFactor The non-uniform scale factors to apply to each world space axis.
		@see Translate(), Transform(). */
	void Scale(const vec &centerPoint, const vec &scaleFactor);

	/// Applies a transformation to this AABB.
	/** This function transforms this AABB with the given transformation, and then recomputes this AABB
		to enclose the resulting oriented bounding box. This transformation is not exact and in general, calling
		this function results in the loosening of the AABB bounds.
		@param transform The transformation to apply to this AABB. This function assumes that this
			transformation does not contain shear, nonuniform scaling or perspective properties, i.e. that the fourth
			row of the float4x4 is [0 0 0 1].
		@see Translate(), Scale(), Transform(), classes float3x3, float3x4, float4x4, Quat. */
	void TransformAsAABB(const float3x3 &transform);
	void TransformAsAABB(const float3x4 &transform);
	void TransformAsAABB(const float4x4 &transform);
	void TransformAsAABB(const Quat &transform);

	/// Applies a transformation to this AABB and returns the resulting OBB.
	/** Transforming an AABB produces an oriented bounding box. This set of functions does not apply the transformation
		to this object itself, but instead returns the OBB that results in the transformation.
		@param transform The transformation to apply to this AABB. This function assumes that this
			transformation does not contain shear, nonuniform scaling or perspective properties, i.e. that the fourth
			row of the float4x4 is [0 0 0 1].
		@see Translate(), Scale(), TransformAsAABB(), classes float3x3, float3x4, float4x4, Quat. */
	OBB Transform(const float3x3 &transform) const;
	OBB Transform(const float3x4 &transform) const;
	OBB Transform(const float4x4 &transform) const;
	OBB Transform(const Quat &transform) const;

	/// Computes the closest point inside this AABB to the given point.
	/** If the target point lies inside this AABB, then that point is returned.
		@see Distance(), Contains(), Intersects().  */
	vec ClosestPoint(const vec &tar_pt) const;

	/// Computes the distance between this AABB and the given object.
	/** This function finds the nearest pair of points on this and the given object, and computes their distance.
		If the two objects intersect, or one object is contained inside the other, the returned distance is zero.
		@see Contains(), Intersects(), ClosestPoint(). */
	float Distance(const vec &point) const;

	/// Tests if the given object is fully contained inside this AABB.
	/** This function returns true if the given object lies inside this AABB, and false otherwise.
		@note The comparison is performed using less-or-equal, so the faces of this AABB count as being inside, but
			due to float inaccuracies, this cannot generally be relied upon.
		@see Distance(), Intersects(), ClosestPoint(). */
	bool Contains(const vec &point) const;
	bool Contains(const LineSegment &lineseg) const;
	bool Contains(const vec &aabbMinPoint, const vec &aabbMaxPoint) const;
	bool Contains(const AABB &aabb) const { return Contains(aabb.minpt, aabb.maxpt); }
	bool Contains(const OBB &obb) const;
	bool Contains(const Triangle &triangle) const;
	bool Contains(const Polygon &polygon) const;

	/// Tests whether this AABB and the given object intersect.
	/** Both objects are treated as "solid", meaning that if one of the objects is fully contained inside
		another, this function still returns true. (e.g. in case a line segment is contained inside this AABB,
		or this AABB is contained inside a OBB, etc.)
		@param dNear [out] If specified, receives the parametric distance along the line denoting where the
			line entered this AABB.
		@param dFar [out] If specified, receives the parametric distance along the line denoting where the
			line exited this AABB.
		@see Contains(), Distance(), ClosestPoint().
		@note If you do not need the intersection intervals, you should call the functions without these
			parameters in the function signature for optimal performance. */
	bool Intersects(const Line &line, float &dNear, float &dFar) const;
	bool Intersects(const Line &line) const;
	bool Intersects(const LineSegment &lineseg, float &dNear, float &dFar) const;
	bool Intersects(const LineSegment &lineseg) const;
	bool Intersects(const Plane &plane) const;
	bool Intersects(const AABB &aabb) const;
	bool Intersects(const OBB &obb) const;
	bool Intersects(const Triangle &triangle) const;
	bool Intersects(const Polygon &polygon) const;

	/// Projects this AABB onto the given axis.
	/** @param axis The axis to project onto. This vector can be unnormalized.
		@param dMin [out] Returns the minimum extent of this AABB on the given axis.
		@param dMax [out] Returns the maximum extent of this AABB on the given axis. */
	void ProjectToAxis(const vec &axis, float &dMin, float &dMax) const;

	int UniqueFaceNormals(vec *out) const;
	int UniqueEdgeDirections(vec *out) const;

	/// Expands this AABB to enclose the given object.
	/** This function computes an AABB that encloses both this AABB and the specified object, and stores the resulting
		AABB into this.
		@note The generated AABB is not necessarily the optimal enclosing AABB for this AABB and the given object. */
	void Enclose(const vec &point);
	void Enclose(const vec &aabbMinPoint, const vec &aabbMaxPoint);
	void Enclose(const LineSegment &lineseg);
	void Enclose(const AABB &aabb) { Enclose(aabb.minpt, aabb.maxpt); }
	void Enclose(const OBB &obb);
	void Enclose(const Triangle &triangle);
	void Enclose(const Polygon &polygon);
	void Enclose(const vec *pointArray, int numPoints);

	/// Generates an unindexed triangle mesh representation of this AABB.
	/** @param numFacesX The number of faces to generate along the X axis. This value must be >= 1.
		@param numFacesY The number of faces to generate along the Y axis. This value must be >= 1.
		@param numFacesZ The number of faces to generate along the Z axis. This value must be >= 1.
		@param outPos [out] An array of size numVertices which will receive a triangle list
			of vertex positions. Cannot be null.
		@param outNormal [out] An array of size numVertices which will receive vertex normals.
			If this parameter is null, vertex normals are not returned.
		@param outUV [out] An array of size numVertices which will receive vertex UV coordinates.
			If this parameter is null, a UV mapping is not generated.
		@param ccwIsFrontFacing If true, then the front-facing direction of the faces will be the sides
			with counterclockwise winding order. Otherwise, the faces are generated in clockwise winding order.
		The number of vertices that outPos, outNormal and outUV must be able to contain is
		(x*y + x*z + y*z)*2*6. If x==y==z==1, then a total of 36 vertices are required. Call
		NumVerticesInTriangulation to obtain this value.
		@see ToEdgeList(), NumVerticesInTriangulation(). */
	void Triangulate(int numFacesX, int numFacesY, int numFacesZ,
	                 vec *outPos, vec *outNormal, float2 *outUV,
	                 bool ccwIsFrontFacing) const;

	/// Returns the number of vertices that the Triangulate() function will output with the given subdivision parameters.
	/** @see Triangulate(). */
	static int NumVerticesInTriangulation(int numFacesX, int numFacesY, int numFacesZ)
	{
		return (numFacesX*numFacesY + numFacesX*numFacesZ + numFacesY*numFacesZ)*2*6;
	}

	/// Generates an edge list representation of the edges of this AABB.
	/** @param outPos [out] An array that contains space for at least 24 vertices (NumVerticesInEdgeList()).
		@see Triangulate(), Edge(), NumVerticesInEdgeList(). */
	void ToEdgeList(vec *outPos) const;

	/// Returns the number of vertices that the ToEdgeList() function will output.
	/** @see ToEdgeList(). */
	static int NumVerticesInEdgeList()
	{
		return 4*3*2;
	}

#if defined(MATH_ENABLE_STL_SUPPORT)
	/// Returns a human-readable representation of this AABB. Most useful for debugging purposes.
	/** The returned string specifies the center point and the half-axes of this AABB. */
	std::string ToString() const;
	std::string SerializeToString() const;

	/// Returns a string of C++ code that can be used to construct this object. Useful for generating test cases from badly behaving objects.
	std::string SerializeToCodeString() const;

	static AABB FromString(const std::string &str) { return FromString(str.c_str()); }
#endif

	static AABB FromString(const char *str, const char **outEndStr = 0);

	/// Finds the set intersection of this and the given AABB.
	/** @return This function returns the AABB that is contained in both this and the given AABB.  */
	AABB Intersection(const AABB &aabb) const;

	/// Computes the intersection of a line, line segment and an AABB.
	/** Based on "T. Kay, J. Kajiya. line Tracing Complex Scenes. SIGGRAPH 1986 vol 20, number 4. pp. 269-"
		http://www.siggraph.org/education/materials/HyperGraph/raytrace/rtinter3.htm
		@param linePos The starting position of the line.
		@param lineDir The direction of the line. This direction vector must be normalized!
		@param tNear [in, out] For the test, the input line is treated as a line segment. Pass in the signed distance
			from the line origin to the start of the line. For a Line-AABB test, -FLOAT_INF is typically passed here.
			For a line-AABB test, 0.0f should be inputted. If intersection occurs, the signed distance from line origin
			to the line entry point in the AABB is returned here.
		@param tFar [in, out] Pass in the signed distance from the line origin to the end of the line. For Line-AABB and
			line-AABB tests, pass in FLOAT_INF. For a LineSegment-AABB test, pass in the length of the line segment here.
			If intersection occurs, the signed distance from line origin to the line exit point in the AABB
			is returned here.
		@return True if an intersection occurs, false otherwise.
		@note This is a low level utility function. It may be more convenient to use one of the AABB::Intersects()
			functions instead.
		@see Intersects(). */
	bool IntersectLineAABB(const vec &linePos, const vec &lineDir, float &tNear, float &tFar) const;

	bool IntersectLineAABB_CPP(const vec &linePos, const vec &lineDir, float &tNear, float &tFar) const;
#ifdef MATH_SIMD
	bool IntersectLineAABB_SSE(const float4 &linePos, const float4 &lineDir, float tNear, float tFar) const;
#endif

	bool Equals(const AABB &rhs, float epsilon = 1e-3f) const { return minpt.Equals(rhs.minpt, epsilon) && maxpt.Equals(rhs.maxpt, epsilon); }
};

OBB operator *(const float3x3 &transform, const AABB &aabb);
OBB operator *(const float3x4 &transform, const AABB &aabb);
OBB operator *(const float4x4 &transform, const AABB &aabb);
OBB operator *(const Quat &transform, const AABB &aabb);

#ifdef MATH_ENABLE_STL_SUPPORT
std::ostream &operator <<(std::ostream &o, const AABB &aabb);
#endif

#ifdef MATH_SIMD
void AABBTransformAsAABB_SIMD(AABB &aabb, const float4x4 &m);
#endif

MATH_END_NAMESPACE
