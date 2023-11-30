/*******************************************************************************
Copyright © Toramon Co., Ltd. 2017-2023. All Rights Reserved.
File name    : Line.h
Author       : qijianquan qijq@toramon.com
Version      :
Date         : 2023-11-20 15:15:15
LastEditors  :
LastEditTime :
Description  : Line in 3D space
Others       :
Log          :
*******************************************************************************/
#pragma once

#include "../MathGeoLibFwd.h"
#include "../Math/float3.h"

MATH_BEGIN_NAMESPACE

/// A line in 3D space
class Line
{
public:
    /// Specifies the origin of this line.
    vec pos;

    /// The normalized direction vector of this Line.
    /** @note For proper functionality, this direction vector needs to always be
        normalized. If you set to this member manually, remember to make sure
        you only assign normalized direction vectors. */
    vec dir;

    /// The default constructor; pos and dir are undefined.
    Line() {}

    /// Constructs a new line by explicitly specifying the member variables.
    /** @param pos The origin position of the line.
        @param dir The direction of the line. This vector MUST be normalized.
        @see pos, dir. */
    Line(const vec &pos, const vec &dir);

    /// Converts a LineSegment to a Line.
    /** This constructor sets pos = lineseg.a,
        and dir = (lineseg.b - lineseg.a).Normalized().
        @see class LineSegment, ToLineSegment(). */
    explicit Line(const LineSegment &lineseg);

    bool IsFinite() const;

    /// Gets a point along the line at the given distance.
    /** @return pos + distance * dir.
        @see pos, dir. */
    vec GetPoint(float distance) const;

    /// Translates this Line in world space.
    /** @param offset The amount of displacement to apply to this Line.
        @see Transform(). */
    void Translate(const vec &offset);

    /// Applies a transformation to this line, in-place.
    /** @see Translate(), classes float3x3, float3x4, float4x4, Quat. */
    void Transform(const float3x3 &transform);
    void Transform(const float3x4 &transform);
    void Transform(const float4x4 &transform);
    void Transform(const Quat &transform);

    /// Tests if the given object is fully contained on this line.
    /** @param dist_th Used to allow errors caused by floating-point inaccuracies.
        @return True if contains.
        @see class LineSegment, Distance(), ClosestPoint(), Intersects(). */
    bool Contains(const vec &point, float dist_th = 1e-3f) const;
    bool Contains(const LineSegment &lineseg, float dist_th = 1e-3f) const;

    /// Tests if two lines are equal. Set equality (pos and dir can be completely different)
    bool Equals(const Line &line, float epsilon = 1e-3f) const;

    /// Computes the distance between this line and the given object.
    /** If the two objects intersect, or one object is contained inside the other, the returned distance is 0.
        @param d [out] If specified, receives the parametric distance along this
            line that specifies the closest point on this line to the given object.
            The value returned here can be negative.
        @see Contains(), Intersects(), ClosestPoint(), GetPoint(). */
    float Distance(const vec &point) const { float d; return Distance(point, d); }
    float Distance(const vec &point, float &d) const;
    /** @param d2 [out] If specified, receives the parametric distance along
        the other line that specifies the closest point on that line to this line.
        The value returned here can be negative. */
    float Distance(const Line &other) const { float d, d2; return Distance(other, d, d2); }
    float Distance(const Line &other, float &d) const { float d2; return Distance(other, d, d2); }
    float Distance(const Line &other, float &d, float &d2) const;
    float Distance(const LineSegment &other) const { float d, d2; return Distance(other, d, d2); }
    float Distance(const LineSegment &other, float &d) const { float d2; return Distance(other, d, d2); }
    float Distance(const LineSegment &other, float &d, float &d2) const;

    /// Computes the closest point on this line to the given object.
    /** If the other object intersects this line, this function will return an
        arbitrary point inside the region of intersection.
        @param d [out] If specified, receives the parametric distance along this
            line that specifies the closest point on this line to the given object.
            The value returned here can be negative.
        @see Contains(), Distance(), Intersects(), GetPoint(). */
    vec ClosestPoint(const vec &tar_pt) const { float d; return ClosestPoint(tar_pt, d); }
    vec ClosestPoint(const vec &tar_pt, float &d) const;
    /** @param d2 [out] If specified, receives the parametric distance along
        the other line that specifies the closest point on that line to this line.
        The value returned here can be negative. */
    vec ClosestPoint(const Line &other) const { float d, d2; return ClosestPoint(other, d, d2); }
    vec ClosestPoint(const Line &other, float &d) const { float d2; return ClosestPoint(other, d, d2); }
    vec ClosestPoint(const Line &other, float &d, float &d2) const;
    vec ClosestPoint(const LineSegment &other) const { float d, d2; return ClosestPoint(other, d, d2); }
    vec ClosestPoint(const LineSegment &other, float &d) const { float d2; return ClosestPoint(other, d, d2); }
    vec ClosestPoint(const LineSegment &other, float &d, float &d2) const;
    /** @param barycentric_coord_UV [out] If specified, receives the barycentric UV coordinates
            representing the closest point on the triangle to this line.
        @see Contains(), Distance(), Intersects(), GetPoint(), Triangle::Point(float u, float v). */
    vec ClosestPoint(const Triangle &triangle) const { float d; return ClosestPoint(triangle, d); }
    vec ClosestPoint(const Triangle &triangle, float &d) const;
    vec ClosestPoint(const Triangle &triangle, float &d, float2 &barycentric_coord_UV) const;

    /// Tests whether this line and the given object intersect.
    /** Both objects are treated as "solid", meaning that if one of the objects is fully contained inside
        another, this function still returns true.
        @param d [out] If specified, this parameter will receive the parametric distance of
            the intersection point along this object. Use the GetPoint(d) function
            to get the actual point of intersection. This pointer may be null.
        @param intersect_pt [out] If specified, receives the actual point of intersection. This pointer
            may be null.
        @return True if an intersection occurs or one of the objects is contained inside the other, false otherwise.
        @see Contains(), Distance(), ClosestPoint(), GetPoint(). */
    bool Intersects(const Triangle &triangle, float *d, vec *intersect_pt) const;
    bool Intersects(const Plane &plane, float *d) const;
    /** @param near [out] The distance along this line to where the line enters.
        @param far [out] The distance along this line to where the line exits. */
    bool Intersects(const AABB &aabb, float &near, float &far) const;
    bool Intersects(const AABB &aabb) const;
    bool Intersects(const OBB &obb, float &near, float &far) const;
    bool Intersects(const OBB &obb) const;
    bool Intersects(const Polygon &polygon) const;
    bool Intersects(const Circle &disc) const;

    /// Converts this Line to a LineSegment.
    /** @param d Specifies the position of the other endpoint along this Line.
        @return A LineSegment with point a at pos, and point b at pos + d * dir.
        @see pos, dir, Line::Line, class LineSegment. */
    LineSegment ToLineSegment(float d) const;

    /// Converts this Line to a LineSegment.
    /** @param start Specifies the position of the first endpoint along this Line.
        @param end Specifies the position of the second endpoint along this Line.
        @return A LineSegment with point a at pos + start * dir, and point b at pos + end * dir.
        @see pos, dir, Line::Line, class LineSegment, ToLine(). */
    LineSegment ToLineSegment(float start, float end) const;

    /// Projects this Line onto the given 1D axis direction vector.
    /** This function collapses this Line onto an 1D axis.
        @param direction The 1D axis to project to.
        @param outmin [out] Returns the minimum extent of this object along the projection axis.
        @param outmax [out] Returns the maximum extent of this object along the projection axis. */
    void ProjectToAxis(const vec &direction, float &outmin, float &outmax) const;

    /// Tests if the given three points are collinear.
    static bool AreCollinear(const vec &p1, const vec &p2, const vec &p3, float epsilon = 1e-3f);

    /// Computes the closest point pair on two lines.
    /** @param v0 The starting point of the first line.
        @param v10 The direction vector of the first line. This can be unnormalized.
        @param v2 The starting point of the second line.
        @param v32 The direction vector of the second line. This can be unnormalized.
        @param d [out] Receives the normalized distance of the closest point along the first line.
        @param d2 [out] Receives the normalized distance of the closest point along the second line.
        @return Returns the closest point on line start0<->end0 to the second line.
        @note This is a low-level utility function. You probably want to use ClosestPoint() or Distance() instead.
        @see ClosestPoint(), Distance(). */
    static void ClosestPointLineLine(const vec &v0, const vec &v10, const vec &v2, const vec &v32, float &d, float &d2);

#if defined(MATH_ENABLE_STL_SUPPORT)
    /// Returns a human-readable representation of this Line.
    std::string ToString() const;
    std::string SerializeToString() const;

    /// Returns a string of C++ code that can be used to construct this object.
    std::string SerializeToCodeString() const;
#endif
};

Line operator *(const float3x3 &transform, const Line &line);
Line operator *(const float3x4 &transform, const Line &line);
Line operator *(const float4x4 &transform, const Line &line);
Line operator *(const Quat &transform, const Line &line);

#ifdef MATH_ENABLE_STL_SUPPORT
std::ostream &operator <<(std::ostream &o, const Line &line);
#endif

MATH_END_NAMESPACE
