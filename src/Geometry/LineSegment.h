/*******************************************************************************
Copyright © Toramon Co., Ltd. 2017-2023. All Rights Reserved.
File name    : LineSegment.h
Author       : qijianquan qijq@toramon.com
Version      :
Date         : 2023-11-21 15:15:15
LastEditors  :
LastEditTime :
Description  : Line Segment in 3D space
Others       :
Log          :
*******************************************************************************/
#pragma once

#include "../MathGeoLibFwd.h"
#include "../Math/float3.h"

MATH_BEGIN_NAMESPACE

/// A line segment in 3D space.
class LineSegment
{
public:
    /// The starting point and end point.
    vec a, b;

    /// The default constructor.
    LineSegment() {}

    /// Constructs a line segment through the given points.
    LineSegment(const vec &a, const vec &b);

    /// Constructs a line segment from a a line.
    /** This constructor takes the line origin position as the start point.
        @param d The distance along the line for the end point of this line segment.
        @see a, b, class Line, Line::GetPoint(). */
    explicit LineSegment(const Line &line, float d);

    /// Returns a point on the line.
    /** @param d The normalized distance along the line segment to compute.
        @return (1-d)*a + d*b.
        @see a, b, Line::GetPoint(). */
    vec GetPoint(float d) const;

    /// Returns the center point of this line segment; GetPoint(0.5f).
    vec CenterPoint() const;

    /// Reverses the direction of this line segment.
    void Reverse();

    /// Returns the normalized direction vector that points in the direction a->b.
    vec Dir() const;

    /// Returns an arbitrary point inside this LineSegment. (for GJK intersection.)
    inline vec AnyPointFast() const { return a; }

    /// Computes an extreme point of this LineSegment in the given direction.
    /** An extreme point is a farthest point along this LineSegment in the given direction.
        Given a direction, this point is not necessarily unique.
        @param direction The direction vector of the direction to find the extreme point.
            This vector may be unnormalized, but may not be null.
        @return An extreme point of this LineSegment in the given direction.
            The returned point is always either a or b.
        @see a, b.*/
    vec ExtremePoint(const vec &direction) const;
    vec ExtremePoint(const vec &direction, float &project_dist) const;

    /// Translates this LineSegment in world space.
    /** @param offset The amount of displacement to apply to this LineSegment.
        @see Transform(). */
    void Translate(const vec &offset);

    /// Applies a transformation to this line in-place.
    void Transform(const float3x3 &transform);
    void Transform(const float3x4 &transform);
    void Transform(const float4x4 &transform);
    void Transform(const Quat &transform);

    /// Computes the length of this line segment.
    /** @return |b-a|.
        @see a, b. */
    float Length() const;
    /// Computes the squared length of this line segment.
    float LengthSq() const;

    /// Tests if this line segment is finite.
    /** @return True if both a and b have finite floating-point values. */
    bool IsFinite() const;

    /// Tests if this line segment represents the same set of points than the given line segment.
    /** @param dist_th Specifies how much distance threshold to allow in the comparison.
        @return True if a == rhs.a && b == rhs.b, or, a == rhs.b && b = rhs.a, within the given epsilon. */
    bool Equals(const LineSegment &rhs, float dist_th = 1e-3f) const;

    /// Tests if the given point or line segment is contained on this line segment.
    bool Contains(const vec &point, float dist_th = 1e-3f) const;
    bool Contains(const LineSegment &lineseg, float dist_th = 1e-3f) const;

    /// Computes the closest point on this line segment to the given object.
    /** @param d [out] If specified, this parameter receives the normalized distance along
            this line segment which specifies the closest point on this line segment to
            the specified point.
        @return The closest point on this line segment to the given object.
        @see Contains(), Distance(), Intersects(). */
    vec ClosestPoint(const vec &point) const { float d; return ClosestPoint(point, d); }
    vec ClosestPoint(const vec &point, float &d) const;
    vec ClosestPointD(const vec &point, double &d) const;
    /** @param d2 [out] If specified, this parameter receives the (normalized, in case of line segment)
            distance along the other line object which specifies the closest point on that line to
            this line segment. */
    vec ClosestPoint(const Line &other) const { float d, d2; return ClosestPoint(other, d, d2); }
    vec ClosestPoint(const Line &other, float &d) const { float d2; return ClosestPoint(other, d, d2); }
    vec ClosestPoint(const Line &other, float &d, float &d2) const;
    vec ClosestPoint(const LineSegment &other) const { float d, d2; return ClosestPoint(other, d, d2); }
    vec ClosestPoint(const LineSegment &other, float &d) const { float d2; return ClosestPoint(other, d, d2); }
    vec ClosestPoint(const LineSegment &other, float &d, float &d2) const;

    /// Computes the distance between this line segment and the given object.
    /** @param d [out] If specified, this parameter receives the normalized distance along
            this line segment which specifies the closest point on this line segment to
            the specified point.
        @return The distance between this line segment and the given object.
        @see Constains(), ClosestPoint(), Intersects(). */
    float Distance(const vec &point) const { float d; return Distance(point, d); }
    float Distance(const vec &point, float &d) const;
    /** @param d2 [out] If specified, this parameter receives the (normalized, in case of line segment)
            distance along the other line object which specifies the closest point on that line to
            this line segment. */
    float Distance(const Line &other) const { float d, d2; return Distance(other, d, d2); }
    float Distance(const Line &other, float &d) const { float d2; return Distance(other, d, d2); }
    float Distance(const Line &other, float &d, float &d2) const;
    float Distance(const LineSegment &other) const { float d, d2; return Distance(other, d, d2); }
    float Distance(const LineSegment &other, float &d) const { float d2; return Distance(other, d, d2); }
    float Distance(const LineSegment &other, float &d, float &d2) const;
    float Distance(const Plane &other) const;

    float DistanceSq(const vec &point) const;
    float DistanceSq(const LineSegment &other) const;
    double DistanceSqD(const vec &point) const;

    /// Tests whether this line segment and the given object intersect.
    /** Both objects are treated as "solid", meaning that if one of the objects is fully contained inside
        another, this function still returns true. (for example, if this line segment is contained inside a LineSegment)
        @todo Output intersection point. */
    bool Intersects(const Plane &plane) const;
    /** @param d [out] If specified, this parameter receives the normalized distance along
            this line segment which specifies the closest point on this line segment to
            the specified point. This pointer may be null. */
    bool Intersects(const Plane &plane, float *d) const;
    /** @param intersect_pt [out] If specified, receives the point of intersection. This pointer may be null. */
    bool Intersects(const Triangle &triangle, float *d, vec *intersect_pt) const;
    /** @param near [out] The distance along this line to where the line enters.
        @param far [out] The distance along this line to where the line exits. */
    bool Intersects(const AABB &aabb, float &near, float &far) const;
    bool Intersects(const AABB &aabb) const;
    bool Intersects(const OBB &obb, float &near, float &far) const;
    bool Intersects(const OBB &obb) const;
    bool Intersects(const Polygon &polygon) const;
    /** @param epsilon If testing intersection between two line segments,
            a distance threshold value is used to account for floating-point inaccuracies. */
    bool Intersects(const LineSegment &lineseg, float epsilon = 1e-3f) const;
    /// Tests if this line segment intersects the given disc.
    bool Intersects(const Circle &disc) const;

    /// Converts this LineSegment to a Line: pos = a, dir = Dir().
    Line ToLine() const;

    /// Projects this LineSegment onto the given 1D axis direction vector.
    /** This function collapses this LineSegment onto an 1D axis for the purposes of e.g. separate axis test computations.
        The function returns a 1D range [outmin, outmax] denoting the interval of the projection.
        @param direction The 1D axis to project to. This vector may be unnormalized, in which case the output
            of this function gets scaled by the length of this vector.
        @param outmin [out] Returns the minimum extent of this object along the projection axis.
        @param outmax [out] Returns the maximum extent of this object along the projection axis. */
    void ProjectToAxis(const vec &direction, float &outmin, float &outmax) const;

#if defined(MATH_ENABLE_STL_SUPPORT)
    /// Returns a human-readable representation of this LineSegment. Most useful for debugging purposes.
    std::string ToString() const;
    std::string SerializeToString() const;

    /// Returns a string of C++ code that can be used to construct this object.
    std::string SerializeToCodeString() const;
    static LineSegment FromString(const std::string &str) { return FromString(str.c_str()); }
#endif

    static LineSegment FromString(const char *str, const char **outEndStr = 0);
};

LineSegment operator *(const float3x3 &transform, const LineSegment &line);
LineSegment operator *(const float3x4 &transform, const LineSegment &line);
LineSegment operator *(const float4x4 &transform, const LineSegment &line);
LineSegment operator *(const Quat &transform, const LineSegment &line);

#ifdef MATH_ENABLE_STL_SUPPORT
std::ostream &operator <<(std::ostream &o, const LineSegment &lineseg);
#endif

MATH_END_NAMESPACE
