/*******************************************************************************
Copyright © Toramon Co., Ltd. 2017-2023. All Rights Reserved.
File name    : float2.h
Author       : qijianquan qijq@toramon.com
Version      :
Date         : 2023-11-20 15:15:15
LastEditors  :
LastEditTime :
Description  : A 2D (x,y) ordered pair.
Others       :
Log          :
*******************************************************************************/
#pragma once

#include "../MathBuildConfig.h"

#ifdef MATH_ENABLE_STL_SUPPORT
#include <string>
#include <vector>
#endif

#include "../MathGeoLibFwd.h"
#include "MathConstants.h"
#include "assume.h"
#include "MathFunc.h"

MATH_BEGIN_NAMESPACE

/// A vector of form (x,y).
class float2
{
public:
    /// Specifies the number of elements in this vector.
    enum {
        Size = 2
    };

    /// The x,y component.
    /** float2 is 8 bytes in size. Components lies in the memory offsets 0-3 and 4-7 of this class. */
    float x, y;

    /// The default constructor.
    float2() {}

    /// The float2 copy constructor.
    float2(const float2 &rhs) { x = rhs.x; y = rhs.y; }

    /// Constructs a new float2 with the value (x, y).
    float2(float x, float y);

    /// Constructs a new float2 with the value (scalar, scalar).
    explicit float2(float scalar);

    /// Constructs this float2 from a C array, (data[0], data[1]).
    /** @param data An array carrying x and y. This pointer may not be null. */
    explicit float2(const float *data);

    /// Casts this float2 to a C array.
    /** Use ptr()[0] to access the x component, and ptr()[1] to access the y component.
        @note Since the returned pointer points to this class, do not dereference the pointer after this
            float2 has been deleted. You should never store a copy of the returned pointer.
        @return A pointer to the first float element of this class. The data is contiguous in memory.
        @see operator [](), At(). */
    FORCE_INLINE float *ptr() { return &x; }
    FORCE_INLINE const float *ptr() const { return &x; }

    /// Accesses an element of this vector using array notation.
    /** @param index The element to get. 0 for x, 1 for y.
        @note If you have a non-const instance of this class, you can use this notation to set the elements of
            this vector as well, e.g. vec[1] = 10.f; would set the y-component of this vector.
        @see ptr(), At(). */
    FORCE_INLINE float &operator [](int index) { return At(index); }
    FORCE_INLINE float operator [](int index) const { return At(index); }

    /// Accesses an element of this vector.
    /** @param index The element to get. 0 for x, 1 for y.
        @note If you have a non-const instance of this class, you can use this notation to set the elements of
            this vector as well, e.g. vec.At(1) = 10.f; would set the y-component of this vector.
        @see ptr(), operator [](). */
    FORCE_INLINE  float At(int index) const {
        MATH_NS::Clamp01(index);
        return ptr()[index];
    }
    FORCE_INLINE float &At(int index) {
        MATH_NS::Clamp01(index);
        return ptr()[index];
    }

    /// [indexTitle: operators +,-,*,/,=,+=,-=,*=,/=]
    /** @return float2(x + v.x, y + v.y); */
    float2 operator +(const float2 &v) const;
    /** @return float2(-x, -y). */
    float2 operator -() const;
    /// @return float2(x - v.x, y - v.y); */
    float2 operator -(const float2 &v) const;
    /** @return float2(x * scalar, y * scalar); */
    float2 operator *(float scalar) const;
    /** @return float2(x / scalar, y / scalar); */
    float2 operator /(float scalar) const;
    /** @return float2(+x, +y). */
    float2 operator +() const { return *this; }

    /** @return A reference to this. */
    /// Assigns a vector to another.
    float2 &operator =(const float2 &v);
    /// Adds a vector to this vector, in-place.
    float2 &operator +=(const float2 &v);
    /// Subtracts a vector from this vector, in-place.
    float2 &operator -=(const float2 &v);
    /// Multiplies this vector by a scalar, in-place.
    float2 &operator *=(float scalar);
    /// Divides this vector by a scalar, in-place.
    float2 &operator /=(float scalar);

#ifdef MATH_ENABLE_UNCOMMON_OPERATIONS
    // 一些未定义操作
    float2 operator *(const float2 &vector) const { return this->Mul(vector); }
    float2 operator /(const float2 &vector) const { return this->Div(vector); }
    float2 &operator *=(const float2 &vector) { *this = this->Mul(vector); return *this; }
    float2 &operator /=(const float2 &vector) { *this = this->Div(vector); return *this; }
#endif

    /// @return (x+v.x, y+v.y).
    float2 Add(const float2 &v) const { return *this + v; }
    /// @return (x+s, y+s).
    float2 Add(float s) const;
    /// @return (x-v.x, y-v.y).
    float2 Sub(const float2 &v) const { return *this - v; }
    /// @return (x-s, y-s).
    float2 Sub(float s) const;
    /// @return (s-x, s-y).
    float2 SubLeft(float s) const;
    /// @return (x*v.x, y*v.y).
    float2 Mul(const float2 &v) const;
    /// @return (x*s, y*s).
    float2 Mul(float s) const { return *this * s; }
    /// @return (x/v.x, y/v.y).
    float2 Div(const float2 &v) const;
    /// @return (x/s, y/s).
    float2 Div(float s) const { return *this / s; }
    /// @return (s/x, s/y).
    float2 DivLeft(float s) const;

    /// Performs a 2D swizzled access to this vector.
    float2 xx() const { return float2(x,x); }
    float2 xy() const { return float2(x,y); }
    float2 yx() const { return float2(y,x); }
    float2 yy() const { return float2(y,y); }

    /// Reinterpret-casts this float4 to a vec2d. (practically projects this 4D vector to 2D x-y part).
    FORCE_INLINE const vec2d &ToVec2D() const { return *this; }

    /// Performs a swizzled access to this vector.
    /** For example, Swizzled(2,1,0) return float3(z,y,x). Swizzled(2,2,2,2) returns float4(z,z,z,z). */
    float4 Swizzled(int i, int j, int k, int l) const;  // (i,j,k,l) in the range [0, 3]
    float3 Swizzled(int i, int j, int k) const;  // (i,j,k) [0, 2]
    float2 Swizzled(int i, int j) const;  // (i,j) [0, 1]

    /// Generates a new float2 by filling its entries by the given scalar.
    /** @see float2::float2(float scalar), SetFromScalar(). */
    static float2 FromScalar(float scalar);

    /// Fills each entry of this float2 by the given scalar.
    /** @see float2::float2(float scalar), FromScalar(). */
    void SetFromScalar(float scalar);

    /// Sets all elements of this vector.
    /** @see x, y, At().. */
    void Set(float x, float y);

    /// Polar coordinates to an euclidean float2 (x,y) pair.
    /** @param theta The direction of the vector; in the range [-pi, pi] (, or [0, 2pi]).
            The value theta==0 returns a value in the +X direction,
            the value theta=pi/2 corresponds to +Y,...
        @param length The magnitude of the vector.
        @see FromPolarCoordinates, ToPolarCoordinates, AimedAngle. */
    void SetFromPolarCoordinates(float theta, float length);
    void SetFromPolarCoordinates(const float2 &polar) { SetFromPolarCoordinates(polar.x, polar.y); }
    static float2 FromPolarCoordinates(float theta, float length);
    static float2 FromPolarCoordinates(const float2 &polar) { return FromPolarCoordinates(polar.x, polar.y); }

    /// Euclidean float2(x,y) to polar coordinates float2(theta, length).
    /** @return x stores the direction [-pi/2, pi/2]. y stores the length (radius) of this vector.
        @see SetFromPolarCoordinates, FromPolarCoorindates, AimedAngle. */
    float2 ToPolarCoordinates() const;

    /// Returns the aimed angle direction of this vector, in radians.
    /** @note This vector does not need to be normalized but to be non-zero.
        @return The aimed angle in the range [-pi/2, pi/2].
        @see ToPolarCoordinates, FromPolarCoordinates, SetFromPolarCoordinates. */
    float AimedAngle() const;

    /// Computes the length of this vector.
    /** @return Sqrt(x*x + y*y). */
    float Length() const;

    /// Computes the squared length of this vector.
    float LengthSq() const;

    /// Normalizes this float2.
    /** @note If this function fails to normalize the vector, no error message is printed,
            the vector is set to (1,0) and an error code 0 is returned.
        @note This function operates in-place.
        @return The old length of this vector, or 0 if normalization failed.
        @see Normalized(). */
    float Normalize();

    /// Returns a normalized copy of this vector.
    /** @note If the vector cannot be normalized (0), the vector (1, 0) is returned,
            and an error message is printed. If you do not want to generate an error
            message on failure, but want to handle the failure yourself, use Normalize() instead.
        @see Normalize(). */
    float2 Normalized() const;

    /// Scales this vector so that its new length is as given.
    /** @note This function operates in-place.
        @return newLength * Normalize()
        @see ScaledToLength(). */
    float ScaleToLength(float newLength);

    /// Returns a scaled copy of this vector which has its new length as given.
    /** @return newLength * Normalized()
        @see ScaleToLength(). */
    float2 ScaledToLength(float newLength) const;

    /// Tests if the length of this vector is one, up to the given epsilon.
    /** @see IsZero(), IsFinite(), IsPerpendicular(). */
    bool IsNormalized(float epsilonSq = 1e-5f) const;

    /// Tests if this is the zero vector, up to the given epsilon.
    /** @see IsNormalized(), IsFinite(), IsPerpendicular(). */
    bool IsZero(float epsilonSq = 1e-6f) const;

    /// Tests if this vector contains valid finite elements.
    /** @see IsNormalized(), IsZero(), IsPerpendicular(). */
    bool IsFinite() const;

    /// Tests if two vectors are perpendicular to each other.
    /** @see IsNormalized(), IsZero(), IsPerpendicular(), Equals(). */
    bool IsPerpendicular(const float2 &other, float epsilonSq = 1e-5f) const;

    /// Tests if two vectors are equal, up to the given epsilon.
    /** @see IsPerpendicular(). */
    bool Equals(const float2 &other, float epsilon = 1e-3f) const;
    bool Equals(float x, float y, float epsilon = 1e-3f) const;

#if defined(MATH_ENABLE_STL_SUPPORT)
    /// Returns "(x, y)".
    std::string ToString() const;

    /// Returns "x,y".
    std::string SerializeToString() const;

    /// Returns a string of C++ code that can be used to construct this object.
    std::string SerializeToCodeString() const;
    static float2 FromString(const std::string &str) { return FromString(str.c_str()); }
#endif

    /// Parses a string that is of form "x,y" or "(x,y)" or "(x;y)" or "x y" to a new float2.
    static float2 FromString(const char *str, const char **outEndStr = 0);

    /// @return x + y.
    float SumOfElements() const;
    /// @return x * y.
    float ProductOfElements() const;
    /// @return (x+y)/2.
    float AverageOfElements() const;
    /// @return Min(x, y).
    float MinElement() const;
    /// Returns the index that has the smallest value in this vector.
    int MinElementIndex() const;
    /// @return Max(x, y).
    float MaxElement() const;
    /// Returns the index that has the smallest value in this vector.
    int MaxElementIndex() const;
    /// @return float2(|x|, |y|).
    float2 Abs() const;
    /// @return float2(-x, -y).
    float2 Neg() const;
    /// @return float2(1/x, 1/y). */
    float2 Recip() const;
    /// Each element that is larger than ceil is replaced by ceil.
    float2 Min(float ceil) const;
    /// Each element that is larger than ceil is replaced by ceil.
    float2 Min(const float2 &ceil) const;
    /// Each element that is smaller than floor is replaced by floor.
    float2 Max(float floor) const;
    /// Each element that is smaller than floor is replaced by floor.
    float2 Max(const float2 &floor) const;
    /// Returns a vector that has floor <= this[i] <= ceil for each element.
    float2 Clamp(float floor, float ceil) const;
    /// Limits each element of this vector between the corresponding elements.
    float2 Clamp(const float2 &floor, const float2 &ceil) const;
    /// Limits each element of this vector in the range [0, 1].
    float2 Clamp01() const;

    /// Computes the distance between this and the given float2.
    float Distance(const float2 &point) const;

    /// Computes the squared distance between this and the given point.
    float DistanceSq(const float2 &point) const;

    /// Computes the dot product of this and the given vector.
    /** @return x*v.x + y*v.y.
        @see AngleBetween(), ProjectTo(), ProjectToNorm(), Perp(), PerpDot(). */
    float Dot(const float2 &v) const;

    /// Returns a vector that rotated this vector 90 degrees CW. (Rotated90CW)
    float2 Perp() const;

    /// Computes the perp-dot product. this->Dot(rhs.Perp())
    float PerpDot(const float2 &rhs) const;

    /// Rotates this vector 90 degrees CW.
    void Rotate90CW();

    /// Returns a vector that rotated this vector 90 degrees CW. (y, -x)
    float2 Rotated90CW() const;

    /// Rotates this vector 90 degrees counterclock-wise .
    void Rotate90CCW();

    /// Returns a vector that rotated this vector 90 degrees CCW. (-y, x)
    float2 Rotated90CCW() const;

    /// Projects this vector onto the given unnormalized direction vector.
    /** @param direction The direction vector to project this vector onto.
            This function will normalize this vector, so you can pass in an unnormalized vector.
        @see ProjectToNorm(). */
    float2 ProjectTo(const float2 &direction) const;

    /// Projects this vector onto the given normalized direction vector.
    /** @param direction The vector to project onto. This vector must be normalized.
        @see ProjectTo(). */
    float2 ProjectToNorm(const float2 &direction) const;

    /// Returns the angle between this vector and the specified vector, in radians.
    /** @note This function takes into account both vectors can be unnormalized, and normalizes the computations.
            If you are computing the angle between two normalized vectors, it is better to use AngleBetweenNorm().
        @see AngleBetweenNorm(). */
    float AngleBetween(const float2 &other) const;

    /// Returns the angle between this vector and the specified normalized vector, in radians.
    /** @param other The direction vector to compute the angle against.
        @note Both vectors must be normalized to call this function.
        @see AngleBetween(). */
    float AngleBetweenNorm(const float2 &other) const;

    /// Breaks this vector down into parallel and perpendicular components with respect to the given direction.
    /** @param direction The direction the decomposition is to be computed. This vector must be normalized.
        @param parallel [out] Receives the part of this vector that is parallel to the given direction vector.
        @param perpendicular [out] Receives the part of this vector that is perpendicular to the given direction vector. */
    void Decompose(const float2 &direction, float2 &parallel, float2 &perpendicular) const;

    /// Linearly interpolates between this and the vector b.
    /** @param b The target endpoint to lerp towards to.
        @param t The interpolation weight, in the range [0, 1].
        @return Lerp(b, t) returns (1-t)*this + t*b. */
    float2 Lerp(const float2 &b, float t) const;
    /// a.Lerp(b, t).
    static float2 Lerp(const float2 &a, const float2 &b, float t);

    /// Makes the given vectors linearly independent. (Gram-Schmidt procedure)
    /** @note If any of the input vectors is zero, then the resulting set of vectors cannot be made orthogonal.
        @see AreOrthogonal(), Orthonormalize(), AreOrthonormal(). */
    static void Orthogonalize(const float2 &a, float2 &b);

    /// Returns true if the given vectors are orthogonal to each other.
    /** @see Orthogonalize(), Orthonormalize(), AreOrthonormal(). */
    static bool AreOrthogonal(const float2 &a, const float2 &b, float epsilon = 1e-3f);

    /// Makes the given vectors linearly independent and normalized in length. (Gram-Schmidt procedure)
    /** Both a and be will be normalized.
        @note If either of the input vectors is zero, then the resulting set of vectors cannot be made orthonormal.
        @see Orthogonalize(), AreOrthogonal(), AreOrthonormal(). */
    static void Orthonormalize(float2 &a, float2 &b);

    /// Tests if the triangle a->b->c is oriented counter-clockwise.
    //  Point C lies to the left of the directed line AB.
    static bool OrientedCCW(const float2 &a, const float2 &b, const float2 &c);

#ifdef MATH_ENABLE_STL_SUPPORT
    /// Computes the 2D convex hull of the given point set.
    /* @see ConvexHullInPlace */
    static void ConvexHull(const float2 *pts, int npts, std::vector<float2> &convex_hull);
#endif

    /// Computes the 2D convex hull of the given point set, in-place.
    /** This version of the algorithm works in-place, meaning that when the algorithm finishes,
        pts will contain the list of the points on the convex hull.
        @note As a convention, the convex hull winds counter-clockwise when graphed in the xy plane where
            +x points to the right and +y points up. That is, walking along the polylist
            intArray[0] -> pts[1] -> pts[2] -> ... -> pts[npts-1] -> pts[0] performs
            a counter-clockwise tour.
        @param pts [in, out] A pointer to an array of npts float2 points that represent a point cloud. This
            array will be rewritten to contain the convex hull of the original point set.
        @return The number of points on the convex hull, i.e. the number of elements used in pts after the operation.
        @see ConvexHull(). */
    static int ConvexHullInPlace(float2 *pts, int npts);

    /// Tests whether a 2D convex hull contains the given point.
    /** @param convexHull [in] A pointer to an array of points in the convex hull.
        @param numPointsInConvexHull The number of elements in the array convexHull.
        @param point The target point to test. */
    static bool ConvexHullContains(const float2 *convexHull, int numPointsInConvexHull, const float2 &point);

    /// Computes the minimum-area rectangle that bounds the given point set. [noscript]
    /** Implementation adapted from Christer Ericson's Real-time Collision Detection, p.111.
        @param pts [in] A pointer to an array of points to process.
        @param npts The number of elements in the array pointed to by pts.
        @param center [out] This variable will receive the center point of the rectangle.
        @param uDir [out] This variable will receive a normalized direction vector pointing one of the side directionss of the rectangle.
        @param vDir [out] This variable will receive a normalized direction vector pointing the other side direction of the rectangle.
        @param minU [out] Receives the minimum extent of the processed point set along the u direction.
        @param maxU [out] Receives the maximum extent of the processed point set along the u direction.
        @param minV [out] Receives the minimum extent of the processed point set along the v direction.
        @param maxV [out] Receives the maximum extent of the processed point set along the v direction.
        @note This algorithm runs in O(n^2) time to the number of points in the input.
        @note For best performance, the input point array should contain only the points in the convex hull of the point set. This algorithm
            does not compute the convex hull for you.
        @return The area of the resulting rectangle. */
    static float MinAreaRectInPlace(float2 *pts, int npts, float2 &center, float2 &uDir, float2 &vDir, float &minU, float &maxU, float &minV, float &maxV);

    /** @note Due to static data initialization order being undefined in C++, do NOT use this
            member to initialize other static data in other compilation units! */
    static const float2 zero;  // (0, 0)
    static const float2 one;  // (1, 1)
    static const float2 unitX;  // (1, 0)
    static const float2 unitY;  // (0, 1)
    static const float2 nan;  // (NaN, NaN), Never compare a float2 to this value!
    static const float2 inf;  // (+infinity, +infinity)
};

#ifdef MATH_ENABLE_STL_SUPPORT
/// Prints this float2 to the given stream.
std::ostream &operator <<(std::ostream &out, const float2 &rhs);
#endif

float2 operator *(float scalar, const float2 &rhs);

#ifdef MATH_ENABLE_UNCOMMON_OPERATIONS
inline float2 operator /(float scalar, const float2 &rhs) { return float2::FromScalar(scalar) / rhs; }
#endif

inline float Dot(const float2 &a, const float2 &b) { return a.Dot(b); }
inline float2 Abs(const float2 &a) { return a.Abs(); }
inline float Length(const float2 &a) { return a.Length(); }
inline float Distance(const float2 &a, const float2 &b) { return a.Distance(b); }
inline float2 Min(const float2 &a, const float2 &b) { return a.Min(b); }
inline float2 Max(const float2 &a, const float2 &b) { return a.Max(b); }
inline float2 Clamp(const float2 &a, float floor, float ceil) { return a.Clamp(floor, ceil); }
inline float2 Clamp(const float2 &a, const float2 &floor, const float2 &ceil) { return a.Clamp(floor, ceil); }
inline float2 Clamp01(const float2 &a) { return a.Clamp01(); }
inline float2 Lerp(const float2 &a, const float2 &b, float t) { return a.Lerp(b, t); }

inline float2 Perp2D(const float2 &v) { return v.Perp(); }
float2 Mul2D(const float3x3 &transform, const float2 &v);
float2 MulPos2D(const float3x4 &transform, const float2 &v);
float2 MulDir2D(const float3x4 &transform, const float2 &v);
float2 MulPos2D(const float4x4 &transform, const float2 &v);
float2 MulDir2D(const float4x4 &transform, const float2 &v);

template<typename T>
int float2_ConvexHullInPlace(T *p, int n);

template<typename T>
bool float2_ConvexHullContains(T *hull, int n, const T &point);

MATH_END_NAMESPACE
