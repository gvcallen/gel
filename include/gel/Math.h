#pragma once

#include "math.h"

#include <BasicLinearAlgebra.h>

#define GEL_PI_OVER_180 (M_PI / 180.0)
#define GEL_180_OVER_PI (180.0 / M_PI)
#define GEL_PI_TIMES_2 (2.0 * M_PI)

#define GEL_DEGREES(x) ((x) * GEL_180_OVER_PI)
#define GEL_RADIANS(x) ((x) * GEL_PI_OVER_180)

namespace gel
{

template <typename Real>
struct Bounds1
{
    Real min;
    Real max;
};

template <typename Real, size_t rows, size_t cols>
struct Matrix
{
    Real data[rows * col];
}

template <typename Real>
struct Vec3
{
    Real x, y, z;

    Vec3<Real>();
    Vec3<Real>(Real const x, Real const y, Real const z);

    Vec3<Real> operator+() const;
    Vec3<Real> operator-() const;
    Vec3<Real> operator+(Vec3<Real> const & summand) const;
    Vec3<Real> operator-(Vec3<Real> const & subtrahend) const;
    Vec3<Real> operator*(Real const operand) const;
    Vec3<Real> operator/(Real const divisor) const;
    Vec3<Real> & operator+=(Vec3<Real> const & summand);
    Vec3<Real> & operator-=(Vec3<Real> const & subtrahend);
    Vec3<Real> & operator*=(Real const operand);
    Vec3<Real> & operator/=(Real const divisor);
};

template <typename Real>
Vec3<Real> operator*(const float scalar, const Vec3<Real>& vec)
{
    return vec * scalar;
}

template <typename Real>
Vec3<Real>::Vec3()
    : Vec3<Real>(0.0, 0.0, 0.0)
{
    // intentionally empty
}

template <typename Real>
Vec3<Real>::Vec3(Real const x, Real const y, Real const z)
    : x(x)
    , y(y)
    , z(z)
{
    // intentionally empty
}

template <typename Real>
Vec3<Real> Vec3<Real>::operator+() const
{
    return *this;
}

template <typename Real>
Vec3<Real> Vec3<Real>::operator-() const
{
    return Vec3<Real>{-x,
                -y,
                -z};
}

template <typename Real>
Vec3<Real> Vec3<Real>::operator+(Vec3<Real> const & summand) const
{
    return Vec3<Real>{x + summand.x,
                y + summand.y,
                z + summand.z};
}

template <typename Real>
Vec3<Real> Vec3<Real>::operator-(Vec3<Real> const & subtrahend) const
{
    return Vec3<Real>{x - subtrahend.x,
                y - subtrahend.y,
                z - subtrahend.z};
}

template <typename Real>
Vec3<Real> Vec3<Real>::operator*(Real const operand) const
{
    return Vec3<Real>{x * operand,
                y * operand,
                z * operand};
}

template <typename Real>
Vec3<Real> Vec3<Real>::operator/(Real const divisor) const
{
    return Vec3<Real>{x / divisor,
                y / divisor,
                z / divisor};
}

template <typename Real>
Vec3<Real> & Vec3<Real>::operator+=(Vec3<Real> const & summand)
{
    x += summand.x;
    y += summand.y;
    z += summand.z;
    return *this;
}

template <typename Real>
Vec3<Real> & Vec3<Real>::operator-=(Vec3<Real> const & subtrahend)
{
    x -= subtrahend.x;
    y -= subtrahend.y;
    z -= subtrahend.z;
    return *this;
}

template <typename Real>
Vec3<Real> & Vec3<Real>::operator*=(Real const operand)
{
    x *= operand;
    y *= operand;
    z *= operand;
    return *this;
}

template <typename Real>
Vec3<Real> & Vec3<Real>::operator/=(Real const divisor)
{
    x /= divisor;
    y /= divisor;
    z /= divisor;
    return *this;
}

using Bounds1f = Bounds1<float>;
using Vec3f = Vec3<float>;
using Mat3f = BLA::Matrix<3, 3, float>;

Vec3f cross(Vec3f& v1, Vec3f& v2);
float length(Vec3f& v);
Vec3f normalize(Vec3f& v);

float normalizeAngle2PI(float x);

Vec3f azimuthElevationToCartesian(float az, float el, float r);

void cartesianToAzimuthElevation(float* az, float* el, float* r, Vec3f& pt);


} // namespace gel