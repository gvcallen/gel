#pragma once

#define GEL_PI_OVER_180 (M_PI / 180.0)
#define GEL_180_OVER_PI (180.0 / M_PI)
#define GEL_PI_TIMES_2 (2.0 * M_PI)

namespace gel
{

double normalizeAngle2PI(double x);

template <typename Real>
struct Bounds1
{
    Real min;
    Real max;
};


template <typename Real>
struct Vec2
{
    Real x, y, z;

    Vec2();
    Vec2(Real const x, Real const y, Real const z);

    Vec2 operator+() const;
    Vec2 operator-() const;
    Vec2 operator+(Vec2 const & summand) const;
    Vec2 operator-(Vec2 const & subtrahend) const;
    Vec2 operator*(Real const operand) const;
    Vec2 operator/(Real const divisor) const;
    Vec2 & operator+=(Vec2 const & summand);
    Vec2 & operator-=(Vec2 const & subtrahend);
    Vec2 & operator*=(Real const operand);
    Vec2 & operator/=(Real const divisor);
};

template <typename Real>
struct Vec3
{
    Real x, y;

    Vec3();
    Vec3(Real const x, Real const y);

    Vec3 operator+() const;
    Vec3 operator-() const;
    Vec3 operator+(Vec3 const & summand) const;
    Vec3 operator-(Vec3 const & subtrahend) const;
    Vec3 operator*(Real const operand) const;
    Vec3 operator/(Real const divisor) const;
    Vec3 & operator+=(Vec3 const & summand);
    Vec3 & operator-=(Vec3 const & subtrahend);
    Vec3 & operator*=(Real const operand);
    Vec3 & operator/=(Real const divisor);
};

Vec2::Vec2()
    : Vec2(0.0, 0.0)
{
    // intentionally empty
}

Vec2::Vec2(Real const x, Real const y)
    : x(x)
    , y(y)
{
    // intentionally empty
}

Vec2 Vec2::operator+() const
{
    return *this;
}

Vec2 Vec2::operator-() const
{
    return Vec2{-x, -y};
}

Vec2 Vec2::operator+(Vec2 const & summand) const
{
    return Vec2{x + summand.x,
                y + summand.y};
}

Vec2 Vec2::operator-(Vec2 const & subtrahend) const
{
    return Vec2{x - subtrahend.x,
                y - subtrahend.y};
}

Vec2 Vec2::operator*(Real const operand) const
{
    return Vec2{x * operand,
                y * operand};
}

Vec2 Vec2::operator/(Real const divisor) const
{
    return Vec2{x / divisor,
                y / divisor};
}

Vec2 & Vec2::operator+=(Vec2 const & summand)
{
    x += summand.x;
    y += summand.y;
    return *this;
}

Vec2 & Vec2::operator-=(Vec2 const & subtrahend)
{
    x -= subtrahend.x;
    y -= subtrahend.y;
    return *this;
}

Vec2 & Vec2::operator*=(Real const operand)
{
    x *= operand;
    y *= operand;
    return *this;
}

Vec2 & Vec2::operator/=(Real const divisor)
{
    x /= divisor;
    y /= divisor;
    return *this;
}

Vec3::Vec3()
    : Vec3(0.0, 0.0, 0.0)
{
    // intentionally empty
}

Vec3::Vec3(Real const x, Real const y, Real const z)
    : x(x)
    , y(y)
    , z(z)
{
    // intentionally empty
}

Vec3 Vec3::operator+() const
{
    return *this;
}

Vec3 Vec3::operator-() const
{
    return Vec3{-x,
                -y,
                -z};
}

Vec3 Vec3::operator+(Vec3 const & summand) const
{
    return Vec3{x + summand.x,
                y + summand.y,
                z + summand.z};
}

Vec3 Vec3::operator-(Vec3 const & subtrahend) const
{
    return Vec3{x - subtrahend.x,
                y - subtrahend.y,
                z - subtrahend.z};
}

Vec3 Vec3::operator*(Real const operand) const
{
    return Vec3{x * operand,
                y * operand,
                z * operand};
}

Vec3 Vec3::operator/(Real const divisor) const
{
    return Vec3{x / divisor,
                y / divisor,
                z / divisor};
}

Vec3 & Vec3::operator+=(Vec3 const & summand)
{
    x += summand.x;
    y += summand.y;
    z += summand.z;
    return *this;
}

Vec3 & Vec3::operator-=(Vec3 const & subtrahend)
{
    x -= subtrahend.x;
    y -= subtrahend.y;
    z -= subtrahend.z;
    return *this;
}

Vec3 & Vec3::operator*=(Real const operand)
{
    x *= operand;
    y *= operand;
    z *= operand;
    return *this;
}

Vec3 & Vec3::operator/=(Real const divisor)
{
    x /= divisor;
    y /= divisor;
    z /= divisor;
    return *this;
}

using Vec2f = Vec2<float>;
using Vec2d = Vec2<double>;
using Vec2f = Vec2<float>;
using Vec2d = Vec2<double>;
using Bounds1f = Bounds1<float>;
using Bounds1d = Bounds1<double>;

} // namespace gel