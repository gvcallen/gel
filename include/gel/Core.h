#pragma once

#include <Arduino.h>

#include "Error.h"

#ifdef GEL_NO_STL
#define ETL_NO_STL
#endif

#include <etl/optional.h>
#include <etl/span.h>
#include <etl/expected.h>
#include <etl/vector.h>

namespace gel
{

using namespace etl;

#define NUM_ELEMS(a) (sizeof(a)/sizeof 0[a])
#define DEBUG_VARIABLE(x) Serial.print(#x " = "); Serial.println(x);

#define PI_OVER_180 (M_PI / 180.0)
#define PI_TIMES_2 (2.0 * M_PI)

double normalizeAngle2PI(double x);

struct Bounds1f
{
    float min;
    float max;
};

struct Bounds1d
{
    double min;
    double max;
};

struct Vec3f
{
    float x, y, z;

    Vec3f();
    Vec3f(float const x, float const y, float const z);

    Vec3f operator+() const;
    Vec3f operator-() const;
    Vec3f operator+(Vec3f const & summand) const;
    Vec3f operator-(Vec3f const & subtrahend) const;
    Vec3f operator*(float const operand) const;
    Vec3f operator/(float const divisor) const;
    Vec3f & operator+=(Vec3f const & summand);
    Vec3f & operator-=(Vec3f const & subtrahend);
    Vec3f & operator*=(float const operand);
    Vec3f & operator/=(float const divisor);
};

template<typename T>
using optional = etl::optional<T>;

template<typename T, const size_t MAX_SIZE>
using vector = etl::vector<T, MAX_SIZE>;

template<typename T>
using span = etl::span<T>;

using etl::nullopt;

} // namespace gel