#pragma once

#include <Arduino.h>

#include <etl/
#include <etl/optional.h>
#include <etl/span.h>

#include "Error.h"

namespace gel
{

#define NUM_ELEMS(a) (sizeof(a)/sizeof 0[a])
#define DEBUG_VARIABLE(x) Serial.print(#x " = "); Serial.println(x);

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
using etl::span;

// test

} // namespace gel