#pragma once

#include <Arduino.h>

#include "Error.h"
#include "Math.h"

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

template<typename T>
using optional = etl::optional<T>;

template<typename T, const size_t MAX_SIZE>
using vector = etl::vector<T, MAX_SIZE>;

template<typename T>
using span = etl::span<T>;

using etl::nullopt;

} // namespace gel