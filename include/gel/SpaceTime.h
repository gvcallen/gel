#pragma once

#include <gel/Math.h>

namespace gel {

// In UTC
struct TimeInfo
{
    int	sec;
    int	min;
    int	hour;
    int	day; // of the month
    int	mon;
    int	year;

    uint64_t getSecondsSinceEpoch();
};

struct GeoLocation
{
    float lat;
    float lng;
    float altitude;

    Vec3f toCartesian(const GeoLocation& origin) const;
};

struct GeoInstant
{
public:
    GeoInstant();
    GeoInstant(float lat, float lng, float altitude, uint64_t secondsSinceEpoch);
    GeoInstant(float lat, float lng, float altitude, TimeInfo timeInfo);
public:
    uint64_t secondsSinceEpoch; // in UTC from January 1st 1970
    GeoLocation location;
};

} // namespace gel