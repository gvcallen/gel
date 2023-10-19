#include "gel/SpaceTime.h"
// #include "gel/Core.h"
#include "gel/WGS84.h"
#include <TimeLib.h>

#include <Arduino.h>

namespace gel
{

uint64_t TimeInfo::getSecondsSinceEpoch()
{
    tmElements_t timeEl;
    timeEl.Second = this->sec;
    timeEl.Minute = this->min;
    timeEl.Hour = this->hour;
    timeEl.Day = this->day;
    timeEl.Month = this->mon;
    timeEl.Year = this->year - 1970;

    return makeTime(timeEl);
}

GeoInstant::GeoInstant()
{
    this->secondsSinceEpoch = 0;
    this->location.altitude = 0.0;
    this->location.lat = 0.0;
    this->location.lng = 0.0;
}

GeoInstant::GeoInstant(float lat, float lng, float altitude, uint64_t secondsSinceEpoch)
{
    this->location.lat = lat;
    this->location.lng = lng;
    this->location.altitude = altitude;
    this->secondsSinceEpoch = secondsSinceEpoch;
}

GeoInstant::GeoInstant(float lat, float lng, float altitude, TimeInfo timeInfo)
{
    this->location.lat = lat;
    this->location.lng = lng;
    this->location.altitude = altitude;
    this->secondsSinceEpoch = timeInfo.getSecondsSinceEpoch();
}

Vec3f GeoLocation::toCartesian(const GeoLocation& origin) const
{
    std::array<double, 2> reference{origin.lat, origin.lng};
    std::array<double, 2> position{this->lat, this->lng};
    std::array<double, 2> cartesianPosition = wgs84::toCartesian(reference, position);
    
    Vec3f result = {(float)cartesianPosition[0], (float)cartesianPosition[1], this->altitude - origin.altitude};
    return result;
}

} // namespace gel