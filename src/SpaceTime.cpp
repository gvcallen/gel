#include "gel/SpaceTime.h"

#include <gel/WGS84.h>
#include <time.h>

namespace gel
{

uint64_t TimeInfo::getSecondsSinceEpoch()
{
    struct tm time_info;

    time_info.tm_sec = this->sec;
    time_info.tm_min = this->min;
    time_info.tm_hour = this->hour;
    time_info.tm_mday = this->day;
    time_info.tm_mon = this->mon;
    time_info.tm_year = this->year;

    return ::mktime(&time_info);
}

GeoInstant::GeoInstant()
{
    this->secondsSinceEpoch = ::time(NULL);
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