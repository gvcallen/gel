#pragma once

#include "gel/Core.h"
#include "gel/SpaceTime.h"

#include <Arduino.h>
#include <TinyGPS++.h>

namespace gel
{

struct GpsPins
{
    uint8_t tx;
    uint8_t rx;
};

class Gps
{
public:
    Gps() = default;

    Error begin(GpsPins &pins);
    Error update();
    expected<double, Error> getLatitude();
    expected<double, Error> getLongitude();
    expected<double, Error> getAltitude();
    expected<float, Error> getMagneticDeclination();
    expected<uint8_t, Error> getSecond();
    expected<uint8_t, Error> getMinute();
    expected<uint8_t, Error> getHour();
    expected<uint8_t, Error> getDay();
    expected<uint8_t, Error> getMonth();
    expected<uint16_t, Error> getYear();

    expected<uint64_t, Error> getCurrentSecondsSinceEpoch();
    expected<uint64_t, Error> getReceivedSecondsSinceEpoch();
    expected<GeoInstant, Error> getGeoInstant();
    expected<GeoLocation, Error> getGeoLocation();

private:
    TinyGPSPlus tinyGps{};
    Stream* gpsSerial;
};

} // namespace gel