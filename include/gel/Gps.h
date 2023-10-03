#include <Arduino.h>
#include <TinyGPS++.h>

#include "gel/Core.h"

namespace gel
{

class Gps
{
    public:
        Gps() = default;

        Error begin(Stream *gpsSerial);
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

    private:
        TinyGPSPlus tinyGps{};
        Stream *gpsSerial = nullptr;
};

} // namespace gel