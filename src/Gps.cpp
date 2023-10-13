#include "gel/Core.h"
#include "gel/Gps.h"
#include "gel/MagDec.h"

#include <Stream.h>
#include <SoftwareSerial.h>

namespace gel
{

Error Gps::begin(GpsPins& pins)
{
    auto softwareSerial = new SoftwareSerial(pins.rx, pins.tx);
    softwareSerial->begin(9600);
    this->gpsSerial = softwareSerial;

    return Error::None;
}

Error Gps::update()
{
    while (gpsSerial->available() > 0)
    {
        char c = gpsSerial->read();
        bool encodeResult = tinyGps.encode(c);
        if (!encodeResult)
            return Error::BadCommunication;
    }
    
    return Error::None;
}

expected<double, Error> Gps::getLatitude()
{
    if (!tinyGps.location.isValid())
        return make_unexpected<double>(Error::InvalidState);

    return tinyGps.location.lat();
}

expected<double, Error> Gps::getLongitude()
{
    if (!tinyGps.location.isValid())
        return make_unexpected<double>(Error::InvalidState);

    return tinyGps.location.lng();
}

expected<double, Error> Gps::getAltitude()
{
    if (!tinyGps.altitude.isValid())
        return make_unexpected<double>(Error::InvalidState);

    return tinyGps.altitude.meters();
}

expected<float, Error> Gps::getMagneticDeclination()
{
    auto lng = getLongitude();
    if (!lng)
        return make_unexpected<float>(lng.error());

    auto lat = getLatitude();
    if (!lat)
        return expected<float, Error>{unexpected<Error>{lat.error()}};

    auto year = getYear();
    
    return gel::getMagneticDeclination(lat.value(), lng.value(), year.value_or(2014));
}

expected<uint8_t, Error> Gps::getSecond()
{
    if (!tinyGps.time.isValid())
        return make_unexpected<uint8_t>(Error::InvalidState);

    return tinyGps.time.second();
}

expected<uint8_t, Error> Gps::getMinute()
{
    if (!tinyGps.time.isValid())
        return make_unexpected<uint8_t>(Error::InvalidState);

    return tinyGps.time.minute();
}

expected<uint8_t, Error> Gps::getHour()
{
    if (!tinyGps.time.isValid())
        return make_unexpected<uint8_t>(Error::InvalidState);

    return tinyGps.time.hour();
}

expected<uint8_t, Error> Gps::getDay()
{
    if (!tinyGps.date.isValid())
        return make_unexpected<uint8_t>(Error::InvalidState);

    return tinyGps.date.day();
}

expected<uint8_t, Error> Gps::getMonth()
{
    if (!tinyGps.date.isValid())
        return make_unexpected<uint8_t>(Error::InvalidState);

    return tinyGps.date.month();
}

expected<uint16_t, Error> Gps::getYear()
{
    if (!tinyGps.date.isValid())
        return make_unexpected<uint16_t>(Error::InvalidState);

    return tinyGps.date.year();
}

expected<uint64_t, Error> Gps::getCurrentSecondsSinceEpoch()
{
    if (!tinyGps.time.isValid())
        return make_unexpected<uint64_t>(Error::InvalidState);

    TimeInfo timeInfo;
    timeInfo.sec = getSecond().value();
    timeInfo.min = getMinute().value();
    timeInfo.hour = getHour().value();
    timeInfo.day = getDay().value();
    timeInfo.mon = getMonth().value();
    timeInfo.year = getYear().value();
    
    // return timeInfo.getSecondsSinceEpoch();
    return timeInfo.getSecondsSinceEpoch() + tinyGps.time.age() / 1000;
}

expected<uint64_t, Error> Gps::getReceivedSecondsSinceEpoch()
{
    if (!tinyGps.time.isValid())
        return make_unexpected<uint64_t>(Error::InvalidState);

    TimeInfo timeInfo;
    timeInfo.sec = getSecond().value();
    timeInfo.min = getMinute().value();
    timeInfo.hour = getHour().value();
    timeInfo.day = getDay().value();
    timeInfo.mon = getMonth().value();
    timeInfo.year = getYear().value();

    return timeInfo.getSecondsSinceEpoch();    
}

expected<GeoLocation, Error> Gps::getGeoLocation()
{
    if (!tinyGps.date.isValid())
        return make_unexpected<GeoLocation>(Error::InvalidState);
        
    GeoLocation location;
    location.lat = getLatitude().value();
    location.lng = getLongitude().value();
    location.altitude = getAltitude().value();

    return location;
}

expected<GeoInstant, Error> Gps::getGeoInstant()
{
    if (!tinyGps.date.isValid() || !tinyGps.time.isValid())
        return make_unexpected<GeoInstant>(Error::InvalidState);
        
    GeoInstant instant(getLatitude().value(),
                       getLongitude().value(),
                       getAltitude().value(),
                       getReceivedSecondsSinceEpoch().value());

    return instant;
}

} // namespace gel
