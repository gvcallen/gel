#include "gel/Core.h"
#include "gel/GroundStation.h"
#include "gel/SpaceTime.h"

#include <SoftwareSerial.h>

#define LOOKUP_MAG_DEC 0

namespace gel
{

Error telemetryReceived(gel::span<uint8_t> message)
{
    Serial.println("Telemetry received! Message = '");
    Serial.print((const char*)message.data());
    Serial.println("' ");

    return Error::None;
}

Error GroundStation::begin(GroundStationConfig config, GroundStationPins pins)
{
    Error err;

    this->trackingConfig = config.trackingConfig;

    if (err = gps.begin(pins.gps))
        return err;

    unsigned long gpsStartTime = millis(), timeout = 5000;

    #if (LOOKUP_MAG_DEC == 1)
    bool gotMagDec = false;
    while (!gotMagDec && millis() - gpsStartTime < timeout)
    {
        gps.update();
        gotMagDec = gps.getMagneticDeclination().has_value();
    }

    if (!gotMagDec)
        return gel::Error::Timeout;
    
    float magDec = gps.getMagneticDeclination().value();
    #else
    float magDec = GEL_RADIANS(-26.0);
    #endif

    // We want the mount to point east when the azimuthal angle is set to zero.
    // We therefore offset the 
    float deltaNorth = config.trackingConfig.magneticNorthDeltaAzAngle + magDec;
    float deltaEast = deltaNorth - GEL_RADIANS(90.0);
    config.mount.azimuthalAngleOffset = -deltaEast;

    if (err = mount.begin(pins.mount, config.mount))
        return err;

    if (err = radio.begin(pins.radio, config.radio))
        return err;

    if (err = link.begin(radio, config.link))
        return err;

    link.setTelemetryCallback(telemetryReceived);
    lastPositionUpdate = millis();

    return Error::None;
}

Error GroundStation::update()
{   
    gel::Error err;
    
    if (err = updatePosition())
        return err;

    if (err = gps.update())
        return err;

    return Error::None;

    // return link.update();
}

Error GroundStation::updatePosition()
{
    if ((millis() - lastPositionUpdate) < trackingConfig.updateInterval)
        return Error::None;

    lastPositionUpdate = millis();
    
    if (trackingFlags & TrackingFlags::EstimatedLocation)
        return updatePositionEstimatedLocation();
    
    return Error::None;
}

Error GroundStation::updatePositionEstimatedLocation()
{
    uint64_t currentEpochSeconds = getCurrentSecondsSinceEpoch();
    uint64_t location1Seconds = prevEstimatedLocation.secondsSinceEpoch;
    uint64_t location2Seconds = currentEstimatedLocation.secondsSinceEpoch;
    if (currentEpochSeconds > location2Seconds || location1Seconds == location2Seconds)
    {
        return pointAt(currentEstimatedLocation.location);
    }
    else if (currentEpochSeconds < location1Seconds)
    {
        return pointAt(prevEstimatedLocation.location);
    }
    else
    {
        float weight = (float)(currentEpochSeconds - location1Seconds) / (float)(location2Seconds - location1Seconds);
        return pointBetween(prevEstimatedLocation.location, currentEstimatedLocation.location, weight);
    }

    return Error::None;
}

Error GroundStation::calibrate()
{
    mount.calibrate();
    delay(1000);
    // mount.setAzimuthalAngle(GEL_RADIANS(90.0));

    // while (1)
    // {
        // mount.setAzimuthalAngle(GEL_RADIANS(60.0));
        // delay(2000);
        // mount.setAzimuthalAngle(GEL_RADIANS(30.0));
        // delay(2000);
        // mount.setAzimuthalAngle(GEL_RADIANS(60.0));
        // delay(2000);
        // mount.setAzimuthalAngle(GEL_RADIANS(90.0));
        // delay(2000);
    // }

    return Error::None;
}

Error GroundStation::returnToStart()
{
    trackingFlags = TrackingFlags::None;
    return mount.returnToStart();
}

Error GroundStation::returnToStow()
{
    trackingFlags = TrackingFlags::None;
    return mount.returnToStow();
}

Error GroundStation::pointAt(const GeoLocation& location)
{
    return pointAtCartesian(location.toCartesian(this->projectionOrigin));
}

// Weight of zero is all the way to location1
Error GroundStation::pointBetween(const GeoLocation& location1, const GeoLocation& location2, float weight)
{   
    auto pt1 = location1.toCartesian(this->projectionOrigin);
    auto pt2 = location2.toCartesian(this->projectionOrigin);
    auto ptBetween = pt1 * (1.0 - weight) + pt2 * (weight);

    return pointAtCartesian(ptBetween);
}

Error GroundStation::pointAtCartesian(const Vec3f& pt)
{
    auto geoLocation = gps.getGeoLocation();
    if (!geoLocation.has_value())
        return Error::Internal;
    
    auto ptGs = geoLocation.value().toCartesian(this->projectionOrigin);
    gel::Vec3f delta = pt - ptGs;
    auto boresight = normalize(delta);

    return mount.setBoresight(boresight);
}

Error GroundStation::addEstimatedLocation(const GeoInstant& estimatedLocation)
{
    if (!(trackingFlags & TrackingFlags::EstimatedLocation))
    {
        prevEstimatedLocation = currentEstimatedLocation = estimatedLocation;
        this->trackingFlags |= TrackingFlags::EstimatedLocation;
    }
    else
    {
        if (estimatedLocation.secondsSinceEpoch < currentEstimatedLocation.secondsSinceEpoch)
            return Error::Outdated;
        
        prevEstimatedLocation = currentEstimatedLocation;
        currentEstimatedLocation = estimatedLocation;
    }
    
    return Error::None;
}

Error GroundStation::addKnownLocation(const GeoInstant& knownLocation)
{
    this->trackingFlags |= TrackingFlags::KnownLocation;
    this->knownLocation = knownLocation;

    return Error::None;
}

uint64_t GroundStation::getCurrentSecondsSinceEpoch()
{
    return gps.getCurrentSecondsSinceEpoch().value_or(0);
}

Error GroundStation::scanConical(Vec3f estimatedDirection, float conicalAngle)
{
    constexpr double deltaThetaDegrees = 5.0;
    
    constexpr unsigned int numObs = 360.0 / deltaThetaDegrees;
    // double powerObs[numObs];

    for (unsigned int i = 0; i < numObs; i++)
    {
        double theta = ((double)i / numObs) * GEL_PI_TIMES_2;
        mount.setConical(estimatedDirection, conicalAngle, theta);

        float rssi = radio.getRssi(false);
        Serial.println(String("RSSI = ") + String(rssi) + String(" dB at ") + String(GEL_DEGREES(theta)) + String(" degrees"));
    }
    
    return Error::None;
}

Error GroundStation::scanBF()
{
    float elevationStep = trackingConfig.estimatedBeamwidth / trackingConfig.numBeamwidthScanSegments;
    float elevationAngle = mount.getConfig().elevationAngleBounds.min;

    mount.setAzimuthElevation(0.0, elevationAngle);

    float bestRssi = radio.getRssi(false);
    float bestAzimuth = 0.0;
    float bestElevation = elevationAngle;
    
    bool scanAzBackwards = false;
    while (elevationAngle < GEL_RADIANS(90.0))
    {
        float testRssi, testAzimuth;
        scanSegment(testRssi, testAzimuth, elevationAngle, scanAzBackwards);

        if (testRssi > bestRssi)
        {
            bestRssi = testRssi;
            bestAzimuth = testAzimuth;
            bestElevation = elevationAngle;
        }

        elevationAngle += elevationStep;
        scanAzBackwards = !scanAzBackwards;
    }
    
    mount.setAzimuthElevation(bestAzimuth, bestElevation);

    return Error::None;
}

Error GroundStation::scanSegment(float& bestRssi, float& bestAzimuth, float elevationAngle, bool scanAzBackwards)
{
    bestRssi = radio.getRssi(false);
    bestAzimuth = mount.getAzimuthalAngle();
    uint32_t numSamples = trackingConfig.numAzimuthScanSamples;
    
    float azStep = GEL_PI_TIMES_2 / numSamples;
    if (scanAzBackwards)
        azStep *= -1.0;

    mount.setElevationAngle(elevationAngle);

    float testRssi, testAzimuth;
    for (uint32_t i = 0; i < numSamples; i++)
    {
        testAzimuth += azStep;
        mount.setAzimuthalAngle(testAzimuth);
        testRssi = radio.getRssi();

        Serial.print("Rssi = "); Serial.print(testRssi); Serial.println(" dB");
        
        if (testRssi > bestRssi)
        {
            bestRssi = testRssi;
            bestAzimuth = testAzimuth;
        }
    }

    return Error::None;
}

} // namespace gel