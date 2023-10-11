#include "gel/Core.h"
#include "gel/GroundStation.h"
#include "gel/SpaceTime.h"

#include <SoftwareSerial.h>

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

    if (err = mount.begin(pins.mount, config.mount))
        return err;

    if (err = radio.begin(pins.radio, config.radio))
        return err;

    if (err = link.begin(radio, config.link))
        return err;

    if (err = gps.begin(pins.gps))
        return err;

    link.setTelemetryCallback(telemetryReceived);
    lastPositionUpdate = getCurrentSecondsSinceEpoch();

    return Error::None;
}

Error GroundStation::update()
{   
    gel::Error err;
    
    if (err = updatePosition())
        return err;
    
    return link.update();
}

Error GroundStation::updatePosition()
{
    uint64_t currentEpochSeconds = getCurrentSecondsSinceEpoch();
    if ((currentEpochSeconds - lastPositionUpdate) < trackingConfig.updateInterval)
        return Error::None;
    
    if (trackingFlags & TrackingFlags::EstimatedLocation)
        updatePositionEstimatedLocation(currentEpochSeconds);
    
    return Error::None;
}

Error GroundStation::updatePositionEstimatedLocation(uint64_t currentEpochSeconds)
{
    if (currentEpochSeconds > currentEstimatedLocation.secondsSinceEpoch)
    {
        pointAt(currentEstimatedLocation.location);
    }
    else
    {
        uint64_t location1Seconds = prevEstimatedLocation.secondsSinceEpoch;
        uint64_t location2Seconds = currentEstimatedLocation.secondsSinceEpoch;

        float weight = (float)(currentEpochSeconds - location1Seconds) / (float)(location2Seconds - location1Seconds);
        pointBetween(prevEstimatedLocation.location, currentEstimatedLocation.location, weight);
    }

    return Error::None;
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
    gel::Vec3f ptVec = pt - ptGs;
    auto boresight = normalize(ptVec);

    return mount.setBoresight(boresight);
}

Error GroundStation::addEstimatedLocation(const GeoInstant& estimatedLocation)
{
    if (!(trackingFlags & TrackingFlags::EstimatedLocation))
    {
        prevEstimatedLocation = currentEstimatedLocation = estimatedLocation;
    }
    else
    {
        if (estimatedLocation.secondsSinceEpoch < currentEstimatedLocation.secondsSinceEpoch)
            return Error::Outdated;
        
        prevEstimatedLocation = currentEstimatedLocation;
        currentEstimatedLocation = estimatedLocation;
    }
    
    this->trackingFlags |= TrackingFlags::EstimatedLocation;
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
    return gps.getCurrentSecondsSinceEpoch();
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