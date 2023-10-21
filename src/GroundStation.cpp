#include "gel/Core.h"
#include "gel/GroundStation.h"
#include "gel/SpaceTime.h"

#include <SoftwareSerial.h>

#define LOOKUP_MAG_DEC 0

namespace gel
{

Error GroundStation::begin(GroundStationConfig config, GroundStationPins pins)
{
    Error err;

    this->tracking = config.tracking;

    if (err = gps.begin(pins.gps))
        return err;

    #if (LOOKUP_MAG_DEC == 1)
    unsigned long gpsStartTime = millis(), timeout = 5000;
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
    float deltaNorth = tracking.magneticNorthDeltaAzAngle + magDec;
    float deltaEast = deltaNorth - GEL_RADIANS(90.0);
    config.mount.azimuthalAngleOffset = -deltaEast;

    if (err = mount.begin(pins.mount, config.mount))
        return err;

    if (err = radio.begin(pins.radio, config.radio))
        return err;

    if (err = link.begin(radio, config.link))
        return err;

    lastPositionUpdate = millis();

    return Error::None;
}

vector<Error, 3> GroundStation::update()
{   
    vector<Error, 3> errors;
    Error err;
    
    if (err = gps.update())
    {
        if (!err.message)
            err.message = "Error updating GPS";
        errors.push_back(err);
    }

    if (err = link.update())
    {
        if (!err.message)
            err.message = "Error updating communication link";
        errors.push_back(err);
    }

    if (err = updatePosition())
    {
        if (!err.message)
            err.message = "Error updating position";
        errors.push_back(err);
    }

    return errors;
}

Error GroundStation::updatePosition()
{
    if ((millis() - lastPositionUpdate) < tracking.updateInterval)
        return Error::None;

    lastPositionUpdate = millis();
    
    bool knownLocationUsed = false;
    if (trackingFlags & TrackingFlags::KnownLocation)
        return updatePositionKnownLocation(knownLocationUsed);
    
    if (!knownLocationUsed && trackingFlags & TrackingFlags::EstimatedLocation)
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

Error GroundStation::updatePositionKnownLocation(bool& knownLocationUsed)
{
    static gel::RunEvery run(1000);
    
    uint64_t positionAge = getCurrentSecondsSinceEpoch() - knownLocation.secondsSinceEpoch;
    if (positionAge > tracking.knownLocationTrustTimeout)
    {
        if (run)
            Serial.println("Position too old - discarding");
        knownLocationUsed = false;
        return Error::None;
    }

    if (run)
        Serial.println("Using position with age " + String(positionAge));

    knownLocationUsed = true;
    return pointAt(knownLocation.location, true);
}

Error GroundStation::calibrate()
{
    mount.calibrate();
    delay(200);
    mount.setAzimuthalAngle(GEL_RADIANS(90.0)); // point north

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

Error GroundStation::pointAt(const GeoLocation& location, bool lowPass)
{
    // Serial.print("Pointing at: ");
    // Serial.print("Lat = ");
    // Serial.print(location.lat, 5);
    // Serial.print(", Lng = ");
    // Serial.print(location.lng, 5);
    // Serial.print(", Alt = ");
    // Serial.println(location.altitude, 2);
    
    // Serial.print("Point from: ");
    // Serial.print("Lat = ");
    // Serial.print(gps.getLatitude().value(), 5);
    // Serial.print(", Lng = ");
    // Serial.print(gps.getLongitude().value(), 5);
    // Serial.print(", Alt = ");
    // Serial.println(gps.getAltitude().value(), 2);
    
    return pointAtCartesian(location.toCartesian(tracking.mapProjectionOrigin), lowPass);
}

// Weight of zero is all the way to location1
Error GroundStation::pointBetween(const GeoLocation& location1, const GeoLocation& location2, float weight)
{   
    auto pt1 = location1.toCartesian(tracking.mapProjectionOrigin);
    auto pt2 = location2.toCartesian(tracking.mapProjectionOrigin);
    auto ptBetween = pt1 * (1.0 - weight) + pt2 * (weight);

    return pointAtCartesian(ptBetween);
}

Error GroundStation::pointAtCartesian(const Vec3f& pt, bool lowPass)
{
    auto geoLocation = gps.getGeoLocation();
    if (!geoLocation.has_value())
        return Error::Internal;
    
    auto ptGs = geoLocation.value().toCartesian(tracking.mapProjectionOrigin);
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

Error GroundStation::scanBruteForce()
{
    Serial.println("Scanning " + String(tracking.numBeamwidthScanSegments) + " segments using beamwidth = " + String(GEL_DEGREES(tracking.estimatedBeamwidth)));
    float elevationStep = tracking.estimatedBeamwidth / tracking.numBeamwidthScanSegments;
    float elevationAngle = mount.getConfig().elevationAngleBounds.min;

    float bestRssi = radio.getRssi(false);
    float bestElevation = elevationAngle;
    float bestAzimuth;
    if (mount.getConfig().unconstrainedRotation)
        bestAzimuth = mount.getConfig().maxConstrainedRotations * GEL_RADIANS(360); // start azimuth at the edge of constrained limit
    else
        bestAzimuth = GEL_RADIANS(90.0); // start azimuth at north
    
    mount.setAzimuthElevation(bestAzimuth, bestElevation);
    Serial.println("Starting at az = " + String(GEL_DEGREES(bestAzimuth)) + " and el = " + String(GEL_DEGREES(bestElevation)));
    delay(1000);
    bool scanAzBackwards = false;
    
    while (elevationAngle <= GEL_RADIANS(90.0))
    {
        float testRssi, testAzimuth;
        Serial.println("Scanning segment el = " + String(GEL_DEGREES(elevationAngle)));
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
    uint32_t numSamples = tracking.numAzimuthScanSamples;
    
    float azStep = GEL_PI_TIMES_2 / numSamples;
    if (scanAzBackwards)
        azStep *= -1.0;

    mount.setElevationAngle(elevationAngle);

    float testRssi, testAzimuth;
    for (uint32_t i = 0; i < numSamples; i++)
    {
        mount.setAzimuthalAngle(testAzimuth);
        testRssi = radio.getRssi();

        Serial.print("Rssi = "); Serial.print(testRssi); Serial.println(" dB at az = " + String(GEL_DEGREES(testAzimuth)));
        
        if (testRssi > bestRssi)
        {
            bestRssi = testRssi;
            bestAzimuth = testAzimuth;
        }
        testAzimuth += azStep;
    }

    return Error::None;
}

} // namespace gel