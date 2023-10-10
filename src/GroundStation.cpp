#include "gel/Core.h"
#include "gel/GroundStation.h"

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

    this->antennaConfig = config.antennaConfig;

    // MOUNT SETUP
    // if (err = mount.begin(pins.mount, config.mount))
        // return err;
    // if (err = mount.calibrate())
        // return err;

    // RADIO SETUP
    if (err = radio.begin(pins.radio, config.radio))
        return err;


    // LINK SETUP
    if (err = link.begin(radio, config.link))
        return err;

    link.setTelemetryCallback(telemetryReceived);

    return Error::None;
}

Error GroundStation::update()
{   
    // return Error::None;
    return link.update();
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
    float elevationStep = antennaConfig.estimatedBeamwidth / antennaConfig.numBeamwidthScanSegments;
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
    uint32_t numSamples = antennaConfig.numAzimuthScanSamples;
    
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