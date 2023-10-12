#pragma once

#include "gel/Core.h"
#include "gel/Mount.h"
#include "gel/Radio.h"
#include "gel/Link.h"
#include "gel/Imu.h"
#include "gel/Gps.h"
#include "gel/SpaceTime.h"

namespace gel
{

struct TrackingConfig
{
    float estimatedBeamwidth = GEL_RADIANS(60.0f);          // The estimated beamwidth of the antenna, in radians
    uint32_t numBeamwidthScanSegments = 4;                  // The number of segments per beamwidth to scan
    uint32_t numAzimuthScanSamples = 20;                    // The number of signal strength samples to take per azimuthal scan
    
    uint32_t scanTimeout = 120;                             // The timeout, in seconds, before any scan function should stop and return
    uint32_t updateInterval = 1;                            // The update between locations when tracking, in seconds
};

struct GroundStationPins
{
    MountPins mount;
    RadioPins radio;
    ImuPins imu;
    GpsPins gps;
};

struct GroundStationConfig
{
    RadioConfig radio;
    LinkConfig link;
    MountConfig mount;
    TrackingConfig trackingConfig;
};

/*
    Class for a communication system ground station. The ground station contains the
    radio, communication link, motor pointing mount, and also contains any control-system
    functionality.
*/
class GroundStation
{
public:
    GroundStation() = default;
    
    Error begin(GroundStationConfig config, GroundStationPins pins);
    Error update();

    Radio& getRadio() { return radio; }
    Mount& getMount() { return mount; }

    Error calibrate() {mount.calibrate(); return Error::None; };

    Error addEstimatedLocation(const GeoInstant& estimatedLocation);
    Error addKnownLocation(const GeoInstant& knownLocation);
    Error pointAt(const GeoLocation& location);
    Error pointBetween(const GeoLocation& location1, const GeoLocation& location2, float weight);

    uint64_t getCurrentSecondsSinceEpoch();

private:   
    struct TrackingFlags
    {
        static constexpr uint32_t None = 0x00;
        static constexpr uint32_t EstimatedLocation = 0x01;
        static constexpr uint32_t KnownLocation = 0x02;
        static constexpr uint32_t SignalStrength = 0x04;
    };

private:
    Error scanBF();
    Error scanConical(Vec3f estimatedDirection, float conicalAngle);
    Error scanSegment(float& bestRssi, float& bestAzimuth, float elevationAngle, bool scanAzBackwards = false);
    Error updatePosition();
    Error updatePositionEstimatedLocation(uint64_t currentEpochSeconds);
    Error pointAtCartesian(const Vec3f& pot);

private:
    Radio radio;
    Link link;
    Imu imu;
    Mount mount;
    Gps gps;

    GeoInstant currentEstimatedLocation, prevEstimatedLocation;
    GeoInstant knownLocation;

    GeoLocation projectionOrigin = {-34.0, 19.0}; // origin for Cape Town

    size_t lastPositionUpdate; // in seconds Unix Time

    TrackingConfig trackingConfig;
    uint32_t trackingFlags = TrackingFlags::None;
};

} // namespace gel