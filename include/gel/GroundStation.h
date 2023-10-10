#pragma once

#include "gel/Mount.h"
#include "gel/Radio.h"
#include "gel/Link.h"
#include "gel/Imu.h"

namespace gel
{

// Initially set to be the same as the PQGS Protocol, but could theoretically be different
enum class TrackingMode
{
    None = 0x00,
    GpsUploaded = 0x01,
    GpsReceived = 0x02,
    SignalStrength = 0x04,
};

struct AntennaConfig
{
    float estimatedBeamwidth = GEL_RADIANS(60.0f);          // The estimated beamwidth of the antenna, in radians
    uint32_t scanTimeout = 120;                             // The timeout, in seconds, before any scan function should stop and return
    uint32_t numBeamwidthScanSegments = 4;                  // The number of segments per beamwidth to scan
    uint32_t numAzimuthScanSamples = 20;                    // The number of signal strength samples to take per azimuthal scan
};

struct GroundStationPins
{
    MountPins mount;
    RadioPins radio;
    ImuPins imu;
};

struct GroundStationConfig
{
    RadioConfig radio;
    LinkConfig link;
    MountConfig mount;
    AntennaConfig antennaConfig;
};

class GroundStation
{
public:
    enum State
    {
        Scanning,
        Tracking,
        Fixed,
    };

public:
    Error begin(GroundStationConfig config, GroundStationPins pins);
    Error update();
    Error setTrackingMode(TrackingMode mode) { this->mode = mode; return Error::None; };
    

    Radio& getRadio() { return radio; }
    Mount& getMount() { return mount; }

private:
    Error scanBF();
    Error scanConical(Vec3f estimatedDirection, float conicalAngle);
    Error scanSegment(float& bestRssi, float& bestAzimuth, float elevationAngle, bool scanAzBackwards = false);
    
private:
    Radio radio{};
    Link link{};
    Imu imu{};
    Mount mount{};

    State state;
    TrackingMode mode;

    AntennaConfig antennaConfig;
};

} // namespace gel