#pragma once

#include "gel/Core.h"
#include "gel/StepperMotor.h"

namespace gel
{

struct MountPins
{
    StepperMotorPins elevationPins;
    StepperMotorPins azimuthalPins;
    uint8_t azimuthalZeroSensor;
};

struct MountConfig
{
    float azimuthalRevolutionNumSteps = 200.0; 
    float elevationRevolutionNumSteps = 200.0; 
    float azelRatio = 1.0; // The equivalent amount the el gear has to turn to correct the elevation caused by the az gear. To correct elevation due to turning azimuth, in el:az
    gel::Bounds1f elevationAngleBounds = {0.0, 2.0*PI}; // Elevation angle when el is at zero steps, and which el cannot go past
    bool slidingCoax = false; // Whether or not the mount's coax can "slide" to prevent twisting. If false, the mount will not rotate the azimuthal a delta of +/- ~maxNonSlidingRevolutions~ revolutions.
    float maxNonSlidingRevolutions = 2.5;

    bool reverseAzimuthalDirection = false;
    bool reverseElevationDirection = false;
};

// NB - entire class is in radians
class Mount
{
    enum CalibrationMethod
    {
        ElevationControlled,
    };

public:
    Mount() { initialized = false; }

    Error begin(MountPins pins, MountConfig config);
    Error calibrate(CalibrationMethod method = ElevationControlled);
    
    Error setElevationAngle(float angle);
    Error setAzimuthalAngle(float angle);
    Error setAzimuthElevation(float azimuthal, float elevation);
    Error setBoresight(Vec3f& boresight);
    Error setConical(Vec3f& boresight, float radiusAngle, float scanAngle);

    void setRotationMatrix(Mat3f& mat) {this->rotationMatrix = mat; };
    
    float getElevationAngle();
    float getAzimuthalAngle();
    Vec3f getBoresight();

    Error returnToStart();

    MountConfig& getConfig() {return config; };

private:

    void calibrateByControlledElevation();
    Error stepAzimuthalAndElevation(float azSteps, float elSteps, bool simultaneous = true);
    void stepAzimuthalAndElevationSimulatenous(float azSteps, float elSteps, bool elFirst = false);
    void stepAzimuthalAndElevationSequential(float azSteps, float elSteps, bool elFirst = false);

    gel::Bounds1f getElevationPositionBounds();
    bool isElevationCloserToStart();
    
    float convertAzimuthalAngleToPosition(float angle, bool closest = true, bool backwards = false);
    float convertAzimuthalPositionToAngle(float position);
    float convertElevationDeltaAngleToDeltaPosition(float deltaAngle);
    
    float getNewAzimuthalPositionFromAngle(float angle);

private:
    static constexpr float HOLDING_CURRENT = 2.0/3.0, MOVING_CURRENT = 2.0/3.0; 

private:
    bool initialized = false, calibrated = false;
    MountConfig config;

    StepperMotor elevationMotor, azimuthalMotor;
    uint8_t azimuthalZeroSensorPin;

    Mat3f rotationMatrix;
};

} // namespace gel