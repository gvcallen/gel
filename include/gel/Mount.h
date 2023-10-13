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
    float azelRatio = 1.0;                                  // The number of turns (per azimuthal turn) that the elevation gear would have to make to tilt the elevation axis the same amount as caused by the azimuthal gear. Relative to when ~reverseElevationDirection~ is false. 
    
    gel::Bounds1f elevationAngleBounds = {0.0, 2.0*PI};     // Elevation angle when el is at zero steps, and which el cannot go past
    float azimuthalAngleOffset = 0.0;                       // The "offset angle" which defines the angle at the zero position of the azimuthal gear.

    bool slidingCoax = false;                               // Whether or not the mount's coax can "slide" to prevent twisting. If false, the mount will not rotate the azimuthal a delta of +/- ~maxNonSlidingRevolutions~ revolutions.
    float maxNonSlidingRevolutions = 2.5;
    
    bool reverseAzimuthalDirection = false;                 // Specifies that "forward" should be the opposite direction for the azimuthal motor.
    bool reverseElevationDirection = false;                 // Specifies that "forward" should be the opposite direction for the elevation motor. Not that "min" and "max" angles are with respect to this newly specified direction.
    bool calibrateElevationNearMax = false;                 // Used for the "elevation controlled" calibration method. If true, it is assumed that the mount is resting at the "max" elevation angle when calibration starts.
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

    float getElevationAngle();
    float getAzimuthalAngle();
    Vec3f getBoresight();

    Error returnToStart();
    Error returnToStow();

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
};

} // namespace gel