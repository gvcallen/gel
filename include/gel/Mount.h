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
    double azimuthalRevolutionNumSteps = 200.0; 
    double elevationRevolutionNumSteps = 200.0; 
    float azelRatio = 1.0; // The equivalent amount the el gear has to turn to correct the elevation caused by the az gear. To correct elevation due to turning azimuth, in el:az
    gel::Bounds1d elevationAngleBounds = {0.0, 2.0*PI}; // Elevation angle when el is at zero steps, and which el cannot go past
    bool slidingCoax = false; // Whether or not the mount's coax can "slide" to prevent twisting. If false, the mount will not rotate the azimuthal a delta of +/- ~maxNonSlidingRevolutions~ revolutions.
    uint8_t maxNonSlidingRevolutions = 1;
};

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
    
    double getElevationAngle();
    double getAzimuthalAngle();
    Error setElevationAngle(double angle);
    Error setAzimuthalAngle(double angle);
    
    Error setSphericalPosition(double azimuthal, double elevation);
    Error setConicalPosition(Vec2d& sphericalCentreAngle, double radius, double scanAngle);

    Error returnToStart();

private:

    void calibrateByControlledElevation();
    Error stepAzimuthalAndElevation(double azSteps, double elSteps, bool simultaneous = true);
    void stepAzimuthalAndElevationSimulatenous(double azSteps, double elSteps, bool elFirst = false);
    void stepAzimuthalAndElevationSequential(double azSteps, double elSteps, bool elFirst = false);

    gel::Bounds1d getElevationPositionBounds();
    bool isElevationCloserToStart();
    
    double convertAzimuthalAngleToPosition(double angle, bool closest = true, bool backwards = false);
    double convertAzimuthalPositionToAngle(double position);
    double convertElevationDeltaAngleToDeltaPosition(double deltaAngle);
    
    double getNewAzimuthalPositionFromAngle(double angle);

private:
    static constexpr double HOLDING_CURRENT = 2.0/3.0, MOVING_CURRENT = 2.0/3.0; 

private:
    bool initialized = false, calibrated = false;
    MountConfig config;

    StepperMotor elevationMotor, azimuthalMotor;
    uint8_t azimuthalZeroSensorPin;
};

} // namespace gel