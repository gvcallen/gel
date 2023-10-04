#include "gel/Core.h"
#include "gel/StepperMotor.h"

namespace gel
{

struct MountConfig
{
    double azimuthalRevolutionNumSteps = 200.0; 
    double elevationRevolutionNumSteps = 200.0; 
    float gearCorrectionRatio = 1.0; // To correct elevation due to turning azimuth, in el:az
    gel::Bounds1d elevationAngleBounds = {0.0, 2.0*PI}; // Elevation angle when el is at zero steps, and which el cannot go past
};

class Mount
{
    enum CalibrationMethod
    {
        ElevationControlled,
    };

public:
    Mount() { initialized = false; }

    Error begin(StepperMotorPins elevationPins, StepperMotorPins azimuthalPins, MountConfig config);
    void returnToZero();
    Error calibrate(CalibrationMethod method = ElevationControlled);

    double getElevationAngleDegreees() {return getElevationAngle() / PI_OVER_180; };
    double getAzimuthalAngleDegreees() {return getAzimuthalAngle() / PI_OVER_180; };
    double getElevationAngle();
    double getAzimuthalAngle();
    Error setElevationAngleDegrees(double angleInDegrees) {return setElevationAngle(angleInDegrees * PI_OVER_180); };
    void setAzimuthalAngleDegrees(double angleInDegrees) {return setAzimuthalAngle(angleInDegrees * PI_OVER_180); };
    Error setElevationAngle(double angleInRadians);
    void setAzimuthalAngle(double angle);

private:
    void calibrate_elevationControlled();

private:
    static constexpr double HOLDING_CURRENT = 2.0/3.0, MOVING_CURRENT = 2.0/3.0; 

private:
    bool initialized = false, calibrated = false;
    MountConfig config;

    StepperMotor elevationMotor, azimuthalMotor;
};

} // namespace gel