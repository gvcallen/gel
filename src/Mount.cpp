#include "gel/Mount.h"
#include "gel/StepperMotor.h"

namespace gel
{

Error Mount::begin(StepperMotorPins elevationPins, StepperMotorPins azimuthalPins, MountConfig config)
{
    this->config = config;
    this->initialized = true;
    
    StepperMotorConfig azimuthalConfig {};
    StepperMotorConfig elevationConfig {};
    
    azimuthalConfig.reverseDirection = true;
    elevationConfig.reverseDirection = true;

    if (Error err = azimuthalMotor.begin(azimuthalPins, azimuthalConfig))
        return err;
    if (Error err = elevationMotor.begin(elevationPins, elevationConfig))
        return err;
    
    this->calibrated = true;
    return Error::None;
}

void Mount::returnToZero()
{
    // azimuthalMotor.returnToZero();
    // elevationMotor.returnToZero();
}

Error Mount::calibrate(CalibrationMethod method)
{
    switch (method)
    {
    case ElevationControlled:
        calibrate_elevationControlled();
        break;
    
    default:
        return Error::BadParameter;
    }

    return Error::None;
}

void Mount::calibrate_elevationControlled()
{
    // For this calibration, it is assumed that the ground station is stowed such that the elevation axis is "near zero".
    // Then, the azimuthal axis is spun forwards such the elevation axis locks fully down and that zero-sensor is at zero.

    elevationMotor.setState(StepperMotor::State::HalfStepping, 0.0);
    azimuthalMotor.setState(StepperMotor::State::HalfStepping, MOVING_CURRENT);

    delay(100);
    
    // TODO: Should step backwards until zero sensor reading is maximized
    azimuthalMotor.stepBackward(0.1 * config.azimuthalRevolutionNumSteps);
    elevationMotor.setState(StepperMotor::State::HalfStepping, MOVING_CURRENT);
    
    azimuthalMotor.saveZeroPosition();
    elevationMotor.saveZeroPosition();
    
    delay(500);
    elevationMotor.cycleForward();
    
    azimuthalMotor.setCurrentMultiplier(HOLDING_CURRENT);
    elevationMotor.setCurrentMultiplier(HOLDING_CURRENT);
}

double Mount::getElevationAngle()
{
    return config.elevationAngleBounds.min + elevationMotor.getPosition() / config.elevationRevolutionNumSteps;
}

double Mount::getAzimuthalAngle()
{
    return azimuthalMotor.getPosition() / config.azimuthalRevolutionNumSteps;
}

Error Mount::setElevationAngle(double angle)
{
    if (angle < config.elevationAngleBounds.min || angle > config.elevationAngleBounds.max)
        return Error::OutOfRange;
    
    double newPosition = ((angle - config.elevationAngleBounds.min) / PI_TIMES_2) * config.elevationRevolutionNumSteps;
    
    double deltaSteps = newPosition - elevationMotor.getPosition();
    
    // Serial.print("deltaSteps = "); Serial.println(deltaSteps);
    
    azimuthalMotor.setCurrentMultiplier(MOVING_CURRENT);
    elevationMotor.setCurrentMultiplier(MOVING_CURRENT);
    
    // Serial.print("position before = "); Serial.println(elevationMotor.getPosition());
    elevationMotor.stepForward(deltaSteps);
    // Serial.print("position after = "); Serial.println(elevationMotor.getPosition());

    azimuthalMotor.setCurrentMultiplier(HOLDING_CURRENT);
    elevationMotor.setCurrentMultiplier(HOLDING_CURRENT);
    
    return Error::None;
}

void Mount::setAzimuthalAngle(double angle)
{

}

} // namespace gel