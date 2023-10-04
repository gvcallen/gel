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
    
    azimuthalConfig.reverseDirection = false;
    elevationConfig.reverseDirection = true;

    if (Error err = azimuthalMotor.begin(azimuthalPins, azimuthalConfig))
        return err;
    if (Error err = elevationMotor.begin(elevationPins, elevationConfig))
        return err;
    
    this->calibrated = true;
    return Error::None;
}

Error Mount::calibrate(CalibrationMethod method)
{
    switch (method)
    {
    case ElevationControlled:
        calibrateByControlledElevation();
        break;
    
    default:
        return Error::BadParameter;
    }

    return Error::None;
}

void Mount::calibrateByControlledElevation()
{
    // For this calibration, it is assumed that the ground station is stowed such that the elevation axis is "near zero".
    // Then, the azimuthal axis is spun forwards such the elevation axis locks fully down and that zero-sensor is at zero.

    elevationMotor.setState(StepperMotor::State::HalfStepping, 0.0);
    azimuthalMotor.setState(StepperMotor::State::HalfStepping, MOVING_CURRENT);

    delay(100);
    
    // TODO: Should step forwards until zero sensor reading is maximized
    azimuthalMotor.stepForward(0.1 * config.azimuthalRevolutionNumSteps);
    elevationMotor.setState(StepperMotor::State::HalfStepping, MOVING_CURRENT);
    
    azimuthalMotor.saveZeroPosition();
    elevationMotor.saveZeroPosition();
    
    delay(500);
    elevationMotor.cycleForward();
    
    azimuthalMotor.setCurrentMultiplier(HOLDING_CURRENT);
    elevationMotor.setCurrentMultiplier(HOLDING_CURRENT);
}

// double Mount::

double Mount::convertElevationAngleToPosition(double angle)
{
    angle = normalizeAngle2PI(angle);
    return ((angle - config.elevationAngleBounds.min) / PI_TIMES_2) * config.elevationRevolutionNumSteps;
}

double Mount::convertElevationPositionToAngle(double position)
{
    return normalizeAngle2PI(((position / config.elevationRevolutionNumSteps) * PI_TIMES_2) + config.elevationAngleBounds.min);
}

double Mount::convertAzimuthalAngleToPosition(double angle)
{
    angle = normalizeAngle2PI(angle);
    double azPosition = azimuthalMotor.getPosition();
    double revolutionOffset = azPosition - fmod(azPosition, config.azimuthalRevolutionNumSteps);
    DEBUG_VARIABLE(revolutionOffset);
    return revolutionOffset + (angle / PI_TIMES_2) * config.azimuthalRevolutionNumSteps;
}

double Mount::convertAzimuthalPositionToAngle(double position)
{
    return normalizeAngle2PI((position / config.azimuthalRevolutionNumSteps) * PI_TIMES_2);
}

gel::Bounds1d Mount::getElevationPositionBounds()
{
    gel::Bounds1d bounds;

    bounds.min = convertElevationAngleToPosition(config.elevationAngleBounds.min);
    bounds.max = convertElevationAngleToPosition(config.elevationAngleBounds.max);

    return bounds;
}

double Mount::getElevationAngle()
{
    // Elevation angle is a function of azimuthal position and elevation position.
    // When the azimuthal motor rotates counter-clockwise, the elevation angle "increases" by
    // azelRatio * delta_azimuthalPosition (where the azel ratio is the equivalent change
    // in elevation motor position required to change the elevation angle by the same amount).
    
    double azPosition = azimuthalMotor.getPosition();
    double elPosition = elevationMotor.getPosition();

    double equivalentElPosition = azPosition * config.azelRatio + elPosition;
    return convertElevationPositionToAngle(equivalentElPosition);
}

double Mount::getAzimuthalAngle()
{
    // Azimuthal angle is purely a function of the azimuthal motor's position
    return convertAzimuthalPositionToAngle(azimuthalMotor.getPosition());
}

Error Mount::setElevationAngle(double angle)
{
    if (angle < config.elevationAngleBounds.min || angle > config.elevationAngleBounds.max)
        return Error::OutOfRange;

    double deltaElevationSteps = convertElevationAngleToPosition(angle) - elevationMotor.getPosition();
    
    azimuthalMotor.setCurrentMultiplier(MOVING_CURRENT);
    elevationMotor.setCurrentMultiplier(MOVING_CURRENT);
    
    elevationMotor.stepForward(deltaElevationSteps);

    azimuthalMotor.setCurrentMultiplier(HOLDING_CURRENT);
    elevationMotor.setCurrentMultiplier(HOLDING_CURRENT);
    
    return Error::None;
}

// Must already be in desired state
void Mount::stepAzimuthalCompensated(double azSteps, double elSteps, bool elFirst)
{
    // We always step in ratio 1.5 el steps : 1 az steps
    double elStepDec = 5.5, azStepDec = 5.0;

    if (elSteps < 0.0)
        elStepDec *= -1.0;
    
    if (azSteps < 0.0)
        azStepDec *= - 1.0;

    while (fabs(elSteps) > fabs(elStepDec) && fabs(azSteps) > fabs(azStepDec))
    {
        if (elFirst)
        {
            elevationMotor.stepForward(elStepDec);
            azimuthalMotor.stepForward(azStepDec);
        }
        else
        {
            elevationMotor.stepForward(elStepDec);
            azimuthalMotor.stepForward(azStepDec);
        }

        elSteps -= elStepDec;
        azSteps -= azStepDec;
    }

    if (fabs(elSteps) > 0.0)
        elevationMotor.stepForward(elSteps);

    if (fabs(azSteps) > 0.0)
        azimuthalMotor.stepForward(azSteps);
}

void Mount::setAzimuthalAngle(double angle)
{
    // If az rotates counter clockwise, el must compensate by rotating backwards
    // If az rotates clockwise, el must compensate by rotating forwards
    // We must decided whether to rotate az or el first:
    // - If el is closer to zero than end, do el first if it must go forwards, otherwise do az first
    // - If el is closer to end than zero, do el first if it must go backwards, otherwise do az first
    
    double elPosition = elevationMotor.getPosition();
    double azPosition = azimuthalMotor.getPosition();

    gel::Bounds1d elevationPositionBounds = getElevationPositionBounds();    
    
    double newAzPosition = convertAzimuthalAngleToPosition(angle);
    double deltaAzSteps = newAzPosition - azPosition;

    if (fabs(deltaAzSteps) > config.azimuthalRevolutionNumSteps / 2.0)
        deltaAzSteps += config.azimuthalRevolutionNumSteps;

    double deltaElSteps = -deltaAzSteps * config.azelRatio;

    bool elCloserToStart = (elPosition - elevationPositionBounds.min) < elevationPositionBounds.max - elPosition;
    bool elForwards = deltaElSteps > 0.0;
    bool elFirst = (elCloserToStart && elForwards) || (!elCloserToStart && !elForwards);

    azimuthalMotor.setCurrentMultiplier(MOVING_CURRENT);
    elevationMotor.setCurrentMultiplier(MOVING_CURRENT);

    stepAzimuthalCompensated (deltaAzSteps, deltaElSteps, elFirst);

    azimuthalMotor.setCurrentMultiplier(HOLDING_CURRENT);
    elevationMotor.setCurrentMultiplier(HOLDING_CURRENT);
}

} // namespace gel