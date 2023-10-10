#include <Arduino.h>

#include "gel/Mount.h"
#include "gel/StepperMotor.h"

namespace gel
{

Error Mount::begin(MountPins pins, MountConfig config)
{
    this->config = config;
    this->initialized = true;
    this->azimuthalZeroSensorPin = pins.azimuthalZeroSensor;
    
    StepperMotorConfig azimuthalConfig {};
    StepperMotorConfig elevationConfig {};
    
    azimuthalConfig.reverseDirection = config.reverseAzimuthalDirection;
    elevationConfig.reverseDirection = config.reverseElevationDirection;

    if (Error err = azimuthalMotor.begin(pins.azimuthalPins, azimuthalConfig))
        return err;
    if (Error err = elevationMotor.begin(pins.elevationPins, elevationConfig))
        return err;

    pinMode(azimuthalZeroSensorPin, INPUT);
    
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

    elevationMotor.setMode(StepperMotor::Mode::HalfStepping, 0.0);
    azimuthalMotor.setMode(StepperMotor::Mode::HalfStepping, MOVING_CURRENT);

    delay(100);
    
    // For azimuthal, we step forward until the azimuthal zero sensor is high.
    // Note that if it is already high, we go back 0.1 a revolution, and then start
    if (!analogRead(azimuthalZeroSensorPin))
    {
        azimuthalMotor.setSpeed(3.0);
        while (analogRead(azimuthalZeroSensorPin) < 4095)
            azimuthalMotor.stepForward(0.5);
    }
    azimuthalMotor.setSpeed(0.5);
    while (analogRead(azimuthalZeroSensorPin) > 100)
        azimuthalMotor.stepBackward(0.5);

    while (analogRead(azimuthalZeroSensorPin) < 4095)
        azimuthalMotor.stepForward(0.5);

    azimuthalMotor.setSpeed(1.0);

    elevationMotor.setMode(StepperMotor::Mode::HalfStepping, MOVING_CURRENT);
    elevationMotor.cycleForward(1);
    
    azimuthalMotor.saveZeroPosition();
    elevationMotor.saveZeroPosition();

    delay(100);
    
    azimuthalMotor.setCurrentMultiplier(HOLDING_CURRENT);
    elevationMotor.setCurrentMultiplier(HOLDING_CURRENT);
}

Error Mount::returnToStart()
{
    Error err = setAzimuthElevation(0.0, config.elevationAngleBounds.min);
    if (!err)
    {
        azimuthalMotor.saveZeroPosition();
        elevationMotor.saveZeroPosition();
    }
    return err;
}

float Mount::convertElevationDeltaAngleToDeltaPosition(float deltaAngle)
{
    return (deltaAngle / GEL_PI_TIMES_2) * config.elevationRevolutionNumSteps;
}

float Mount::convertAzimuthalAngleToPosition(float angle, bool closest, bool backwards)
{
    angle = normalizeAngle2PI(angle);
    float azPosition = azimuthalMotor.getPosition();
    float revolutionOffset = azPosition - fmod(azPosition, config.azimuthalRevolutionNumSteps);
    
    float newAzPosition = revolutionOffset + (angle / GEL_PI_TIMES_2) * config.azimuthalRevolutionNumSteps;
    float azHalfRevolution = config.azimuthalRevolutionNumSteps / 2.0;

    if (closest)
    {
        // If the candidate new az position is not the closest
        if (fabs (newAzPosition - azPosition) > azHalfRevolution)
        {
            if (newAzPosition > azPosition)
                newAzPosition -= config.azimuthalRevolutionNumSteps;
            else
                newAzPosition += config.azimuthalRevolutionNumSteps;
        }
    }
    else
    {
        if (!backwards && (newAzPosition < azPosition))
            newAzPosition += config.azimuthalRevolutionNumSteps;
        else if (backwards && (newAzPosition > azPosition))
            newAzPosition -= config.azimuthalRevolutionNumSteps;
    }

    return newAzPosition;
}

float Mount::convertAzimuthalPositionToAngle(float position)
{
    return normalizeAngle2PI((position / config.azimuthalRevolutionNumSteps) * GEL_PI_TIMES_2);
}

gel::Bounds1f Mount::getElevationPositionBounds()
{
    gel::Bounds1f bounds;
    
    float elPosition = elevationMotor.getPosition();
    float elAngle = getElevationAngle();

    bounds.min = elPosition - convertElevationDeltaAngleToDeltaPosition(elAngle - config.elevationAngleBounds.min);
    bounds.max = elPosition + convertElevationDeltaAngleToDeltaPosition(elAngle + config.elevationAngleBounds.max);

    return bounds;
}

bool Mount::isElevationCloserToStart()
{
    float elPosition = elevationMotor.getPosition();
    gel::Bounds1f elevationPositionBounds = getElevationPositionBounds();    
    return (elPosition - elevationPositionBounds.min) < elevationPositionBounds.max - elPosition;
}

float Mount::getElevationAngle()
{
    // Elevation angle is a function of azimuthal position and elevation position.
    // When the azimuthal motor rotates counter-clockwise, the elevation angle "increases" by
    // azelRatio * delta_azimuthalPosition (where the azel ratio is the equivalent change
    // in elevation motor position required to change the elevation angle by the same amount).
    
    float azPosition = azimuthalMotor.getPosition();
    float elPosition = elevationMotor.getPosition();

    float equivalentElPosition = azPosition * config.azelRatio + elPosition;
    return normalizeAngle2PI(((equivalentElPosition / config.elevationRevolutionNumSteps) * GEL_PI_TIMES_2) + config.elevationAngleBounds.min);
}

float Mount::getAzimuthalAngle()
{
    // Azimuthal angle is purely a function of the azimuthal motor's position
    return convertAzimuthalPositionToAngle(azimuthalMotor.getPosition());
}

Vec3f Mount::getBoresight()
{
    return azimuthElevationToCartesian(getAzimuthalAngle(), getElevationAngle(), 1.0);
}

Error Mount::setElevationAngle(float angle)
{
    if (angle < config.elevationAngleBounds.min || angle > config.elevationAngleBounds.max)
        return Error::OutOfRange;

    float deltaAngle = angle - getElevationAngle();
    float deltaElSteps = convertElevationDeltaAngleToDeltaPosition(deltaAngle);
    
    stepAzimuthalAndElevation(0.0, deltaElSteps);
    
    return Error::None;
}

void Mount::stepAzimuthalAndElevationSimulatenous(float azSteps, float elSteps, bool elFirst)
{
    float elSpeedSaved = elevationMotor.getSpeed();
    float azSpeedSaved = azimuthalMotor.getSpeed();

    if (fabs(azSteps) <= fabs(elSteps))
    {
        float speedRatio = fabs(azSteps) / fabs(elSteps);
        azimuthalMotor.setSpeed(azSpeedSaved * speedRatio);
    }
    else
    {
        float speedRatio = fabs(elSteps) / fabs(azSteps);
        elevationMotor.setSpeed(elSpeedSaved * speedRatio);
    }

    if (elFirst)
    {
        elevationMotor.stepForward(elSteps, false);
        azimuthalMotor.stepForward(azSteps, false);
    }
    else
    {
        elevationMotor.stepForward(elSteps, false);
        azimuthalMotor.stepForward(azSteps, false);    
    }

    bool elMoving = true, azMoving = true;
    while (elMoving || azMoving)
    {
        elMoving = elevationMotor.tick();
        azMoving = azimuthalMotor.tick();
    }

    elevationMotor.setSpeed(elSpeedSaved);
    azimuthalMotor.setSpeed(azSpeedSaved);
}

void Mount::stepAzimuthalAndElevationSequential(float azSteps, float elSteps, bool elFirst)
{
    if (elFirst)
    {
        elevationMotor.stepForward(elSteps);
        azimuthalMotor.stepForward(azSteps);
    }
    else
    {
        azimuthalMotor.stepForward(azSteps);
        elevationMotor.stepForward(elSteps);
    }
}

// The elevation axis will be clamped if it would push the elevation gear out of bounds
Error Mount::stepAzimuthalAndElevation(float azSteps, float elSteps, bool simultaneous)
{
    azimuthalMotor.setCurrentMultiplier(MOVING_CURRENT);
    elevationMotor.setCurrentMultiplier(MOVING_CURRENT);

    bool elCloserToStart = isElevationCloserToStart();
    bool elForwards = elSteps > 0.0;
    bool elFirst = (elCloserToStart && elForwards) || (!elCloserToStart && !elForwards);
    
    if (simultaneous)
        stepAzimuthalAndElevationSimulatenous(azSteps, elSteps, elFirst);
    else
        stepAzimuthalAndElevationSequential(azSteps, elSteps, elFirst);

    azimuthalMotor.setCurrentMultiplier(HOLDING_CURRENT);
    elevationMotor.setCurrentMultiplier(HOLDING_CURRENT);

    return Error::None;
}

float Mount::getNewAzimuthalPositionFromAngle(float angle)
{
    // If our coax can slide, we simply move to the closest az position. If not, we ensure az stays within confinements
    float newAzPosition = convertAzimuthalAngleToPosition(angle, true);
    if (!config.slidingCoax)
    {
        if (newAzPosition > (config.maxNonSlidingRevolutions * config.azimuthalRevolutionNumSteps))
            newAzPosition = convertAzimuthalAngleToPosition(angle, false, true);
        else if (newAzPosition < (-config.maxNonSlidingRevolutions * config.azimuthalRevolutionNumSteps))
            newAzPosition = convertAzimuthalAngleToPosition(angle, false, false);
    }

    return newAzPosition;
}

Error Mount::setAzimuthElevation(float azimuthalInRadians, float elevationInRadians)
{
    if (elevationInRadians < config.elevationAngleBounds.min || elevationInRadians > config.elevationAngleBounds.max)
        return Error::OutOfRange;
    
    float deltaElAngle = elevationInRadians - getElevationAngle();
    float deltaAzSteps = getNewAzimuthalPositionFromAngle(azimuthalInRadians) - azimuthalMotor.getPosition();
    float deltaElSteps = -deltaAzSteps * config.azelRatio + convertElevationDeltaAngleToDeltaPosition(deltaElAngle);

    return stepAzimuthalAndElevation(deltaAzSteps, deltaElSteps);
}

Error Mount::setConical(Vec3f& boresight, float radiusAngle, float scanAngle)
{
    Vec3f jVec = {0, 1, 0}, iVecLocal, jVecLocal;

    iVecLocal = cross(jVec, boresight);
    jVecLocal = cross(boresight, iVecLocal);
    float r = tan(radiusAngle);

    Vec3f target = boresight + r*cos(scanAngle)*iVecLocal + r*sin(scanAngle)*jVecLocal;
    return setBoresight(target);
}

Error Mount::setBoresight(Vec3f& boresight)
{
    float az, el;
    cartesianToAzimuthElevation(&az, &el, nullptr, boresight);

    return setAzimuthElevation(az, el);
}

Error Mount::setAzimuthalAngle(float angle)
{
    // If az rotates counter clockwise, el must compensate by rotating backwards
    // If az rotates clockwise, el must compensate by rotating forwards
    // We must decided whether to rotate az or el first:
    // - If el is closer to zero than end, do el first if it must go forwards, otherwise do az first
    // - If el is closer to end than zero, do el first if it must go backwards, otherwise do az first
    float deltaAzSteps = getNewAzimuthalPositionFromAngle(angle) - azimuthalMotor.getPosition();
    float deltaElSteps = -deltaAzSteps * config.azelRatio;
    return stepAzimuthalAndElevation(deltaAzSteps, deltaElSteps);
}

} // namespace gel