#include "gel/Core.h"
#include "gel/StepperMotor.h"

namespace gel
{

// NB: The maximum number of state must be reflected in MOTOR_MAX_STATES.

// Full step mode
const float fullStepCurrentModesA[] =   {1.0, 1.0, 1.0, 1.0}; // output to I0 and I1 current (phase A)
const float fullStepCurrentModesB[] =   {1.0, 1.0, 1.0, 1.0}; // output to I0 and I1 current (phase B)
const bool fullStepDirectionValuesA[] =  {1,   0,   0,   1  }; // output to direction (phase A)
const bool fullStepDirectionValuesB[] =  {1,   1,   0,   0  }; // output to direction (phase B)

// Half-stepping mode
const float halfStepCurrentModesA[] =   {3.0/3.0, 3.0/3.0, 3.0/3.0, 1.0/3.0, 3.0/3.0, 3.0/3.0, 3.0/3.0, 1.0/3.0}; // output to I0 and I1 current (phase A)
const float halfStepCurrentModesB[] =   {3.0/3.0, 1.0/3.0, 3.0/3.0, 3.0/3.0, 3.0/3.0, 1.0/3.0, 3.0/3.0, 3.0/3.0}; // output to I0 and I1 current (phase B)
const bool halfStepDirectionValuesA[] =  {0,       0,       0,       1,       1,       1,       1,       0      }; // output to direction (phase A)
const bool halfStepDirectionValuesB[] =  {1,       0,       0,       0,       0,       1,       1,       1      }; // output to direction (phase B)

// Holding mode
const float holdingCurrentModesA[] =    {1.0};
const float holdingCurrentModesB[] =    {1.0};
const bool holdingDirectionValuesA[] =   {0}; // output to direction (phase A)
const bool holdingDirectionValuesB[] =   {1}; // output to direction (phase B)

gel::Error StepperMotor::begin(StepperMotorPins pins, StepperMotorConfig config)
{
    // Setup config and pins
    this->initialized = true;
    this->pins = pins;
    this->config = config;
    this->valueIdx = 0;

    // Set the various pins to output
    pinMode (this->pins.i01, OUTPUT);
    pinMode (this->pins.i11, OUTPUT);
    pinMode (this->pins.ph1, OUTPUT);
    pinMode (this->pins.i02, OUTPUT);
    pinMode (this->pins.i12, OUTPUT);
    pinMode (this->pins.ph2, OUTPUT);

    // Enable a pseudo-holding mode (at 0.0 current)
    setMode(Holding, 0.0);
    
    return Error::None;
}

double StepperMotor::getDivisionalSpeedMultiplier()
{
    if (mode == FullStepping)
        return speedMultiplier;
    else if (mode == HalfStepping)
        return speedMultiplier * 2.0;

    return speedMultiplier;
}

double StepperMotor::getDivisionalStepDelay()
{
    return config.stepDelay / getDivisionalSpeedMultiplier();
}

double StepperMotor::getStepIncrement()
{
    if (mode == FullStepping)
        return 1.0;
    else if (mode == HalfStepping)
        return 0.5;
    return 1.0;
}

void StepperMotor::setSpeed(float speedMultiplier)
{
    this->speedMultiplier = speedMultiplier;
}

void StepperMotor::stepDivisional(bool backwards)
{
    this->prevStepTime = micros();

    bool downwards = backwards != config.reverseDirection;

    if (!downwards)
    {
        valueIdx++;
        if (valueIdx == numValues)
            valueIdx = 0;
    }
    else
    {
        if (valueIdx == 0)
            valueIdx = numValues - 1;
        else
            valueIdx--;

    }
    
    updateIO();
}

void StepperMotor::stepN(double numSteps, bool backwards, bool blocking)
{       
    uint32_t numDivisionalSteps = 0;
    double numFullSteps = 0.0;
    float divisionSpeedMultiplier = getDivisionalSpeedMultiplier();

    if (mode == FullStepping)
    {
        round(numSteps);
        numDivisionalSteps = (uint32_t)numSteps;
        numFullSteps = numDivisionalSteps;
    }
    else if (mode == HalfStepping)
    {
        numSteps *= 2.0;
        round(numSteps);
        numDivisionalSteps = numSteps;
        numFullSteps = numDivisionalSteps / 2.0;
    }

    if (backwards)
        this->position -= numFullSteps;    
    else
        this->position += numFullSteps;
    if (blocking)
    {

        for (uint32_t i = 0; i < numDivisionalSteps; i++)
        {
            stepDivisional(backwards);
            delayMicroseconds(this->config.stepDelay / divisionSpeedMultiplier);
        }
    }
    else
    {
        if (backwards)
            this->divisionalStepsLeft -= numDivisionalSteps;
        else
            this->divisionalStepsLeft += numDivisionalSteps;

        tick();
    }
}

void StepperMotor::stepForward(double numSteps, bool blocking)
{
    if (numSteps < 0.0)
        stepN(-numSteps, true, blocking);
    else
        stepN(numSteps, false, blocking);
}

void StepperMotor::stepBackward(double numSteps, bool blocking)
{
    if (numSteps < 0.0)
        stepN(-numSteps, false, blocking);
    else
        stepN(numSteps, true, blocking);
}

double StepperMotor::cycleForward(uint32_t numCycles)
{
    double numSteps = 0.0;
    for (uint32_t i = 0; i < numCycles; i++)
    {
        stepN(4.0, false);
        numSteps += 4.0;
    }
    return numSteps;
}

double StepperMotor::cycleBackward(uint32_t numCycles)
{
    double numSteps = 0.0;
    for (uint32_t i = 0; i < numCycles; i++)
    {
        stepN(4.0, true);
        numSteps += 4.0;
    }
    return numSteps;
}

bool StepperMotor::tick()
{
    if (abs(divisionalStepsLeft) == 0)
        return false;

    bool busyWithDelay = (micros() - prevStepTime) < getDivisionalStepDelay();

    if (busyWithDelay)
        return true;
     
    bool backwards = divisionalStepsLeft < 0;

    if (backwards)
        divisionalStepsLeft += 1;
    else
        divisionalStepsLeft -= 1;

    stepDivisional(backwards);
    return true;
}

void StepperMotor::updateIO()
{
    uint8_t maxCurrentLevel = config.numCurrentLevels - 1;
    
    uint8_t currentA = currentValuesA[valueIdx] * currentMultiplier * maxCurrentLevel;
    uint8_t currentB = currentValuesB[valueIdx] * currentMultiplier * maxCurrentLevel;
    uint8_t directionA = directionValuesA[valueIdx];
    uint8_t directionB = directionValuesB[valueIdx];

    if (config.activeLow)
    {
        currentA = maxCurrentLevel - currentA;
        currentB = maxCurrentLevel - currentB;
    }
    
    digitalWrite(pins.i01, currentA & 0b01);
    digitalWrite(pins.i11, currentA & 0b10);
    digitalWrite(pins.i02, currentB & 0b01);
    digitalWrite(pins.i12, currentB & 0b10);
    digitalWrite(pins.ph1, directionA);
    digitalWrite(pins.ph2, directionB);
}

void StepperMotor::setMode(Mode mode, float currentMultiplier)
{
    switch (mode)
    {
    case Holding:
        setValueArrays(holdingDirectionValuesA,
                       holdingDirectionValuesB,
                       holdingCurrentModesA,
                       holdingCurrentModesB,
                       NUM_ELEMS(holdingDirectionValuesA)
                       );
        break;
    
    case HalfStepping:
        setValueArrays(halfStepDirectionValuesA,
                       halfStepDirectionValuesB,
                       halfStepCurrentModesA,
                       halfStepCurrentModesB,
                       NUM_ELEMS(halfStepDirectionValuesA)
                       );
        break;
    
    case FullStepping:
        setValueArrays(fullStepDirectionValuesA,
                       fullStepDirectionValuesB,
                       fullStepCurrentModesA,
                       fullStepCurrentModesB,
                       NUM_ELEMS(fullStepDirectionValuesA)
                       );
        break;

    default:
        break;
    }

    this->mode = mode;
    valueIdx = 0;
    setCurrentMultiplier(currentMultiplier);
    updateIO();
}

void StepperMotor::setValueArrays(const bool directionValuesA[], const bool directionValuesB[], const float currentValuesA[], const float currentValuesB[], uint8_t numValues)
{
    this->mode = mode;
    
    this->numValues = numValues;
    memcpy (this->directionValuesA, directionValuesA, sizeof (directionValuesA[0]) * numValues);
    memcpy (this->directionValuesB, directionValuesB, sizeof (directionValuesB[0]) * numValues);
    memcpy (this->currentValuesA, currentValuesA, sizeof (currentValuesA[0]) * numValues);
    memcpy (this->currentValuesB, currentValuesB, sizeof (currentValuesB[0]) * numValues);
}

Error StepperMotor::setCurrentMultiplier(float currentMultiplier)
{
    if (currentMultiplier < 0.0 || currentMultiplier > 1.0)
        return Error::BadParameter;
    
    this->currentMultiplier = currentMultiplier;
    updateIO();
    
    return Error::None;
}

} // namespace gel