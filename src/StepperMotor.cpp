#include "gel/Core.h"
#include "gel/StepperMotor.h"

namespace gel
{

// NB: The maximum number of states must be reflected in MOTOR_MAX_STATES.

// Full step sequence
const float fullStepCurrentStatesA[] =   {1.0, 1.0, 1.0, 1.0}; // output to I0 and I1 current (phase A)
const float fullStepCurrentStatesB[] =   {1.0, 1.0, 1.0, 1.0}; // output to I0 and I1 current (phase B)
const bool fullStepDirectionStatesA[] =  {1,   0,   0,   1  }; // output to direction (phase A)
const bool fullStepDirectionStatesB[] =  {1,   1,   0,   0  }; // output to direction (phase B)

// Half-stepping states
const float halfStepCurrentStatesA[] =   {3.0/3.0, 3.0/3.0, 3.0/3.0, 1.0/3.0, 3.0/3.0, 3.0/3.0, 3.0/3.0, 1.0/3.0}; // output to I0 and I1 current (phase A)
const float halfStepCurrentStatesB[] =   {3.0/3.0, 1.0/3.0, 3.0/3.0, 3.0/3.0, 3.0/3.0, 1.0/3.0, 3.0/3.0, 3.0/3.0}; // output to I0 and I1 current (phase B)
const bool halfStepDirectionStatesA[] =  {0,       0,       0,       1,       1,       1,       1,       0      }; // output to direction (phase A)
const bool halfStepDirectionStatesB[] =  {1,       0,       0,       0,       0,       1,       1,       1      }; // output to direction (phase B)

// Holding states
const float holdingCurrentStatesA[] =    {1.0};
const float holdingCurrentStatesB[] =    {1.0};
const bool holdingDirectionStatesA[] =   {0}; // output to direction (phase A)
const bool holdingDirectionStatesB[] =   {1}; // output to direction (phase B)

gel::Error StepperMotor::begin(StepperMotorPins pins, StepperMotorConfig config)
{
    // Setup config and pins
    this->initialized = true;
    this->pins = pins;
    this->config = config;
    this->stateIdx = 0;

    // Set the various pins to output
    pinMode (this->pins.i01, OUTPUT);
    pinMode (this->pins.i11, OUTPUT);
    pinMode (this->pins.ph1, OUTPUT);
    pinMode (this->pins.i02, OUTPUT);
    pinMode (this->pins.i12, OUTPUT);
    pinMode (this->pins.ph2, OUTPUT);

    // Enable a pseudo-holding state (at 0.0 current)
    setState(Holding, 0.0);
    
    return Error::None;
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
        stateIdx++;
        if (stateIdx == numStates)
            stateIdx = 0;
    }
    else
    {
        if (stateIdx == 0)
            stateIdx = numStates - 1;
        else
            stateIdx--;

    }
    
    updateIO();
}

void StepperMotor::stepN(double numSteps, bool backwards)
{       
    uint32_t numDivisionalSteps = 0;
    double numFullSteps = 0.0;
    float divisionSpeedMultiplier = 1.0;

    if (state == FullStepping)
    {
        round(numSteps);
        numDivisionalSteps = (uint32_t)numSteps;
        divisionSpeedMultiplier = speedMultiplier;
        numFullSteps = numDivisionalSteps;
    }
    else if (state == HalfStepping)
    {
        numSteps *= 2.0;
        round(numSteps);
        numDivisionalSteps = numSteps;
        numFullSteps = numDivisionalSteps / 2.0;
        DEBUG_VARIABLE(numFullSteps);
        divisionSpeedMultiplier = speedMultiplier * 2.0;
    }

    if (!backwards)
        this->currentStep += numFullSteps;
    else
        this->currentStep -= numFullSteps;    

    for (uint32_t i = 0; i < numDivisionalSteps; i++)
    {
        stepDivisional(backwards);
        delayMicroseconds(this->config.stepDelay / divisionSpeedMultiplier);
    }
}

void StepperMotor::stepForward(double numSteps)
{
    if (numSteps < 0.0)
        stepN(-numSteps, true);
    else
        stepN(numSteps, false);
}

void StepperMotor::stepBackward(double numSteps)
{
    if (numSteps < 0.0)
        stepN(-numSteps, false);
    else
        stepN(numSteps, true);
}

double StepperMotor::cycleForward()
{
    stepN(4.0, false);
    return 4.0;
}

double StepperMotor::cycleBackward()
{
    stepN(4.0, true);
    return 4.0;
}

void StepperMotor::updateIO()
{
    uint8_t maxCurrentLevel = config.numCurrentLevels - 1;
    
    uint8_t currentA = currentStatesA[stateIdx] * currentMultiplier * maxCurrentLevel;
    uint8_t currentB = currentStatesB[stateIdx] * currentMultiplier * maxCurrentLevel;
    uint8_t directionA = directionStatesA[stateIdx];
    uint8_t directionB = directionStatesB[stateIdx];

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

void StepperMotor::setState(State state, float currentMultiplier)
{
    switch (state)
    {
    case Holding:
        setStateArrays(holdingDirectionStatesA,
                       holdingDirectionStatesB,
                       holdingCurrentStatesA,
                       holdingCurrentStatesB,
                       NUM_ELEMS(holdingDirectionStatesA)
                       );
        break;
    
    case HalfStepping:
        setStateArrays(halfStepDirectionStatesA,
                       halfStepDirectionStatesB,
                       halfStepCurrentStatesA,
                       halfStepCurrentStatesB,
                       NUM_ELEMS(halfStepDirectionStatesA)
                       );
        break;
    
    case FullStepping:
        setStateArrays(fullStepDirectionStatesA,
                       fullStepDirectionStatesB,
                       fullStepCurrentStatesA,
                       fullStepCurrentStatesB,
                       NUM_ELEMS(fullStepDirectionStatesA)
                       );
        break;

    default:
        break;
    }

    this->state = state;
    stateIdx = 0;
    setCurrentMultiplier(currentMultiplier);
    updateIO();
}

void StepperMotor::setStateArrays(const bool directionStatesA[], const bool directionStatesB[], const float currentStatesA[], const float currentStatesB[], uint8_t numStates)
{
    this->state = state;
    
    this->numStates = numStates;
    memcpy (this->directionStatesA, directionStatesA, sizeof (directionStatesA[0]) * numStates);
    memcpy (this->directionStatesB, directionStatesB, sizeof (directionStatesB[0]) * numStates);
    memcpy (this->currentStatesA, currentStatesA, sizeof (currentStatesA[0]) * numStates);
    memcpy (this->currentStatesB, currentStatesB, sizeof (currentStatesB[0]) * numStates);
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