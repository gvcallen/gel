#pragma once

#include <Arduino.h>
#include "Core.h"

#define MOTOR_MAX_STATES 12

namespace gel
{

struct StepperMotorPins
{
    uint8_t i01;
    uint8_t i11;
    uint8_t ph1;
    uint8_t i02;
    uint8_t i12;
    uint8_t ph2;
    optional<uint8_t> zeroSensor;
    optional<uint8_t> currentSensorA;
    optional<uint8_t> currentSensorB;
};

struct StepperMotorConfig
{
    uint8_t numCurrentLevels = 4;
    bool reverseDirection = true;
    bool activeLow = true;
    uint32_t stepDelay = 20000; // in microseconds
};

class StepperMotor
{
public:
    enum State
    {
        Holding,
        FullStepping,
        HalfStepping,
    };
public:
    StepperMotor() { initialized = false; };
    
    gel::Error begin(StepperMotorPins pins, StepperMotorConfig config);

    void stepForward(double numSteps);
    void stepBackward(double numSteps);
    double cycleForward();
    double cycleBackward();
    
    void saveZeroPosition() { currentStep = 0.0; };

    double getPosition() { return currentStep; }
    void setSpeed(float speedMultiplier);
    Error setCurrentMultiplier(float currentLimit);

    void setState(State state, float currentMultiplier = 1.0);

private:
    void stepDivisional(bool backwards = false);
    void stepN(double numSteps, bool backwards = false);
    void updateIO();
    void setStateArrays(const bool directionStatesA[], const bool directionStatesB[], const float currentStatesA[], const float currentStatesB[], uint8_t numStates);

private:
    bool initialized;
    StepperMotorPins pins;
    StepperMotorConfig config;
    
    uint8_t numStates;
    uint8_t stateIdx;    
    bool directionStatesA[MOTOR_MAX_STATES];
    bool directionStatesB[MOTOR_MAX_STATES];
    float currentStatesA[MOTOR_MAX_STATES];
    float currentStatesB[MOTOR_MAX_STATES];
    State state;
    
    float currentMultiplier = 0.0;
    float speedMultiplier = 1.0;
    uint32_t prevStepTime; // in microseconds
    double currentStep = 0.0;
};


} // namespace gel