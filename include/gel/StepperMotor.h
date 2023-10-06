#pragma once

#include <Arduino.h>
#include "Core.h"

#define MOTOR_MAX_NUM_VALUES 12

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
    enum Mode
    {
        Holding,
        FullStepping,
        HalfStepping,
    };

public:
    StepperMotor() { initialized = false; };
    
    gel::Error begin(StepperMotorPins pins, StepperMotorConfig config);

    void stepForward(double numSteps, bool blocking = true);
    void stepBackward(double numSteps, bool blocking = true);
    double cycleForward(uint32_t numCycles = 1);
    double cycleBackward(uint32_t numCycles = 1);

    bool tick(); // returns true if no work was done, and false otherwise
    
    void saveZeroPosition() { position = 0.0; };

    double getPosition() { return position; }
    void setSpeed(float speedMultiplier);
    float getSpeed() { return speedMultiplier; };
    Error setCurrentMultiplier(float currentLimit);

    void setMode(Mode mode, float currentMultiplier = 1.0);

private:
    double getDivisionalSpeedMultiplier();
    double getDivisionalStepDelay();
    double getStepIncrement();
    void stepDivisional(bool backwards = false);
    void stepN(double numSteps, bool backwards = false, bool blocking = true);
    void updateIO();
    void setValueArrays(const bool directionValuesA[], const bool directionValuesB[], const float currentValuesA[], const float currentValuesB[], uint8_t numValues);

private:
    bool initialized;
    StepperMotorPins pins;
    StepperMotorConfig config;
    
    Mode mode;
    uint8_t numValues;
    uint8_t valueIdx;    
    bool directionValuesA[MOTOR_MAX_NUM_VALUES];
    bool directionValuesB[MOTOR_MAX_NUM_VALUES];
    float currentValuesA[MOTOR_MAX_NUM_VALUES];
    float currentValuesB[MOTOR_MAX_NUM_VALUES];
    
    float currentMultiplier = 0.0;
    float speedMultiplier = 1.0;
    uint32_t prevStepTime = 0.0; // in microseconds
    double position = 0.0;
    int32_t divisionalStepsLeft = 0;
};


} // namespace gel