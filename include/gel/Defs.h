#pragma once

#include <Arduino.h>

#ifdef GEL_NO_STL
#define ETL_NO_STL
#endif

#include <etl/optional.h>
#include <etl/span.h>
#include <etl/expected.h>
#include <etl/vector.h>
#include <etl/array.h>

// #include "gel/Flagset.h"

namespace gel
{

using namespace etl;

#define NUM_ELEMS(a) (sizeof(a)/sizeof 0[a])
#define DEBUG_VARIABLE(x) Serial.print(#x " = "); Serial.println(x);

class RunEvery
{
public:
    RunEvery(unsigned long every, unsigned long startTime = millis())
        : every(every), prevRun(startTime) {}

    operator bool()
    {
        if (millis() - prevRun > every)
        {
            prevRun = millis();
            return true;
        }
        else
        {
            return false;
        }
    }
    
private:
    const unsigned long every;
    unsigned long prevRun;
};


} // namespace gel