#pragma once

#include <Arduino.h>

#ifdef GEL_USE_LORALIB
#include <LoRa.h>
#else
#include <RadioLib.h>
#endif

#include "gel/Core.h"

// NB: Callback functions must be added approriately if this is changed
#define RADIO_MAX_INSTANCES 2

namespace gel
{

enum class ModulationType
{
    LoRa,
    FSK,
};

struct LoRaConfig
{
    uint8_t spreadingFactor = 9;
    uint8_t codeRate = 7;
    float bandwidth = 125.0e3;
    bool implicitHeader = false;
};

struct FSKConfig
{
    float bitRate = 4800.0;                                 // 1200 to 300 000 bps
    float frequencyDeviation = 5.0e3;                       // 600 to 200 000 Hz
    float bandwidth = 125.0e3;                              // 2.6, 3.1, 3.9, 5.2, 6.3, 7.8, 10.4, 12.5, 15.6, 20.8, 25, 31.3, 41.7, 50, 62.5, 83.3, 100, 125, 166.7, 200 and 250 kHz
    float dataShaping = 0.5;                                // 0.0, 0.3, 0.5 or 1.0
};


union ModulationConfig
{   
    LoRaConfig lora;
    FSKConfig fsk;
};

struct RadioConfig
{
    float frequency = 434.0e6;
    uint32_t syncWord = 0x12;
    uint8_t outputPower = 10;                               // 2 to 17 dBm
    uint8_t preambleLength = 10;                            // 6 to 65535
    optional<size_t> payloadLength = nullopt;               // Must be specified for "fixed length" modes
    ModulationType modType = ModulationType::LoRa;
    ModulationConfig modConfig{};
};

struct RadioPins
{
    uint8_t nss;
    uint8_t reset;
    uint8_t dio0;
    optional<uint8_t> dio1;
    optional<uint8_t> dio2;
    optional<uint8_t> dio3;
    optional<uint8_t> dio4;
    optional<SPIClass*> SPI;
};

class Radio
{
public:
    enum State
    {
        Idle = 0,           // Note that if the device is sleep, it is in this state, however it could also just be neither receiving not transmitting
        Transmitting,
        Receiving,
        Scanning,
    };

public:
    // Construction
    Radio() = default;
    Error begin(RadioPins pins, RadioConfig config);

    // Transmit and receive    
    Error startTransmit(span<uint8_t> msg);
    Error startTransmit(String msg);
    Error startReceive();
    Error startScan();

    // Change to idle modes
    Error sleep();
    Error standby();

    // Read data available in the registers
    expected<String, Error> readData();
    size_t available();

    // Get information about the system
    float getRssi();
    State getState() { return currentState; }
    State getPrevState() { return prevState; }
    // uint32_t getTimeOnAir(size_t payloadLength = 0);

    // Change settings
    Error setPreambleLength(size_t length);

private:
    static void callback0(void) { Radio::get(0)->callback(); }
    static void callback1(void) { Radio::get(1)->callback(); }
    static void callback0_int(int packetLength) { Radio::get(0)->callback(); }
    void callback();

private:
    static Radio* instances[RADIO_MAX_INSTANCES];
    static uint8_t numInstances;
    static const uint32_t TIMEOUT_IN_MS = 2000;

    static Radio* get(uint8_t idx);

    void setState(State newState);

private:
    bool initialized = false;

    RadioPins pins;
    RadioConfig config;
    uint32_t frequency;
    
    ModulationType modulation = ModulationType::LoRa;

    uint8_t instanceIdx;
    bool dataReceived = false;
    volatile State currentState = Idle;
    State prevState = Idle;

    #ifdef GEL_USE_LORALIB
    LoRaClass& radio = LoRa;
    #else
    SX1278 radio {nullptr};
    #endif
};

} // namespace gel