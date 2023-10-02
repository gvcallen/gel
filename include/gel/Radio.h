#include <Arduino.h>

#include <RadioLib.h>

#include "gel/Core.h"

// NB: Callback functions must be added approriately if this is changed
#define RADIO_MAX_INSTANCES 5

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
    float bandwidth = 125.0e6;
};

union ModulationConfig
{
    LoRaConfig lora{};
};

struct RadioConfig
{
    float frequency = 434.0e6;
    uint32_t syncWord = 0x12;
    uint8_t outputPower = 10;
    ModulationType modType = ModulationType::LoRa;
    ModulationConfig modConfig{};
    bool master = false;
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
    };

public:
    Radio() {};
    Error begin(RadioPins pins, RadioConfig config);
    
    Error transmit(span<uint8_t> msg);
    Error transmit(const char* msg);
    Error transmit(String msg);
    Error receive();

    Error startTransmit(span<uint8_t> msg);
    Error startTransmit(const char* msg);
    Error startTransmit(String msg);
    Error startReceive();

    Error sleep();

    expected<String, Error> readData();
    
    size_t available();

    float getRssi() { return radio.getRSSI(); }
    State getState() { return currentState; }
    State getPrevState() { return prevState; }

private:
    static void callback0(void) { Radio::get(0)->callback(); }
    static void callback1(void) { Radio::get(1)->callback(); }
    static void callback2(void) { Radio::get(2)->callback(); }
    static void callback3(void) { Radio::get(3)->callback(); }
    static void callback4(void) { Radio::get(4)->callback(); }
    void callback();

private:
    static Radio* instances[RADIO_MAX_INSTANCES];
    static uint8_t numInstances;
    static const uint32_t TIMEOUT_IN_MS = 2000;

    static Radio* get(uint8_t idx);

    void setState(State newState) { prevState = currentState; currentState = newState;}

private:
    bool initialized = false;

    RadioPins pins;
    RadioConfig config;
    uint32_t frequency;
    
    ModulationType modulation = ModulationType::LoRa;

    SX1278 radio {nullptr};
    uint8_t instanceIdx;
    bool operationDone = false;
    volatile State currentState = Idle;
    State prevState = Idle;
};

} // namespace gel