#include <Arduino.h>

#include <RadioLib.h>

#include "gel/Core.h"

#define RADIO_MAX_MODULES 5

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
    Radio() {};
    Error begin(RadioPins pins, RadioConfig config);
    
    Error send(span<uint8_t> msg);
    Error send(const char* msg);
    Error send(String msg);
    
    Error receive();

    expected<String, Error> readData();
    
    Error startListening();
    size_t available();

private:
    static void receivedCallback0(void) { Radio::get(0)->receivedCallback(); }
    static void receivedCallback1(void) { Radio::get(1)->receivedCallback(); }
    static void receivedCallback2(void) { Radio::get(2)->receivedCallback(); }
    static void receivedCallback3(void) { Radio::get(3)->receivedCallback(); }
    static void receivedCallback4(void) { Radio::get(4)->receivedCallback(); }
    void receivedCallback() { receivedFlag = true; };

private:
    static Radio* radios[RADIO_MAX_MODULES];
    static uint8_t numModules;
    static const uint32_t TIMEOUT_IN_MS = 2000;

    static Radio* get(uint8_t idx);

private:
    bool initialized = false;

    RadioPins pins;
    uint32_t frequency;
    
    SX1278 radio{nullptr};
    ModulationType modulation = ModulationType::LoRa;

    bool receivedFlag = false;
    uint8_t moduleIdx;
};

} // namespace gel