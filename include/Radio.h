#include <Arduino.h>

#include "Core.h"
#include "RadioLib.h"

namespace gel
{

enum class ModulationType
{
    LoRa,
    FSK,
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

class Radio
{
public:
    Radio() {};
    Error begin(RadioPins pins, RadioConfig config);
    
    Error send(span<uint8_t> msg);
    Error send(const char* msg);
    Error send(String msg);
    
    Error receive();

private:
    bool initialized = false;

    RadioPins pins;
    uint32_t frequency;
    
    SX1278 radio{nullptr};
    ModulationType modulation = ModulationType::LoRa;
};

} // namespace gel