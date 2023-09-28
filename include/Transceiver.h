#include <Arduino.h>

#include "Core.h"

class LoRaClass;

namespace gel
{

struct TransceiverPins
{
    uint8_t miso;
    uint8_t mosi;
    uint8_t sck;
    uint8_t nss;
    uint8_t reset;
    optional<uint8_t> dio0;
    optional<uint8_t> dio1;
    optional<uint8_t> dio2;
    optional<uint8_t> dio3;
    optional<uint8_t> dio4;
};

class Transceiver
{
public:
    Transceiver() {};
    int begin(TransceiverPins pins, uint32_t frequency, uint32_t syncWord = 0xF3);
    
    int send(const Printable& printable);
    int receive();

private:
    bool initialized = false;

    TransceiverPins pins;
    uint32_t frequency;
    
    LoRaClass* lora;
};

} // namespace gel