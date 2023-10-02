#include <RadioLib.h>

#include "Radio.h"
#include "Core.h"


namespace gel
{

const uint32_t TIMEOUT_IN_MS = 2000;
bool receivedFlags[RADIO_MAX_MODULES];
static uint8_t numModules = 0;

static void receivedCallback0(void) { receivedFlags[0] = true; }
static void receivedCallback1(void) { receivedFlags[1] = true; }
static void receivedCallback2(void) { receivedFlags[2] = true; }
static void receivedCallback3(void) { receivedFlags[3] = true; }
static void receivedCallback4(void) { receivedFlags[4] = true; }

Error Radio::begin(RadioPins pins, RadioConfig config)
{
    if (pins.dio1.has_value())
        radio = new Module(pins.nss, pins.dio0, pins.reset, pins.dio1.value());
    else
        radio = new Module(pins.nss, pins.dio0, pins.reset);

    if (pins.SPI.has_value())
        return Error::NotImplemented;

    int state;
    switch (config.modType)
    {
        case ModulationType::LoRa:
        {
            LoRaConfig& loraConfig = config.modConfig.lora;
            state = radio.begin(config.frequency / 1.0e6,
                                loraConfig.bandwidth / 1.0e6,
                                loraConfig.spreadingFactor,
                                loraConfig.codeRate);
            break;
        }
            
        case ModulationType::FSK:
            return Error::NotImplemented;

        default:
            return Error::BadParameter;
    }

    if (state != RADIOLIB_ERR_NONE)
        return Error::Internal;

    moduleIdx = numModules;
    numModules++;
    return Error::None;
}

Error Radio::send(span<uint8_t> msg)
{
    int state = radio.transmit(msg.data(), msg.size());
  
    if (state != RADIOLIB_ERR_NONE)
    {
        if (state == RADIOLIB_ERR_PACKET_TOO_LONG)
            return Error::OutOfRange;
        else if (state == RADIOLIB_ERR_TX_TIMEOUT)
            return Error::Timeout;
        else
            return Error::Internal;
    }
  
    return Error::None;
}

Error Radio::send(const char* msg)
{
    int state = radio.transmit(msg);
  
    if (state != RADIOLIB_ERR_NONE)
    {
        if (state == RADIOLIB_ERR_PACKET_TOO_LONG)
            return Error::OutOfRange;
        else if (state == RADIOLIB_ERR_TX_TIMEOUT)
            return Error::Timeout;
        else
            return Error::Internal;
    }
  
    return Error::None;
}

Error Radio::send(String msg)
{
    this->send(msg.c_str());
    return Error::None;
}

Error Radio::receive()
{
    return Error::None;
}

} // namespace gel;