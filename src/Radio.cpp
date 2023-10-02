#include <RadioLib.h>

#include "gel/Core.h"
#include "gel/Radio.h"

namespace gel
{

uint8_t Radio::numModules = 0;
Radio* Radio::radios[RADIO_MAX_MODULES] = {};

Error Radio::begin(RadioPins pins, RadioConfig config)
{
    if (numModules == RADIO_MAX_MODULES)
        return Error::CapacityFull;
    
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

Error Radio::startListening()
{
    radio.setPacketReceivedAction(receivedCallback0);
    int state = radio.startReceive();

    if (state != RADIOLIB_ERR_NONE)
        return Error::Internal;

    return Error::None;
}

size_t Radio::available()
{
    if (!receivedFlag == true)
        return 0;

    receivedFlag = false;
    return radio.getPacketLength();
}

expected<String, Error> Radio::readData()
{
    String str;
    int state = radio.readData(str);
    
    if (state != RADIOLIB_ERR_NONE)
        return expected<String, Error>{unexpected<Error>{Error::Internal}};

    return str;
}

Radio* Radio::get(uint8_t idx)
{
    return Radio::radios[idx];
}

} // namespace gel;