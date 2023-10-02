#include <RadioLib.h>

#include "gel/Core.h"
#include "gel/Radio.h"

namespace gel
{

uint8_t Radio::numInstances = 0;
Radio* Radio::instances[RADIO_MAX_INSTANCES] = {};

Error Radio::begin(RadioPins pins, RadioConfig config)
{
    if (numInstances == RADIO_MAX_INSTANCES)
        return Error::CapacityFull;

    this->pins = pins;
    this->config = config;
    
    if (pins.dio1.has_value())
        radio = new Module(pins.nss, pins.dio0, pins.reset, pins.dio1.value());
    else
        radio = new Module(pins.nss, pins.dio0, pins.reset);

    if (pins.SPI.has_value())
        return Error::NotImplemented;

    int err = Error::None;
    switch (config.modType)
    {
        case ModulationType::LoRa:
        {
            LoRaConfig& loraConfig = config.modConfig.lora;

            err = radio.begin(config.frequency * 1.0e-6,
                              loraConfig.bandwidth * 1.0e-3,
                              loraConfig.spreadingFactor,
                              loraConfig.codeRate,
                              config.syncWord,
                              config.outputPower,
                              config.preambleLength);
            break;
        }
            
        case ModulationType::FSK:
        {
            FSKConfig& fskConfig = config.modConfig.fsk;
            err = radio.beginFSK(config.frequency * 1.0e-6,
                                 fskConfig.bitRate * 1e-3,
                                 fskConfig.frequencyDeviation * 1.0e-3,
                                 fskConfig.bandwidth * 1e-3,
                                 config.outputPower,
                                 config.preambleLength);
            
            if      (fskConfig.dataShaping == 0.0) radio.setDataShaping(RADIOLIB_SHAPING_NONE);
            else if (fskConfig.dataShaping == 0.3) radio.setDataShaping(RADIOLIB_SHAPING_0_3);
            else if (fskConfig.dataShaping == 0.5) radio.setDataShaping(RADIOLIB_SHAPING_0_5);
            else if (fskConfig.dataShaping == 1.0) radio.setDataShaping(RADIOLIB_SHAPING_1_0);
            
            break;
        }

        default:
            return Error::BadParameter;
    }

    if (err != RADIOLIB_ERR_NONE)
        return Error::Internal;

    instanceIdx = numInstances;
    instances[instanceIdx] = this;
    numInstances++;

    switch (instanceIdx)
    {
        case 0: radio.setDio0Action(callback0, RISING); break;
        case 1: radio.setDio0Action(callback1, RISING); break;
        case 2: radio.setDio0Action(callback2, RISING); break;
        case 3: radio.setDio0Action(callback3, RISING); break;
        case 4: radio.setDio0Action(callback4, RISING); break;
    }
    
    standby();
    return Error::None;
}

Error Radio::transmit(span<uint8_t> msg)
{
    setState(Transmitting);
    int err = radio.transmit(msg.data(), msg.size());
    setState(Idle);
  
    if (err != RADIOLIB_ERR_NONE)
    {
        if (err == RADIOLIB_ERR_PACKET_TOO_LONG)
            return Error::OutOfRange;
        else if (err == RADIOLIB_ERR_TX_TIMEOUT)
            return Error::Timeout;
        else
            return Error::Internal;
    }
  
    return Error::None;
}

Error Radio::transmit(const char* msg)
{
    setState(Transmitting);
    int err = radio.transmit(msg);
    setState(Idle);
  
    if (err != RADIOLIB_ERR_NONE)
    {
        if (err == RADIOLIB_ERR_PACKET_TOO_LONG)
            return Error::OutOfRange;
        else if (err == RADIOLIB_ERR_TX_TIMEOUT)
            return Error::Timeout;
        else
            return Error::Internal;
    }
  
    return Error::None;
}

Error Radio::transmit(String msg)
{
    return this->transmit(msg.c_str());
}

Error Radio::receive()
{
    return Error::None;
}

Error Radio::startReceive()
{
    int err = radio.startReceive();

    if (err != RADIOLIB_ERR_NONE)
        return Error::Internal;

    setState(Receiving);
    return Error::None;
}

Error Radio::startTransmit(span<uint8_t> msg)
{
    int err = radio.startTransmit(msg.data(), msg.size());

    if (err != RADIOLIB_ERR_NONE)
        return Error::Internal;

    setState(Transmitting);
    return Error::None;
}

Error Radio::startTransmit(const char* msg)
{
    int err = radio.startTransmit(msg);

    if (err != RADIOLIB_ERR_NONE)
        return Error::Internal;

    setState(Transmitting);
    return Error::None;
}

Error Radio::startTransmit(String msg)
{
    int err = radio.startTransmit(msg);

    if (err != RADIOLIB_ERR_NONE)
        return Error::Internal;

    setState(Transmitting);
    return Error::None;
}

Error Radio::sleep()
{
    int err = radio.sleep();

    if (err != RADIOLIB_ERR_NONE)
        return Error::Internal;

    setState(Idle);
    return Error::None;        
}

Error Radio::standby()
{
    int err = radio.standby();

    if (err != RADIOLIB_ERR_NONE)
        return Error::Internal;

    setState(Idle);
    return Error::None;        
}

void Radio::setState(State newState)
{
    operationDone = false;
    prevState = currentState;
    currentState = newState;
}

void Radio::callback()
{
    operationDone = true;
    if (currentState == Transmitting)
        setState(Idle);
};

size_t Radio::available()
{
    if (!(currentState == Receiving) || !(operationDone == true))
        return 0;

    return radio.getPacketLength();
}

expected<String, Error> Radio::readData()
{
    String str;
    int err = radio.readData(str);
    operationDone = false;
    
    if (err != RADIOLIB_ERR_NONE)
        return expected<String, Error>{unexpected<Error>{Error::Internal}};

    return str;
}

Radio* Radio::get(uint8_t idx)
{
    return Radio::instances[idx];
}

} // namespace gel;