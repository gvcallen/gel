#include "gel/Core.h"
#include "gel/Radio.h"

namespace gel
{

uint8_t Radio::numInstances = 0;
Radio* Radio::instances[RADIO_MAX_INSTANCES] = {};

Error Radio::begin(RadioPins pins, RadioConfig config)
{
    int err = 0;

    if (numInstances == RADIO_MAX_INSTANCES)
        return Error::CapacityFull;

    if (config.modConfig.lora.implicitHeader && !config.payloadLength.has_value())
        return Error::BadParameter;

    if (pins.SPI.has_value())
        return Error::NotImplemented;        

    this->pins = pins;
    this->config = config;

    #if defined(GEL_USE_LORALIB)
    
    if (pins.dio1.has_value())
        return gel::Error::UnsupportedParameter;

    radio = new Module(pins.nss, pins.dio0, pins.reset);

    #else

    if (pins.dio1.has_value())
        radio = new Module(pins.nss, pins.dio0, pins.reset, pins.dio1.value());
    else
        radio = new Module(pins.nss, pins.dio0, pins.reset);

    #endif

    switch (config.modType)
    {
        case ModulationType::LoRa:
        {
            LoRaConfig& loraConfig = config.modConfig.lora;
            
            #if defined(GEL_USE_LORALIB)

            err = radio.begin(config.frequency * 1.0e-6,
                              loraConfig.bandwidth * 1.0e-3,
                              loraConfig.spreadingFactor,
                              loraConfig.codeRate + 4,
                              config.syncWord,
                              config.outputPower,
                              240, 
                              config.preambleLength);

            if (loraConfig.implicitHeader)
                radio.implicitHeader(config.payloadLength.value());

            #else

            err = radio.begin(config.frequency * 1.0e-6,
                              loraConfig.bandwidth * 1.0e-3,
                              loraConfig.spreadingFactor,
                              loraConfig.codeRate + 4,
                              config.syncWord,
                              config.outputPower,
                              config.preambleLength);


            if (loraConfig.implicitHeader)
            {
                if (config.payloadLength.has_value())
                    radio.implicitHeader(config.payloadLength.value());
                else
                    return Error::BadParameter;
            }

            #endif
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

            radio.packetMode();
            
            #if defined(GEL_USE_LORALIB)
            
            radio.setDataShaping(fskConfig.dataShaping);

            #else
            
            if      (fskConfig.dataShaping == 0.0) radio.setDataShaping(RADIOLIB_SHAPING_NONE);
            else if (fskConfig.dataShaping == 0.3) radio.setDataShaping(RADIOLIB_SHAPING_0_3);
            else if (fskConfig.dataShaping == 0.5) radio.setDataShaping(RADIOLIB_SHAPING_0_5);
            else if (fskConfig.dataShaping == 1.0) radio.setDataShaping(RADIOLIB_SHAPING_1_0);

            #endif

            break;
        }

        default:
            return Error::BadParameter;
    }

    if (err != 0)
        return Error::Internal;

    instanceIdx = numInstances;
    instances[instanceIdx] = this;
    numInstances++;

    #if defined(GEL_USE_LORALIB)

    switch (instanceIdx)
    {
        case 0: radio.setDio0Action(callback0); break;
        case 1: radio.setDio0Action(callback1); break;
    }

    #else

    switch (instanceIdx)
    {
        case 0: radio.setDio0Action(callback0, RISING); break;
        case 1: radio.setDio0Action(callback1, RISING); break;
    }

    #endif
    
    initialized = true;
    return standby();
}

Error Radio::startReceive()
{
    int err = 0;

    if (currentState == State::Transmitting)
        return Error::InvalidState;
    
    err = radio.startReceive();
    
    if (err != 0)
        return Error::Internal;

    setState(Receiving);
    return Error::None;
}

Error Radio::startTransmit(span<uint8_t> msg)
{
    int err = radio.startTransmit(msg.data(), msg.size());

    if (err != 0)
        return Error::Internal;

    setState(Transmitting);
    return Error::None;
}

Error Radio::startTransmit(String msg)
{
    span<uint8_t> msgSpan((uint8_t*)msg.c_str(), msg.length());
    return startTransmit(msgSpan);
}

float Radio::getRssi(bool ofLastPacket)
{
    #if defined(GEL_USE_LORALIB)

    if (!ofLastPacket)
        return Error::NotImplemented;

    return radio.getRSSI();
    
    #else

    return radio.getRSSI(ofLastPacket);
    
    #endif
}

float Radio::getSNR()
{
    return radio.getSNR();
}

Error Radio::sleep()
{
    int err = 0;
    
    err = radio.sleep();

    if (err != 0)
        return Error::Internal;

    setState(Idle);
    return Error::None;
}

Error Radio::standby()
{
    int err = radio.standby();

    if (err != 0)
        return Error::Internal;

    setState(Idle);
    return Error::None;        
}

void Radio::setState(State newState)
{
    prevState = currentState;
    currentState = newState;
}

void Radio::callback()
{
    if (currentState == Receiving)
        dataReceived = true;
    else
        setState(Idle);
};

size_t Radio::available()
{
    if (!(currentState == Receiving) || !(dataReceived == true))
        return 0;

    return radio.getPacketLength();
}

expected<String, Error> Radio::readData()
{
    int err;
    String str;

    if (!dataReceived)
        return str;

    err = radio.readData(str);

    dataReceived = false;
    if (err != 0)
        return expected<String, Error>{unexpected<Error>{Error::Internal}};

    return str;
}

Error Radio::readData(span<uint8_t> data)
{
    int err;

    if (!dataReceived)
        return Error::NotFound;

    err = radio.readData(data.data(), data.size());

    dataReceived = false;
    if (err != 0)
        return Error::Internal;

    return Error::None;
}

Error Radio::setPreambleLength(size_t length)
{
    int err = 0;
    
    err = radio.setPreambleLength(length);

    if (err != 0)
        return Error::Internal;

    return Error::None;
}

Radio* Radio::get(uint8_t idx)
{
    return Radio::instances[idx];
}

} // namespace gel;