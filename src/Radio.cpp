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

    #if defined(GEL_USE_LORASANDEEP)

    if (pins.dio1.has_value())
        return gel::Error::UnsupportedParameter;
    radio.setPins(pins.nss, pins.reset, pins.dio0);

    #elif defined(GEL_USE_LORALIB)
    
    if (pins.dio1.has_value())
        return gel::Error::UnsupportedParameter;

    radio = new LoRa(pins.nss, pins.dio0, pins.reset);

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
            
            #if defined(GEL_USE_LORASANDEEP)

            if (!radio.begin(config.frequency))
                return Error::Internal;
            
            radio.setFrequency(config.frequency);
            radio.setSignalBandwidth(loraConfig.bandwidth);
            radio.setCodingRate4(loraConfig.codeRate);
            radio.setSyncWord(config.syncWord);
            radio.setTxPower(config.outputPower);
            radio.setPreambleLength(config.preambleLength);
            
            #elif defined(GEL_USE_LORALIB)

            err = radio.begin(config.frequency * 1.0e-6,
                              loraConfig.bandwidth * 1.0e-3,
                              loraConfig.spreadingFactor,
                              loraConfig.codeRate + 4,
                              config.syncWord,
                              config.outputPower,
                              120, 
                              config.preambleLength);

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
                // radio.setMode()
                
                // if (config.payloadLength.has_value())
                    // radio.implicitHeader(config.payloadLength.value());
                // else
                    // return Error::BadParameter;
            }

            #endif
            break;
        }
            
        case ModulationType::FSK:
        {
            #if defined(GEL_USE_LORASANDEEP)

            return Error::UnsupportedParameter;


            #else

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

    #if defined(GEL_USE_LORASANDEEP)

    if (numInstances > 1)
        return Error::CapacityFull;

    radio.onReceive(callback0_int);
    radio.onTxDone(callback0);

    #elif defined(GEL_USE_LORALIB)

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
    
    #ifdef GEL_USE_LORASANDEEP
    if (config.modConfig.lora.implicitHeader)
        radio.receive(config.payloadLength.value());
    else
        radio.receive();
    #else
    err = radio.startReceive();
    #endif
    
    if (err != 0)
        return Error::Internal;

    setState(Receiving);
    return Error::None;
}

Error Radio::startTransmit(span<uint8_t> msg)
{
    int err = 0;

    #ifdef GEL_USE_LORASANDEEP
    radio.beginPacket(config.modConfig.lora.implicitHeader);
    radio.write(msg.data(), msg.size());
    err = radio.endPacket(true);
    #else
    err = radio.startTransmit(msg.data(), msg.size());
    #endif

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

Error Radio::startScan()
{
    #if defined(GEL_USE_LORASANDEEP)
    
    return Error::NotImplemented;
    
    #elif defined(GEL_USE_LORALIB)
    
    return Error::NotImplemented;

    #else

    int err = radio.startChannelScan();
    if (err)
        return gel::Error::Internal;
    
    setState(Scanning);
    return Error::None;
    #endif
}

float Radio::getRssi(bool ofLastPacket)
{
    #if defined(GEL_USE_LORASANDEEP)
    return (float)radio.packetRssi();
    
    #elif defined(GEL_USE_LORALIB)

    if (!ofLastPacket)
        return Error::NotImplemented;

    return radio.getRSSI();
    #else

    return radio.getRSSI(ofLastPacket);
    
    #endif
}

float Radio::getSNR()
{
    #ifdef GEL_USE_LORASANDEEP
    return Error::NotImplemented;
    #else
    return radio.getSNR();
    #endif
}

Error Radio::sleep()
{
    int err = 0;
    
    #ifdef GEL_USE_LORASANDEEP
    radio.sleep();
    #else
    err = radio.sleep();
    #endif

    if (err != 0)
        return Error::Internal;

    setState(Idle);
    return Error::None;
}

Error Radio::standby()
{
    int err = 0;
    
    #ifdef GEL_USE_LORASANDEEP
    radio.idle();
    #else
    err = radio.standby();
    #endif

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
    
    if (currentState == Scanning)
        standby();
};

size_t Radio::available()
{
    if (!(currentState == Receiving) || !(dataReceived == true))
        return 0;

    #ifdef GEL_USE_LORASANDEEP
    return radio.available();
    #else
    return radio.getPacketLength();
    #endif
}

expected<String, Error> Radio::readData()
{
    int err;
    String str;

    if (!dataReceived)
        return str;

    #ifdef GEL_USE_LORASANDEEP
    str = radio.readString();
    #else
    err = radio.readData(str);
    #endif

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

    #ifdef GEL_USE_LORASANDEEP
    return Errro:NotImplemented;
    #else
    err = radio.readData(data.data(), data.size());
    #endif

    dataReceived = false;
    if (err != 0)
        return Error::Internal;

    return Error::None;
}

Error Radio::setPreambleLength(size_t length)
{
    int err = 0;
    
    #ifdef GEL_USE_LORASANDEEP
    radio.setPreambleLength(length);
    #else
    err = radio.setPreambleLength(length);
    #endif

    if (err != 0)
        return Error::Internal;

    return Error::None;
}

Radio* Radio::get(uint8_t idx)
{
    return Radio::instances[idx];
}

} // namespace gel;