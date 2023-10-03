#include <Arduino.h>

#include "gel/Link.h"

namespace gel {

Error Link::begin(Radio &radio, LinkConfig config)
{
    this->initialized = true;
    this->radio = &radio;
    this->config = config;

    return Error::None;
}

Error Link::update()
{
    // Decide if "listening" must be toggled. Could also be implemented with interrupts -
    // we choose to use times with an "update" function instead to give the user the choice.
    uint32_t currentTime = millis();
    if ((currentTime - prevListenTime) > config.listenWindow)
    {
        if (listening)
        {
            setState(Telemetry);
        }
        else
        {
            setState(Telecommand);
            prevListenTime = currentTime;
        }

        listening = !listening;
    }
    
    // Update for the relevant sub-state
    switch (state)
    {
        case Telemetry:
            return update_telemetry();
            
        case Telecommand:
            return update_telecommand();   

        case Idle:
            break;
    }
    
    return Error::None;   
}

Error Link::update_telemetry()
{
    Radio::State radioState = radio->getState();

    switch (radioState)
    {
    case Radio::Transmitting:
        // Still transmitting. Simply return
        break;
    
    case Radio::Idle:
        // Not sending. Start sending telemetry
        telemetryCallback(sendPayload);
        radio->startTransmit(sendPayload);
        break;

    default:
        break;
    }

    return Error::None;
}

Error Link::update_telecommand()
{
    Radio::State radioState = radio->getState();

    // First, we deal with any data we may have received and forward it
    // to the Telecommand callback to be transmitted
    if (radio->available() > 0)
    {
        auto msg = radio->readData();
        const char* msgString = msg.value().c_str();
        strcpy((char*)receivePayload.data(), msgString);
        telecommandCallback(receivePayload, sendPayload);
        return radio->startTransmit(sendPayload);
    }

    if (radioState != Radio::Receiving)
        return radio->startReceive();

    return Error::None;
}

} // namespace gel