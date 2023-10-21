#include <Arduino.h>

#include "gel/Link.h"

namespace gel
{

Error Link::begin(Radio &radio, LinkConfig config)
{
    this->initialized = true;
    this->radio = &radio;
    this->config = config;

    this->payloadLength = radio.getConfig().payloadLength.value_or(LINK_MAX_PAYLOAD);
    sendPayload.resize(this->payloadLength);
    receivePayload.resize(this->payloadLength);

    setState(Telemetry);

    return Error::None;
}

void Link::setState(State newState)
{
    if (this->state == newState)
        return;

    this->state = newState;
    if (newState == Telemetry)
        telemetryStartTime = millis();
    else if (newState == Telecommand)
        telecommandStartTime = millis();
}

Error Link::update()
{
    if (config.controller)
        return updateController();
    else
        return updateResponder();
}

Error Link::updateResponder()
{
    // Decide if "listening" must be toggled. Could also be implemented with interrupts -
    // we choose to use times with an "update" function instead to give the user the choice.

    uint32_t currentTime = millis();
    if ((listening && (currentTime - telecommandStartTime > config.listenWindow)))
    {
        setState(Telemetry);
        listening = false;
    }
    else if (!listening && (config.listenWindow > 0) && (currentTime - telemetryStartTime) > config.listenInterval)
    {
        setState(Telecommand);
        listening = true;
    }

    // Update for the relevant sub-state
    switch (state)
    {
    case Telemetry:
        return updateResponderTelemetry();

    case Telecommand:
        return updateResponderTelecommand();
    }

    return Error::None;
}

Error Link::updateResponderTelemetry()
{
    auto radioState = radio->getState();

    switch (radioState)
    {
    // We are busy transmitting the previous telemetry packet
    case Radio::Transmitting:
        break;

    // We have just finished transmitting the previous packet - send the next one
    default:
        if (sleeping)
        {
            if (millis() - lastTelemetryPacketTime < config.packetSleep)
                return Error::None; // still sleeping
            else
                sleeping = false; // finished sleeping
        }
        else
        {
            lastTelemetryPacketTime = millis();
            if (config.packetSleep > 0)
            {
                sleeping = true;
                return Error::None;
            }
        }

        telemetryCallback(sendPayload);
        radio->startTransmit(sendPayload);
        break;
    }

    return Error::None;
}

Error Link::updateResponderTelecommand()
{
    Radio::State radioState = radio->getState();

    // First, we deal with any data we may have received and forward it
    // to the Telecommand callback to be transmitted
    size_t packetLength = radio->available();
    if (packetLength)
    {
        radio->readData(receivePayload);
        telecommandCallback(receivePayload, sendPayload);

        return radio->startTransmit(sendPayload);
    }

    if (radioState != Radio::Transmitting)
        return radio->startReceive();

    return Error::None;
}

Error Link::updateController()
{
    // Update for the relevant sub-state
    switch (state)
    {
    case Telemetry:
        return updateControllerTelemetry();

    case Telecommand:
        return updateControllerTelecommand();

    default:
        return Error::InvalidState;
    }
}

Error Link::updateControllerTelemetry()
{
    if (radio->available())
    {
        Serial.println("Available! Radio state = " + String(radio->getState()));
        if (gel::Error err = radio->readData(this->receivePayload))
            return err;
        
        if (this->telemetryCallback)
            return this->telemetryCallback(receivePayload);
    }

    if (radio->getState() == Radio::State::Idle)
    {
        Serial.println("Starting to receive");
        radio->startReceive();
    }

    return Error::None;
}

Error Link::updateControllerTelecommand()
{
    return Error::None;
}

float Link::getDataRate()
{
    return 0.0;
}

} // namespace gel