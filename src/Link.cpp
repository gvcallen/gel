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
    {
        telemetryStartTime = millis();
        numBitsInWindow = 0;
    }
    else if (newState == Telecommand)
    {
        telecommandStartTime = millis();
    }
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

    // Serial.print("#1 "); Serial.println(millis());

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
        Serial.println("Listening...");
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
            numBitsInWindow += payloadLength * 8;
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
    if (radio->getState() == Radio::State::Idle)
    {
        Serial.println("startReceive");
        radio->startReceive();
    }

    if (radio->available())
    {
        radio->readData(this->receivePayload);
        if (this->telemetryCallback)
            return this->telemetryCallback(receivePayload);
    }

    return Error::None;
}

Error Link::updateControllerTelecommand()
{
    return Error::None;
}

float Link::getDataRate()
{
    if (config.controller)
    {
        if (lastTelemetryPacketTime > telemetryStartTime)
            return (float)numBitsInWindow / ((lastTelemetryPacketTime - telemetryStartTime) / 1000.0);
        else
            return 0.0;
    }
    else
    {
        return (float)numBitsInWindow / ((lastTelemetryPacketTime - telemetryStartTime) / 1000.0);
    }

    return 0.0;
}

} // namespace gel