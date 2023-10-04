#pragma once

#include "gel/Core.h"
#include "gel/Radio.h"

#define LINK_MAX_PAYLOAD 255

namespace gel {

struct LinkConfig
{
    bool controller = false;
    uint32_t listenInterval = 9000;
    uint32_t listenWindow = 1000;
};


using TelecommandCallback = Error(*)(span<uint8_t>, span<uint8_t>);     // Parameters: command, response
using TelemetryCallback = Error(*)(span<uint8_t>);            // Parameters: telemetry

// A half-duplex point-to-point communication link between a Controller and a Responder, with two modes:
// Telemetry:
//      Payloads are sent from the Responder at regular intervals, while the Controller scans for payloads.
//      The Responder sends the time it left until it takes its next "break", where it will listen for a message for a short period.
//      This is the state both parties start in. 
// Telecommand:
//      The Controller sends messages to the Responder, and the responder processes and responds approriately.
class Link
{
public:
    enum State
    {
        Idle,                   // Idle. Not sending or receiving.
        Telemetry,              // Connected. Continuous telemetry from responder to controller.
        Telecommand,            // Connected. Telecommands from controller, and replies from responder.
    };

public:
    Link() = default;
    
    Error begin(Radio &radio, LinkConfig config);
    Error update();

    void setTelecommandCallback(TelecommandCallback callback) { this->telecommandCallback = callback; };
    void setTelemetryCallback(TelemetryCallback callback) { this->telemetryCallback = callback; };

    void setState(State newState);
    State getState() { return state; }

private:
    Error update_telemetry();
    Error update_telecommand();

private:
    bool initialized = false;
    Radio *radio;
    LinkConfig config;
    
    State state = Idle;
    TelecommandCallback telecommandCallback;
    TelemetryCallback telemetryCallback;

    uint32_t prevRecieveTime = 0;
    uint32_t telecommandStartTime = 0;
    uint32_t telemetryStartTime = 0;
    bool listening = false;
    vector<uint8_t, LINK_MAX_PAYLOAD> receivePayload;
    vector<uint8_t, LINK_MAX_PAYLOAD> sendPayload;

};

} // namespace gel