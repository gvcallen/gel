#pragma once

#include "gel/Core.h"
#include "gel/Radio.h"

#define LINK_MAX_PAYLOAD 255

namespace gel {

struct LinkConfig
{
    bool controller = false;
    bool heartbeat = true;
};

using f_int_t = int(*)(int);
using InteractiveCallback = Error(*)(span<uint8_t>, span<uint8_t>);
using StreamingCallback = Error(*)(span<uint8_t>);

// A half-duplex point-to-point communication link between a Controller and a Responder, with two modes:
// Streaming:
//      Payloads are sent from the Responder at regular intervals, while the Controller scans for payloads.
//      The Responder sends the time it left until it takes its next "break", where it will listen for a message for a short period.
//      This is the state both parties start in. 
// Interactive:
//      The Controller sends messages to the Responder, and the responder processes and responds approriately.
class Link
{
public:
    enum Status
    {
        Idle,                   // Idle. Not sending or receiving.
        Streaming,              // Connected. Continuous data streaming from responder to controller.
        Interactive,            // Connected. Interactive commands/requests from controller, and replies from responder.
    };

public:
    Link() = default;
    
    Error begin(Radio &radio, LinkConfig config);
    Error update();

    void setInteractiveCallback(InteractiveCallback callback) {this->interactiveCallback = callback; };
    void setStreamingCallback(StreamingCallback callback) {this->streamingCallback = callback; };

    Status getStatus() { return status; }

private:
    bool initialized = false;
    Radio *radio;
    LinkConfig config;
    
    Status status = Idle;
    InteractiveCallback interactiveCallback;
    StreamingCallback streamingCallback;

    uint32_t lastRecieveTime = 0;
    vector<uint8_t, LINK_MAX_PAYLOAD> receivePayload;
    vector<uint8_t, LINK_MAX_PAYLOAD> sendPayload;

};

} // namespace gel