#pragma once

#include <Arduino.h>

#include <gel/Core.h>

namespace gel
{ 

struct Error : Printable
{
    enum [[nodiscard]] Code
    {
        None = 0,
        Timeout,
        NotFound,
        BadParameter,
        UnsupportedParameter,
        BadCommunication,
        InvalidState,
        OutOfRange,
        Internal,
        CapacityFull,
        NotImplemented,
    };

    Error() = default;
    Error(Code other) : code(other) {}

    // Methods
    explicit operator bool() { return code != Error::None; }
    virtual size_t printTo(Print& p) const override;

    // Operators
    bool operator==(Error other) { return this->code == other.code; };
    bool operator!=(Error other) { return !(*this == other); }

    // Members
    Code code = None;
    const char* message = nullptr;
};

} // namespace gel