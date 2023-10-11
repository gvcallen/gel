#pragma once

#include <Arduino.h>

#include "gel/Defs.h"

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
        Ignored,
        Outdated,
    };

    Error() = default;
    Error(Code c) : code(c) {}
    Error(Code c, const char* msg) : code(c), message(msg) {}

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

template<typename T>
expected<T, Error> make_unexpected(Error::Code c, const char* msg = nullptr)
{
    Error e{c, msg};
    return etl::expected<T, Error>{unexpected<Error>{e}};
}

template<typename T>
expected<T, Error> make_unexpected(Error e)
{
    return expected<T, Error>{unexpected<Error>{e}};
}

} // namespace gel