#pragma once

#include <Arduino.h>
#include <etl/expected.h>
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
        BadCommunication,
        InvalidState,
        OutOfRange,
        Internal,
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

template<typename TValue, typename TError>
using expected = etl::expected<TValue, TError>;

template<typename TError>
using unexpected = etl::unexpected<TError>;

} // namespace gel