#include "gel/Error.h"

namespace gel
{

size_t Error::printTo(Print& p) const
{
    size_t n = 0;
    n += p.print(F("\tCode: "));
    n += p.println(static_cast<size_t>(code));
    if (message)
    {
        n += p.print(F("\tMessage: "));
        n += p.print(message);
    }

    return n;
}

} // namespace gel