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
    return Error::None;   
}


} // namespace gel