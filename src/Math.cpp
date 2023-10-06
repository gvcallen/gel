#include "gel/Math.h"

namespace gel
{

double normalizeAngle2PI(double x)
{
    x = fmod(x, GEL_PI_TIMES_2);
    if (x < 0.0)
        x += GEL_PI_TIMES_2;
    return x;
}

} // namespace gel