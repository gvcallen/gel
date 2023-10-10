#include "gel/Math.h"

namespace gel
{

Vec3f cross(Vec3f& v1, Vec3f& v2)
{
    Vec3f result;

    result.x = v1.y * v2.z - v1.z * v2.y;
    result.y = v1.x * v2.z - v1.z * v2.x;
    result.z = v1.x * v2.y - v1.y * v2.x;

    return result;
}

void cartesianToAzimuthElevation(float* az, float* el, float* r, Vec3f& pt)
{
    float XsqPlusYsq = pt.x*pt.x + pt.y*pt.y;
    if (r)
        *r = sqrt(XsqPlusYsq + pt.z*pt.z);
    if (el)
        *el = atan2(pt.z, sqrt(XsqPlusYsq));
    if (az)
        *az = atan2(pt.y, pt.x);
}

Vec3f azimuthElevationToCartesian(float az, float el, float r)
{
    Vec3f result;

    result.x = r * cos(az) * cos(el);
    result.y = r * sin(az) * cos(el);
    result.z = r * sin(el);

    return result;
}

float normalizeAngle2PI(float x)
{
    x = fmod(x, GEL_PI_TIMES_2);
    if (x < 0.0)
        x += GEL_PI_TIMES_2;
    return x;
}

} // namespace gel