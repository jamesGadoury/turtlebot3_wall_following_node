#pragma once

#include <cmath>

using std::cos;
using std::sin;

namespace turtlebot3
{

struct LaserDetection
{
    float distance;
    float angle;

    float x() const
    {
        return distance * cos(angle);
    }
    float y() const
    {
        return distance * sin(angle);
    }
};

} // namespace turtlebot3
