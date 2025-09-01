#include <point.hpp>

namespace turtlebot3
{

struct LaserDetection
{
    float distance;
    float angle;
};

Point to_point(const LaserDetection& detection);

} // namespace turtlebot3
