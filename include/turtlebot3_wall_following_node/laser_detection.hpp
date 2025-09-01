#include "turtlebot3_wall_following_node/point.hpp"

#include <vector>

namespace turtlebot3
{

struct LaserDetection
{
    float distance;
    float angle;
};

Point to_point(const LaserDetection& detection);
std::vector<Point> to_points(const std::vector<LaserDetection>& detections);

} // namespace turtlebot3
