#include "turtlebot3_wall_following_node/laser_detection.hpp"

#include <algorithm>
#include <cmath>

using std::cos;
using std::sin;
using std::vector;

namespace turtlebot3
{
Point to_point(const LaserDetection& detection)
{
    Point p;
    p.x = detection.distance * cos(detection.angle);
    p.y = detection.distance * sin(detection.angle);
    return p;
}

vector<Point> to_points(const vector<LaserDetection>& detections)
{
    vector<Point> points;
    std::transform(detections.cbegin(),
        detections.cend(),
        std::back_inserter(points),
        [](const LaserDetection& detection) { return to_point(detection); });
    return points;
}
} // namespace turtlebot3
