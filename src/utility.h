#ifndef UTILITY_H_
#define UTILITY_H_
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

namespace Util{
/**
* @brief Agent - struct to record the agent postion
*/
struct Pose{
    Pose() : x(0), y(0) {}
    Pose(int x, int y) : x(x), y(y) {}
    int x;
    int y;
};

struct Action{
    Action(int dx, int dy) : dx(dx), dy(dy) {}
    int dx;
    int dy;
};
nav_msgs::Path convertVectorToPathMessage(const std::vector<Pose> &path);
};
using Pose = Util::Pose;
using Action = Util::Action;


#endif