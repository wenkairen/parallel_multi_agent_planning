#include "utility.h"

namespace Util{
nav_msgs::Path convertVectorToPathMessage(const std::vector<Pose> &path)
{   
    geometry_msgs::Pose pose;
    nav_msgs::Path final_path;
    final_path.poses = std::vector<geometry_msgs::PoseStamped>(path.size());
    std_msgs::Header header_msg;
    header_msg.stamp = ros::Time::now();
    header_msg.frame_id = "map";
    final_path.header = header_msg;

    for(int i = 0; i < path.size(); i++){
        pose.position.x = path[i].x;
        pose.position.y = path[i].y;
        final_path.poses[i].pose = pose;
    }

    return final_path;
};
};