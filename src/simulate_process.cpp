
#include <ros/ros.h>
#include <iostream>
#include <algorithm>
#include "agent.h"
#include "map.h"
#include "utility.h"

void InitializeObjects(std::vector<std::shared_ptr<Agent>> &agents, std::shared_ptr<Map> &map)
{
    // set up the prameter of the start agent
    // here shows an example of 3 agents
    int nAgents = 3;
    std::vector<Pose> startList{Pose(2, 3), Pose(1, 1), Pose(4, 5)};
    std::vector<Pose> goaltList{Pose(12, 16), Pose(19, 19), Pose(18, 18)};
    for (size_t i = 0; i < nAgents; i++)
    {
        agents.push_back(std::make_shared<Agent>(startList.at(i),goaltList.at(i), i));
    }
    for (auto& agent : agents)
    {
        agent->setAgentOnMap(map);
        map->setOccupied(agent->getCurrentPose());
    }
};

/*----------- main function -----------*/
int main(int argc, char **argv)
{
    // ROS initialize node
    ros::init(argc, argv, "parallel multi agent  planner");

    // Initialize the objects
    std::shared_ptr<Map> map = std::make_shared<Map>(20, 20);
    std::vector<std::shared_ptr<Agent>> agents;
    std::vector<std::future<void>> futures;
    InitializeObjects(agents,map);

    // Lanunch the thread
    for(size_t i = 0; i < agents.size(); i++)
    {
        futures.emplace_back(std::async(std::launch::async, &Agent::planning, agents[i]) );
    }
    
    std::for_each(futures.begin(), futures.end(), [](std::future<void> &ftr) {
        ftr.wait();
    });

    // visulization in ROS
    ros::NodeHandle nh_;
    ros::Publisher pub_1 = nh_.advertise<nav_msgs::Path>("path_0" , 1);
    ros::Publisher pub_2 = nh_.advertise<nav_msgs::Path>("path_1" , 1);
    ros::Publisher pub_3 = nh_.advertise<nav_msgs::Path>("path_2" , 1);

    nav_msgs::Path final_path_1 = Util::convertVectorToPathMessage(agents[0]->getPath());
    nav_msgs::Path final_path_2 = Util::convertVectorToPathMessage(agents[1]->getPath());
    nav_msgs::Path final_path_3 = Util::convertVectorToPathMessage(agents[2]->getPath());

    ros::Rate rate(10.0);
    while(nh_.ok()){
        pub_1.publish(final_path_1);
        pub_2.publish(final_path_2);
        pub_3.publish(final_path_3);
        rate.sleep();
    }

    return 0;
}
