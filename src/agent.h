#ifndef AGENT_H
#define AGENT_H

#include <iostream>
#include <thread>
#include <vector>
#include <future>
#include <mutex>
#include <cmath>
#include "utility.h"

// forward declas class
class Map;

class Agent
{
public:
    /**
     * @brief Agent
     * @param start - 2D start pose
     * @param goal - 2D end pose
     * @param id - agent id number
     */
    Agent(Pose start, Pose goal, int id) : curPose_(start), goalPose_(goal), id_(id) { path_.push_back(start);}

    /**
     * @brief planning - starting the planning process to reach the goal
     */
    void planning();

    /**
     * @brief setAgentOnMap - setup the agent point to a desired map
     * @param map- 2d grid map
     */
    void setAgentOnMap(std::shared_ptr<Map> &map);

    /**
     * @brief printPath - print the the path pose
     */
    void printPath();

    /**
     * @brief getId - get the agent id
     * @return id number
     */
    size_t getId() {return id_; };

    /**
     * @brief getPath - return the path vector<Pose>
     */
    std::vector<Pose> getPath() { return path_; };
    
    /**
     * @brief getCurrentPose - return the agent current pose
     */
    Pose getCurrentPose() { return curPose_; };
    
private:
    /**
     * @brief distToGoal - calculate the current pose distance to Goal
     * @param action - the agent's action from actions
     * @return the distance value
     */
    double distToGoal(Action action);

    /**
     * @brief getNextPlan - choose the next action that closest to the goal
     * @param currentActions - the agent's action from actions
     */
    Action getNextPlan(std::vector<Action> currentActions);
    
    /**
     * @brief reachGoal - true if reach the goal
     */
    bool reachGoal();

    /**
     * @brief preConductNextPlan - based on the result of  checkNextPlanAvailable
     */
    bool preConductNextPlan(Action action);

    /**
     * @brief checkNextPlanAvailable - check the next action if it is vaild, 
     * @return
     */
    bool checkNextPlanAvailable(Action action);

    /**
     * @brief takeAction - take one step action from actions
     * @param action
     */
    void takeAction(Action action); 

    /**
     * @brief removeAction - remove the current action from current action list
     * @param action the next_action that fails to move in the previous step
     * @param actions all the current action options
     * @return the remaining actions
     */
    std::vector<Action> removeAction(Action action, std::vector<Action> actions);

    /**
     * @brief oneStepfromGoal - check if only one step distance to goal
     */
    bool oneStepfromGoal(Action action);

    int id_;
    Pose curPose_;
    Pose goalPose_;
    std::shared_ptr<Map> map_ptr_;
    std::vector<Pose> path_;
    std::vector<Action> actions{Action(0,1), Action(0,-1), Action(1,0), Action(-1,0)};
};


#endif
