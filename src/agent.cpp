#include "agent.h"
#include "map.h"
#include "utility.h"

void Agent::planning()
{
    std::unique_lock<std::mutex> lck(map_ptr_->mtx_);
    std::cout << " [Agent thread id " + std::to_string(id_)  + " ]: " << std::this_thread::get_id() << std::endl;
    lck.unlock();
    // check the next best action based on 4 directions.
    Action next_action = getNextPlan(actions);
    auto curActions = actions;
    // count the actions that been used in each attempt
    int count = 1;
    // run through the loop till reach goal 
    while (!reachGoal())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        
        // if the current pose is one action from goal, then we think the agent is able to reach the goal
        if (oneStepfromGoal(next_action))
        {
            path_.push_back(goalPose_);
            break;
        }

        // check the best action and see if the pose we gonna at if taken or not
        // if the next_action is not taken by other agent, we take the step and try to move the next best option
        // else the next_action resulting pose is taken, then we need to switch the left 3 possible actions
        // if all the directions are unaviable and the count is 4
        // whhich means the 4 potential poses around the agent are all taken, so no solution
        if (preConductNextPlan(next_action))
        {
            takeAction(next_action);
            path_.push_back(curPose_);
            next_action = getNextPlan(actions);
            curActions = actions;
            count = 1;
        }
        else{
            curActions = removeAction(next_action, curActions);
            if (curActions.empty() || count == 4)
            {
                std::cout << "[Error 0] Can't find valid path" << std::endl;
            }
            next_action = getNextPlan(curActions);
            count++;
        }

    }
    // print the final path
    printPath();
};

bool Agent::oneStepfromGoal(Action action)
{
    int dx = std::abs(curPose_.x + action.dx - goalPose_.x);
    int dy = std::abs(curPose_.y + action.dy - goalPose_.y);
    if (dx < 1 && dy <= 1)
    {
        return true;
    }
    return false;
};

std::vector<Action> Agent::removeAction(Action action, std::vector<Action> actions)
{
    std::vector<Action> new_actions;
    for (auto a : actions)
    {
        if (a.dx == action.dx && a.dy == action.dy)
        {
            continue;
        }
        new_actions.push_back(a);
    }
    return new_actions;
};

void Agent::printPath()
{
    std::unique_lock<std::mutex> lck(map_ptr_->mtx_);
    std::cout << "Successfully finding path -> " << "[ Path " + std::to_string(id_) + "]" << " ";
    for (auto& p : path_){
        std::cout << "[" + std::to_string(p.x) + "," + std::to_string(p.y) + "]" << " ";
    }
    std::cout << " " << std::endl;
    lck.unlock();
};

bool Agent::reachGoal()
{
    if (curPose_.x == goalPose_.x && curPose_.y == goalPose_.y)
    {
        return true;
    }
    return false;
};

void Agent::setAgentOnMap(std::shared_ptr<Map> &map)
{
    map_ptr_ = map;
};

double Agent::distToGoal(Action action)
{
    double dfx = std::pow(curPose_.x + action.dx - goalPose_.x, 2);
    double dfy = std::pow(curPose_.y + action.dy - goalPose_.y, 2);
    double dist = sqrt(dfx + dfy);
    return dist;
};

Action Agent::getNextPlan(std::vector<Action> currentActions)
{
    // find the actction that nearst to goal;
    double minDist = std::numeric_limits<double>::max();
    Action minAction(0,0);
    for (auto action : currentActions)
    {
        double dist = distToGoal(action);
        if (dist < minDist)
        {
            minDist = dist;
            minAction.dx = action.dx;
            minAction.dy = action.dy;
        }
    }
    return minAction;
};

void Agent::takeAction(Action action)
{
    curPose_.x += action.dx;
    curPose_.y += action.dy;
};

bool Agent::checkNextPlanAvailable(Action action)
{
    Pose pose;
    pose.x = curPose_.x + action.dx;
    pose.y = curPose_.y + action.dy;
    auto tempPose = map_ptr_->getSize();
    if (pose.x < 0 || pose.x >= tempPose.x || pose.y < 0 || pose.y >= tempPose.y)
    {
        return false;
    }
    int idx = map_ptr_->readMap(pose);
    if(idx == 1){
        return false;
    }
    else
    {
        return true;
    }
};

bool Agent::preConductNextPlan(Action action)
{
    if (checkNextPlanAvailable(action))
    {
        Pose p;
        p.x = curPose_.x + action.dx;
        p.y = curPose_.y + action.dy;
        map_ptr_->setOccupied(p);
        return true;
    }
    else
    {
        return false;
    }
};
