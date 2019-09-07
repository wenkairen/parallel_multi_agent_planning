#include "map.h"


Pose Map::getSize()
{
    std::lock_guard<std::mutex> lck(mtx_);
    Pose p;
    p.x = length_;
    p.y = width_;
    return p;
};

int Map::readMap(Pose pose)
{
    std::unique_lock<std::mutex> lck(mtx_);;
    int idx = map_[pose.x][pose.y];
    lck.unlock();
    return idx;
};

void Map::setOccupied(Pose pose)
{
    std::lock_guard<std::mutex> lck(mtx_);
    // Check the pose whether within the map boundary
    if( pose.x < 0 || pose.x >= length_ || pose.y < 0 || pose.y >= width_)
    {
        throw std::runtime_error("Invaild Input");
    }
    else
    {
        map_[pose.x][pose.y] = 1;
    }
};
