#ifndef MAP_H
#define MAP_H

#include <iostream>
#include <thread>
#include <vector>
#include <future>
#include <mutex>
#include <cmath>
#include "utility.h"

/**
 * @brief Map - 2d grid Map
*/ 
class Map{
public:
    Map(int length, int width) : length_(length), width_(width), map_(length, std::vector<int> (width, 0)) {};
    /**
     * @brief setOccupied - set the agent pose onto the map
     * @param pose - struct with two variabels int x, y
     */
    void setOccupied(Pose pose);
    /**
     * @brief readMap - check the map with given pose, 1 means taken, 0 means empty
     * @param pose - struct with two variabels int x, y
     */
    int readMap(Pose pose);
    
    /**
     * @brief getSize - get the map dimension
     */
    Pose getSize();
    // for lock the map to read and write
    std::mutex mtx_;
private:
    int length_;
    int width_;
    std::vector<std::vector<int>> map_;
};

#endif