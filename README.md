# CPPND: Capstone parallel_multi_agent_planning Repo

## Dependencies for Running Locally
* cmake >= 3.7
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions
### the code must run in ROS kineitc version on Ubuntu 16.04
1. In your catkin_ws workspace: `cd  ~/catkin_ws/src
2. Clone this repo.
3  open the folder: ` cd parallel_multi_agent_planning
4. Make a build directory : `mkdir build && cd build`
4. Compile: `cmake .. && make -j8`
5. Then switch to top catkin_workspace: cd ~/catkin_ws
6. check: `ls
7. you should be able to see: `build devel src
8. Then run catkin_make: `catkin_make
9. source the enviroment: ` source devel/setup.bash
10.ros launch file to see the planning path in rviz: `oslaunch parallel_multi_agent_planning planning.launch
