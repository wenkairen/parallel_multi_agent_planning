add_library(parallel_multi_agent_planning SHARED map.cpp agent.cpp utility.cpp)
target_link_libraries(parallel_multi_agent_planning ${catkin_LIBRARIES})

add_executable(simulate_process simulate_process.cpp)
target_link_libraries(simulate_process parallel_multi_agent_planning)