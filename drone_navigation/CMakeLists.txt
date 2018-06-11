cmake_minimum_required(VERSION 2.8.3)
project(drone_navigation)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    visualization_msgs
    sensor_msgs
    pcl_ros
    pcl_conversions
    nav_msgs
    std_msgs
    std_srvs
)

find_package(PCL REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

add_library(path_planner src/PathPlanner.cpp)
add_library(pathfinder src/Pathfinder.cpp)
add_library(block_allocator src/BlockAllocator.cpp)
add_library(costmap src/Costmap.cpp)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package( )

add_executable(drone_odom src/drone_odom.cpp)
target_link_libraries(drone_odom ${catkin_LIBRARIES})

add_executable(drone_navigation src/drone_navigation.cpp)
target_link_libraries(drone_navigation path_planner pathfinder block_allocator costmap ${catkin_LIBRARIES})
