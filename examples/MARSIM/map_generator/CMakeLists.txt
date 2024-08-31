cmake_minimum_required(VERSION 2.8.3)
project(map_generator)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        geometry_msgs
        pcl_conversions
        rospack
        roslib
        rosfmt
)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
        CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
)


set(THIRD_PARTY
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
)


add_executable(map_pub src/map_publisher.cpp)
target_link_libraries(map_pub
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES})