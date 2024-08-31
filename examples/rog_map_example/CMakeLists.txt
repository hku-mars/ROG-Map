cmake_minimum_required(VERSION 2.8.3)
project(rog_map_example)

set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3 -Wall -g -fPIC")
add_compile_options(-DQT_NO_VERSION_TAGGING)
add_compile_options(-Werror=unused-variable)
add_compile_options(-Werror=return-type)

add_definitions(-DORIGIN_AT_CORNER)
#add_definitions(-DORIGIN_AT_CENTER)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        pcl_ros
        geometry_msgs
        nav_msgs
        rog_map
)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        #        LIBRARIES rog_map
        #        CATKIN_DEPENDS roscpp rospy std_msgs
        #  DEPENDS system_lib
)

include_directories(
        SYSTEM
        include
        ../../../../devel/include
        ${catkin_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

set(THIRD_PARTY
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        -ldw
)

add_library(kdtree
        include/utils/kdtree.c
)


add_executable(rrt_example_node
        Apps/rrt_example_node.cpp
)
target_link_libraries(rrt_example_node
        ${THIRD_PARTY}
        kdtree
)

add_executable(astar_example_node
        Apps/astar_example_node.cpp
)
target_link_libraries(astar_example_node
        ${THIRD_PARTY}
)

add_executable(marsim_example_node
        Apps/marsim_example_node.cpp
)
target_link_libraries(marsim_example_node
        ${THIRD_PARTY}
)