#include "rog_map/rog_map.h"


int main(int argc, char** argv) {
    ros::init(argc, argv, "rm_node");
    ros::NodeHandle nh("~");

    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    /* 1. Creat a ROGMap ptr*/
    rog_map::ROGMap::Ptr rog_map_ptr = std::make_shared<rog_map::ROGMap>(nh);

    /* Publisher and subcriber */
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::Duration(1.0).sleep();


    ros::waitForShutdown();
    return 0;
}
