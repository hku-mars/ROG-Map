#include "perfect_drone_sim/perfect_drone_model.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "perfect_tracking");
    ros::NodeHandle n("~");
    PerfectDrone dp(n);
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
