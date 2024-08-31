#include "nav_msgs/Odometry.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <ros/ros.h>
#include "Eigen/Eigen"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "cstring"

using namespace std;

Eigen::Vector3d cur_pos, cur_vel;
bool new_mission;
int benchmark_id; // 1 fast_planner 2 faster 3 bubble 4 super
void cmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr &msg) {
    cur_pos = Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z);
    cur_vel = Eigen::Vector3d(msg->velocity.x, msg->velocity.y, msg->velocity.z);
    if (cur_vel.norm() > 0.1) {
        new_mission = false;
    }
}

bool checkProcessExists(const std::string &processName) {
    std::string command = "pgrep -x " + processName;
    FILE *pipe = popen(command.c_str(), "r");
    if (!pipe) {
        std::cerr << "Error executing command." << std::endl;
        return false;
    }

    char buffer[128];
    std::string result;
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        result += buffer;
    }

    pclose(pipe);

    return !result.empty();
}


void stuckCheckCallback(const ros::TimerEvent &e) {
    static Eigen::Vector3d last_pos = cur_pos;
    static double last_live_time = ros::Time::now().toSec();
    string node_name;
    switch (benchmark_id) {
        case 1: {
            node_name = "fast_planner_no";
            break;
        }
        case 2: {
            node_name = "faster_node";
            break;
        }
        case 3: {
            node_name = "bench_node";
            break;
        }
        case 4: {
            node_name = "super_bench_nod";
            break;
        }
    }
    if (!checkProcessExists(string(node_name))) {
        static double last_t = ros::Time::now().toSec();
        double cur_t = ros::Time::now().toSec();
        if (cur_t - last_t > 1.0) {
            ROS_WARN("%s is not running", node_name.c_str());
            last_t = cur_t;
        }
        if (cur_t - last_live_time > 0.5) {
            if (benchmark_id == 1) {
                ROS_WARN("FastPlanner node died for more than 0.5 s, kill traj server");
                system("pkill -f traj_server");
            } else if (benchmark_id == 2) {
                ROS_WARN("Faster node died for more than 0.5 s, kill global mapper");
                system("pkill -f global_mapper_n");
            }
            exit(0);
        }
        new_mission = true;
    } else {
        static double last_t = ros::Time::now().toSec();
        double cur_t = ros::Time::now().toSec();
        last_live_time = cur_t;
        if (cur_t - last_t > 1.0) {
            ROS_WARN("%s is running", node_name.c_str());
            last_t = cur_t;
        }
    }

    if ((cur_pos - Eigen::Vector3d(0, -50, 1.5)).norm() < 1e-3) {
        new_mission = true;
        cur_vel.setZero();
    }

    static double last_pos_t = ros::Time::now().toSec();
    double cur_t = ros::Time::now().toSec();
    if (!new_mission &&
        (last_pos - cur_pos).norm() < 0.1 &&
        cur_t - last_pos_t > 15) {
        ROS_ERROR("Stuck for more than 15 s!");
        system("pkill -f fast_planner_node");
        system("pkill -f traj_server");
        system("pkill -f global_mapper_ros");
        system("pkill -f faster");
        system("pkill -f bench_node");
        system("pkill -f super_bench_nod");

        cur_vel.setZero();
        new_mission = true;
    }
    if ((last_pos - cur_pos).norm() > 0.01 && !new_mission) {
        last_pos_t = cur_t;
    }
    last_pos = cur_pos;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "stuck_monitor");
    ros::NodeHandle node("~");

    node.getParam("/planner_id", benchmark_id);

    ros::Subscriber pos_cmd_sub = node.subscribe("/planning/pos_cmd", 10, &cmdCallback);
    ros::Timer stuck_check_timer = node.createTimer(ros::Duration(0.01), stuckCheckCallback);


    ros::Duration(1.0).sleep();
    ros::spin();

    return 0;
}
