#include "quadrotor_msgs/PositionCommand.h"
#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_datatypes.h>

class Interface
{
public:
    Interface();

private:
    ros::NodeHandle nh;
    ros::Publisher pub,pub2;
    ros::Subscriber sub,sub2;

    quadrotor_msgs::PositionCommand cmd,cmd2;
    int _n_seq;

    void messageCallback(const geometry_msgs::PoseStampedConstPtr &msg);
    void messageCallback2(const geometry_msgs::PoseStampedConstPtr &msg);
};

Interface::Interface() {
    pub = nh.advertise<quadrotor_msgs::PositionCommand>
            ("/planning/pos_cmd_1", 10);
    pub2 = nh.advertise<quadrotor_msgs::PositionCommand>
            ("/planning/pos_cmd_2", 10);
    sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/quadrotor_1_pos_cmd", 10, &Interface::messageCallback, this);
            ///move_base_simple/goal
    sub2 = nh.subscribe<geometry_msgs::PoseStamped>
            ("/quadrotor_2_pos_cmd", 10, &Interface::messageCallback2, this);

    _n_seq = 0;

    /* kP */
    // double pos_gain[3] = { 5.7, 5.7, 6.2 };
    // double vel_gain[3] = { 3.4, 3.4, 4.0 };

    double pos_gain[3] = { 7, 7, 6.2 };
    double vel_gain[3] = { 4, 4, 4.0 };

    /* control parameter */
    cmd.kx[0] = pos_gain[0];
    cmd.kx[1] = pos_gain[1];
    cmd.kx[2] = pos_gain[2];

    cmd.kv[0] = vel_gain[0];
    cmd.kv[1] = vel_gain[1];
    cmd.kv[2] = vel_gain[2];

        /* control parameter */
    cmd2.kx[0] = pos_gain[0];
    cmd2.kx[1] = pos_gain[1];
    cmd2.kx[2] = pos_gain[2];

    cmd2.kv[0] = vel_gain[0];
    cmd2.kv[1] = vel_gain[1];
    cmd2.kv[2] = vel_gain[2];

    ros::spin();
}

void Interface::messageCallback(const geometry_msgs::PoseStampedConstPtr &msg) {
    // header
    cmd.header.stamp = msg->header.stamp;
    cmd.header.frame_id = "world";

    cmd.trajectory_id = 0;
    cmd.trajectory_flag = 1;
    cmd.position.x = msg->pose.position.x;
    cmd.position.y = msg->pose.position.y;
    cmd.position.z = msg->pose.position.z;
    cmd.velocity.x = 0;
    cmd.velocity.y = 0;
    cmd.velocity.z = 0;
    cmd.acceleration.x = 0;
    cmd.acceleration.y = 0;
    cmd.acceleration.z = 0;

    tf::Quaternion quat;
    double roll,pitch,yaw;
    tf::quaternionMsgToTF(msg->pose.orientation,quat);
    tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);   

    cmd.yaw = yaw;

    pub.publish(cmd);

    // cmd.position.y = msg->pose.position.y + 2;

    // cmd.yaw = -1.7;

    // pub2.publish(cmd);
}

void Interface::messageCallback2(const geometry_msgs::PoseStampedConstPtr &msg) {
    // header
    cmd2.header.stamp = msg->header.stamp;
    cmd2.header.frame_id = "world";

    cmd2.trajectory_id = 0;
    cmd2.trajectory_flag = 1;
    cmd2.position.x = msg->pose.position.x;
    cmd2.position.y = msg->pose.position.y;
    cmd2.position.z = msg->pose.position.z;
    cmd2.velocity.x = 0;
    cmd2.velocity.y = 0;
    cmd2.velocity.z = 0;
    cmd2.acceleration.x = 0;
    cmd2.acceleration.y = 0;
    cmd2.acceleration.z = 0;

    tf::Quaternion quat;
    double roll,pitch,yaw;
    tf::quaternionMsgToTF(msg->pose.orientation,quat);
    tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);   

    cmd2.yaw = yaw;

    pub2.publish(cmd2);
}

int main(int argc, char** argv)
{
    ROS_WARN("*****START*****");
    ros::init(argc, argv, "test_interface");
    Interface Int;

    return 0;
}