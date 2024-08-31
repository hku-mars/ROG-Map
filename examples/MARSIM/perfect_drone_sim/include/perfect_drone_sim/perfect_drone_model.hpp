#ifndef _PERFECT_DRONE_SIM_HPP_
#define _PERFECT_DRONE_SIM_HPP_

#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "nav_msgs/Odometry.h"
#include "tf2_ros/transform_broadcaster.h"
#include "string"
#include "Eigen/Dense"
#include "quadrotor_flatness.hpp"
#include "mutex"
#include "visualization_msgs/MarkerArray.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"

using Vec3f = Eigen::Vector3d;
using PclPoint = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PclPoint>;

typedef Eigen::Matrix<double, 3, 1> Vec3;
typedef Eigen::Matrix<double, 3, 3> Mat33;

typedef Eigen::Matrix<double, 3, 3> StatePVA;
typedef Eigen::Matrix<double, 3, 4> StatePVAJ;
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> DynamicMat;
typedef Eigen::MatrixX4d MatX4;
typedef std::pair<double, Vec3> TimePosPair;

typedef Eigen::Matrix3Xd PolyhedronV;
typedef Eigen::MatrixX4d PolyhedronH;
using namespace flatness;
using namespace std;

class PerfectDrone {
    const string RED = "\033[0;31m";
    const string RESET = "\033[0m";

private:
    template <typename Scalar_t>
    Eigen::Matrix<Scalar_t, 3, 1> quaternion_to_ypr(const Eigen::Quaternion<Scalar_t>& q_) {
        Eigen::Quaternion<Scalar_t> q = q_.normalized();

        Eigen::Matrix<Scalar_t, 3, 1> ypr;
        ypr(2) = atan2(2 * (q.w() * q.x() + q.y() * q.z()), 1 - 2 * (q.x() * q.x() + q.y() * q.y()));
        ypr(1) = asin(2 * (q.w() * q.y() - q.z() * q.x()));
        ypr(0) = atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.y() * q.y() + q.z() * q.z()));

        return ypr;
    }

public:
    PerfectDrone(ros::NodeHandle& n) {
        nh_ = n;
        n.param("mesh_resource", mesh_resource_, std::string("package://perfect_drone_sim/meshes/f250.dae"));
        n.getParam("init_position/x", position_.x());
        n.getParam("init_position/y", position_.y());
        n.getParam("init_position/z", position_.z());
        n.getParam("init_yaw", yaw_);
        n.getParam("mode", mode_);
        /// for rotor drag flatness
        n.getParam("flatness/dh", dh);
        n.getParam("flatness/dv", dv);
        n.getParam("flatness/veps", veps);
        n.getParam("flatness/cp", cp);
        n.getParam("flatness/mass", mass);
        n.getParam("flatness/grav", grav);

        cmd_sub_ = nh_.subscribe("/planning/pos_cmd", 100, &PerfectDrone::cmdCallback, this);

        fov_pub_ = nh_.advertise<visualization_msgs::Marker>("/fov_mkr", 100);
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/lidar_slam/odom", 100);
        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/lidar_slam/pose", 100);
        robot_pub_ = nh_.advertise<visualization_msgs::Marker>("robot", 100);
        df_pub_ = nh_.advertise<quadrotor_msgs::PositionCommand>("df", 100);

        flatness.reset(mass, grav, dh, dv, cp, veps);
        q_ = Eigen::AngleAxisd(yaw_, Vec3::UnitZ());
        odom_.header.frame_id = "world";
        odom_pub_timer_ = nh_.createTimer(ros::Duration(0.01), &PerfectDrone::publishOdom, this);
    }

    ~PerfectDrone() {}

private:
    double fov_upper_deg{-1}, fov_lower_deg{-1};
    ros::Subscriber cmd_sub_;

    visualization_msgs::Marker fov_mkr;
    ros::Publisher odom_pub_, robot_pub_, pose_pub_, df_pub_, fov_pub_;
    ros::Timer odom_pub_timer_;
    ros::NodeHandle nh_;
    Vec3 position_, velocity_;
    double yaw_;
    Eigen::Quaterniond q_;
    nav_msgs::Odometry odom_;
    std::string mesh_resource_;
    FlatnessMap flatness;

    static const int PVAJ_MODE = 1;
    static const int POLYTRAJ_MODE = 2;
    int mode_ = PVAJ_MODE;

    double dh{-1}, dv, veps, cp, mass, grav;

    void cmdCallback(const quadrotor_msgs::PositionCommandConstPtr& msg) {
        Vec3 pos(msg->position.x, msg->position.y, msg->position.z);
        Vec3 vel(msg->velocity.x, msg->velocity.y, msg->velocity.z);
        Vec3 acc(msg->acceleration.x, msg->acceleration.y, msg->acceleration.z);
        Vec3 jerk(msg->jerk.x, msg->jerk.y, msg->jerk.z);
        double yaw_dot = msg->yaw_dot;
        double yaw = msg->yaw;
        updateFlatness(pos, vel, acc, jerk, yaw, yaw_dot);
    }

    void publishOdom(const ros::TimerEvent& e) {
        odom_.pose.pose.position.x = position_.x();
        odom_.pose.pose.position.y = position_.y();
        odom_.pose.pose.position.z = position_.z();

        odom_.pose.pose.orientation.x = q_.x();
        odom_.pose.pose.orientation.y = q_.y();
        odom_.pose.pose.orientation.z = q_.z();
        odom_.pose.pose.orientation.w = q_.w();

        odom_.twist.twist.linear.x = velocity_.x();
        odom_.twist.twist.linear.y = velocity_.y();
        odom_.twist.twist.linear.z = velocity_.z();

        odom_.header.stamp = ros::Time::now();


        odom_pub_.publish(odom_);

        geometry_msgs::PoseStamped pose;
        pose.pose = odom_.pose.pose;
        pose.header = odom_.header;
        pose_pub_.publish(pose);

        static tf2_ros::TransformBroadcaster br_map_ego;
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = odom_.header.stamp;
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = "perfect_drone";
        transformStamped.transform.translation.x = odom_.pose.pose.position.x;
        transformStamped.transform.translation.y = odom_.pose.pose.position.y;
        transformStamped.transform.translation.z = odom_.pose.pose.position.z;
        transformStamped.transform.rotation.x = odom_.pose.pose.orientation.x;
        transformStamped.transform.rotation.y = odom_.pose.pose.orientation.y;
        transformStamped.transform.rotation.z = odom_.pose.pose.orientation.z;
        transformStamped.transform.rotation.w = odom_.pose.pose.orientation.w;
        br_map_ego.sendTransform(transformStamped);

        visualization_msgs::Marker meshROS;
        meshROS.header.frame_id = "world";
        meshROS.header.stamp = odom_.header.stamp;
        meshROS.ns = "mesh";
        meshROS.id = 0;
        meshROS.type = visualization_msgs::Marker::MESH_RESOURCE;
        meshROS.action = visualization_msgs::Marker::ADD;
        meshROS.pose.position = odom_.pose.pose.position;
        meshROS.pose.orientation = odom_.pose.pose.orientation;
        meshROS.scale.x = 1;
        meshROS.scale.y = 1;
        meshROS.scale.z = 1;
        meshROS.mesh_resource = mesh_resource_;
        meshROS.mesh_use_embedded_materials = true;
        meshROS.color.a = 1.0;
        meshROS.color.r = 1.0;
        meshROS.color.g = 1.0;
        meshROS.color.b = 1.0;
        robot_pub_.publish(meshROS);

        static int cnt = 0;
        if (cnt++ > 10) {
            fov_mkr.pose = odom_.pose.pose;
            fov_mkr.header.stamp = ros::Time::now();
            fov_pub_.publish(fov_mkr);
            cnt = 0;
        }
    }

    void updateFlatness(const Vec3& pos, const Vec3& vel,
                        const Vec3& acc, const Vec3& jer, const double yaw, const double yaw_dot) {
        Eigen::Vector3d omg;
        Eigen::Vector4d quat;
        double thr;
        if (dh < 0) {
            Vec3 gravity_ = 9.80 * Eigen::Vector3d(0, 0, 1);
            position_ = pos;
            velocity_ = vel;
            double a_T = (gravity_ + acc).norm();
            Eigen::Vector3d xB, yB, zB;
            Eigen::Vector3d xC(cos(yaw), sin(yaw), 0);

            zB = (gravity_ + acc).normalized();
            yB = ((zB).cross(xC)).normalized();
            xB = yB.cross(zB);
            Eigen::Matrix3d R;
            R << xB, yB, zB;
            q_ = Eigen::Quaterniond(R);
        }
        else {
            flatness.forward(vel, acc, jer, yaw, yaw_dot, thr, quat, omg);
            position_ = pos;
            velocity_ = vel;
            yaw_ = yaw;
            thr /= mass;
            q_ = Eigen::Quaterniond(quat[0], quat[1], quat[2], quat[3]);
        }

        quadrotor_msgs::PositionCommand df;
        df.position.x = position_.x();
        df.position.y = position_.y();
        df.position.z = position_.z();
        df.velocity.x = velocity_.x();
        df.velocity.y = velocity_.y();
        df.velocity.z = velocity_.z();
        df.acceleration.x = acc.x();
        df.acceleration.y = acc.y();
        df.acceleration.z = acc.z();
        df.jerk.x = jer.x();
        df.jerk.y = jer.y();
        df.jerk.z = jer.z();
        df.vel_norm = velocity_.norm();
        df.acc_norm = acc.norm();

        // conver to euler
        Eigen::Vector3d euler = quaternion_to_ypr(q_);
        df.attitude.x = euler[2] * 180 / M_PI;
        df.attitude.y = euler[1] * 180 / M_PI;
        df.attitude.z = euler[0] * 180 / M_PI;
        df.angular_velocity.x = omg[0];
        df.angular_velocity.y = omg[1];
        df.angular_velocity.z = omg[2];
        df.thrust.x = omg.norm();
        df.yaw = yaw;
        df.yaw_dot = yaw_dot;
        df.thrust.z = thr;
        df.header.stamp = ros::Time::now();
        df.header.frame_id = "world";
        df_pub_.publish(df);
    }
};

#endif
