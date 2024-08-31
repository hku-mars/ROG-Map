/*
Copyright (C) 2020 Jialin Ji ()
              2021 Hongkai Ye (kyle_yeh@163.com)
              2024 Yunfan REN
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/
#ifndef ROS_VISUALIZATION_UTILS
#define ROS_VISUALIZATION_UTILS

#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Geometry>
#include <iostream>
#include <unordered_map>

namespace visualization {
    using std::string;
    using std::vector;

    struct BALL {
        Eigen::Vector3d center;
        double radius;
        BALL(const Eigen::Vector3d& c, double r) : center(c), radius(r) {};
        BALL() {};
    };

    struct ELLIPSOID {
        Eigen::Vector3d c;
        double rx, ry, rz;
        Eigen::Matrix3d R;

        ELLIPSOID(const Eigen::Vector3d& center, const Eigen::Vector3d& r, const Eigen::Matrix3d& rot)
            : c(center), rx(r.x()), ry(r.y()), rz(r.z()), R(rot) {};
        ELLIPSOID() {};
    };

    using PublisherMap = std::unordered_map<std::string, ros::Publisher>;
    using TypeMap = std::unordered_map<std::string, std::string>;

    enum Color {
        white,
        red,
        green,
        blue,
        yellow,
        chartreuse,
        black,
        gray,
        orange,
        purple,
        pink,
        steelblue
    };

    class Visualization {
    private:
        ros::NodeHandle nh_;
        PublisherMap publisher_map_;
        TypeMap type_map_;

        void setMarkerColor(visualization_msgs::Marker& marker,
                            Color color = blue,
                            double a = 1) {
            marker.color.a = a;
            switch (color) {
            case white:
                marker.color.r = 1;
                marker.color.g = 1;
                marker.color.b = 1;
                break;
            case red:
                marker.color.r = 1;
                marker.color.g = 0;
                marker.color.b = 0;
                break;
            case green:
                marker.color.r = 0;
                marker.color.g = 1;
                marker.color.b = 0;
                break;
            case blue:
                marker.color.r = 0;
                marker.color.g = 0;
                marker.color.b = 1;
                break;
            case yellow:
                marker.color.r = 1;
                marker.color.g = 1;
                marker.color.b = 0;
                break;
            case chartreuse:
                marker.color.r = 0.5;
                marker.color.g = 1;
                marker.color.b = 0;
                break;
            case black:
                marker.color.r = 0;
                marker.color.g = 0;
                marker.color.b = 0;
                break;
            case gray:
                marker.color.r = 0.5;
                marker.color.g = 0.5;
                marker.color.b = 0.5;
                break;
            case orange:
                marker.color.r = 1;
                marker.color.g = 0.5;
                marker.color.b = 0;
                break;
            case purple:
                marker.color.r = 0.5;
                marker.color.g = 0;
                marker.color.b = 1;
                break;
            case pink:
                marker.color.r = 1;
                marker.color.g = 0;
                marker.color.b = 0.6;
                break;
            case steelblue:
                marker.color.r = 0.4;
                marker.color.g = 0.7;
                marker.color.b = 1;
                break;
            }
        }

        void setMarkerColor(visualization_msgs::Marker& marker,
                            double a,
                            double r,
                            double g,
                            double b) {
            marker.color.a = a;
            marker.color.r = r;
            marker.color.g = g;
            marker.color.b = b;
        }

        void setMarkerScale(visualization_msgs::Marker& marker,
                            const double& x,
                            const double& y,
                            const double& z) {
            marker.scale.x = x;
            marker.scale.y = y;
            marker.scale.z = z;
        }

        void setMarkerPose(visualization_msgs::Marker& marker,
                           const double& x,
                           const double& y,
                           const double& z) {
            marker.pose.position.x = x;
            marker.pose.position.y = y;
            marker.pose.position.z = z;
            marker.pose.orientation.w = 1;
            marker.pose.orientation.x = 0;
            marker.pose.orientation.y = 0;
            marker.pose.orientation.z = 0;
        }

        template <class ROTATION>
        void setMarkerPose(visualization_msgs::Marker& marker,
                           const double& x,
                           const double& y,
                           const double& z,
                           const ROTATION& R) {
            marker.pose.position.x = x;
            marker.pose.position.y = y;
            marker.pose.position.z = z;
            Eigen::Quaterniond r(R);
            marker.pose.orientation.w = r.w();
            marker.pose.orientation.x = r.x();
            marker.pose.orientation.y = r.y();
            marker.pose.orientation.z = r.z();
        }

    public:
        Visualization(ros::NodeHandle& nh) : nh_(nh) {}

        void clearMarker(const string& topic) {
            const auto publisher = publisher_map_.find(topic);
            const auto type = type_map_.find(topic);
            auto& pub = publisher->second;
            if (publisher != publisher_map_.end() && type != type_map_.end()) {
                if (type->second == "MarkerArray") {
                    visualization_msgs::MarkerArray mkr;
                    mkr.markers.resize(1);
                    mkr.markers[0].type = visualization_msgs::Marker::DELETEALL;
                    pub.publish(mkr);
                }
                else if ("Marker") {
                    visualization_msgs::Marker mkr;
                    mkr.action = visualization_msgs::Marker::DELETEALL;
                    pub.publish(mkr);
                }
            }
        }

        template <class CENTER, class TOPIC>
        void visualize_a_ball(const CENTER& c,
                              const double& r,
                              const TOPIC& topic,
                              const Color color = blue,
                              const double a = 1,
                              const int idx = 0) {
            auto got = publisher_map_.find(topic);
            if (got == publisher_map_.end()) {
                ros::Publisher pub = nh_.advertise<visualization_msgs::Marker>(topic, 10);
                publisher_map_[topic] = pub;
                type_map_[topic] = "Marker";
                ros::Duration(0.1).sleep();
            }
            visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.id = idx;
            setMarkerColor(marker, color, a);
            setMarkerScale(marker, 2 * r, 2 * r, 2 * r);
            setMarkerPose(marker, c[0], c[1], c[2]);
            marker.header.stamp = ros::Time::now();
            publisher_map_[topic].publish(marker);
        }

        template <class PC, class TOPIC>
        void visualize_pointcloud(const PC& pc, const TOPIC& topic) {
            auto got = publisher_map_.find(topic);
            if (got == publisher_map_.end()) {
                ros::Publisher pub = nh_.advertise<sensor_msgs::PointCloud2>(topic, 10);
                publisher_map_[topic] = pub;
                type_map_[topic] = "PointCloud2";
                ros::Duration(0.1).sleep();
            }
            pcl::PointCloud<pcl::PointXYZ> point_cloud;
            sensor_msgs::PointCloud2 point_cloud_msg;
            point_cloud.reserve(pc.size());
            for (const auto& pt : pc) {
                point_cloud.points.emplace_back(pt[0], pt[1], pt[2]);
            }
            pcl::toROSMsg(point_cloud, point_cloud_msg);
            point_cloud_msg.header.frame_id = "world";
            point_cloud_msg.header.stamp = ros::Time::now();
            publisher_map_[topic].publish(point_cloud_msg);
        }

        template <class PATH, class TOPIC>
        void visualize_path(const PATH& path, const TOPIC& topic) {
            auto got = publisher_map_.find(topic);
            if (got == publisher_map_.end()) {
                ros::Publisher pub = nh_.advertise<nav_msgs::Path>(topic, 10);
                publisher_map_[topic] = pub;
                type_map_[topic] = "Path";
                ros::Duration(0.1).sleep();
            }
            nav_msgs::Path path_msg;
            geometry_msgs::PoseStamped tmpPose;
            tmpPose.header.frame_id = "world";
            for (const auto& pt : path) {
                tmpPose.pose.position.x = pt[0];
                tmpPose.pose.position.y = pt[1];
                tmpPose.pose.position.z = pt[2];
                path_msg.poses.push_back(tmpPose);
            }
            path_msg.header.frame_id = "world";
            path_msg.header.stamp = ros::Time::now();
            publisher_map_[topic].publish(path_msg);
        }

        template <class BALLS, class TOPIC>
        void visualize_balls(const BALLS& balls,
                             const TOPIC& topic,
                             const Color color = blue,
                             const double a = 0.2) {
            auto got = publisher_map_.find(topic);
            if (got == publisher_map_.end()) {
                ros::Publisher pub =
                    nh_.advertise<visualization_msgs::MarkerArray>(topic, 10);
                publisher_map_[topic] = pub;
                type_map_[topic] = "MarkerArray";
                ros::Duration(0.1).sleep();
            }
            visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.id = 0;
            setMarkerColor(marker, color, a);
            visualization_msgs::MarkerArray marker_array;
            marker_array.markers.reserve(balls.size() + 1);
            marker.action = visualization_msgs::Marker::DELETEALL;
            marker_array.markers.push_back(marker);
            marker.action = visualization_msgs::Marker::ADD;
            for (const auto& ball : balls) {
                setMarkerPose(marker, ball.center[0], ball.center[1], ball.center[2]);
                auto d = 2 * ball.radius;
                setMarkerScale(marker, d, d, d);
                marker_array.markers.push_back(marker);
                marker.id++;
            }
            publisher_map_[topic].publish(marker_array);
        }

        template <class ELLIPSOIDS, class TOPIC>
        void visualize_ellipsoids(const ELLIPSOIDS& ellipsoids,
                                  const TOPIC& topic,
                                  const Color color = blue,
                                  const double a = 0.2) {
            auto got = publisher_map_.find(topic);
            if (got == publisher_map_.end()) {
                ros::Publisher pub =
                    nh_.advertise<visualization_msgs::MarkerArray>(topic, 10);
                publisher_map_[topic] = pub;
                type_map_[topic] = "MarkerArray";
                ros::Duration(0.1).sleep();
            }
            visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.id = 0;
            setMarkerColor(marker, color, a);
            visualization_msgs::MarkerArray marker_array;
            marker_array.markers.reserve(ellipsoids.size() + 1);
            marker.action = visualization_msgs::Marker::DELETEALL;
            marker_array.markers.push_back(marker);
            marker.action = visualization_msgs::Marker::ADD;
            for (const auto& e : ellipsoids) {
                setMarkerPose(marker, e.c[0], e.c[1], e.c[2], e.R);
                setMarkerScale(marker, 2 * e.rx, 2 * e.ry, 2 * e.rz);
                marker_array.markers.push_back(marker);
                marker.id++;
            }
            publisher_map_[topic].publish(marker_array);
        }

        template <class PAIRLINE, class TOPIC>
        // eg for PAIRLINE: std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
        void visualize_pairline(const PAIRLINE& pairline, const TOPIC& topic, const Color& color = green,
                                double scale = 0.1,
                                int id = 0) {
            auto got = publisher_map_.find(topic);
            if (got == publisher_map_.end()) {
                ros::Publisher pub = nh_.advertise<visualization_msgs::Marker>(topic, 10);
                publisher_map_[topic] = pub;
                type_map_[topic] = "Marker";
                ros::Duration(0.1).sleep();
            }
            visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.type = visualization_msgs::Marker::LINE_LIST;
            marker.action = visualization_msgs::Marker::ADD;
            marker.id = id;
            setMarkerPose(marker, 0, 0, 0);
            setMarkerColor(marker, color, 1);
            setMarkerScale(marker, scale, scale, scale);
            marker.points.resize(2 * pairline.size());
            for (size_t i = 0; i < pairline.size(); ++i) {
                marker.points[2 * i + 0].x = pairline[i].first[0];
                marker.points[2 * i + 0].y = pairline[i].first[1];
                marker.points[2 * i + 0].z = pairline[i].first[2];
                marker.points[2 * i + 1].x = pairline[i].second[0];
                marker.points[2 * i + 1].y = pairline[i].second[1];
                marker.points[2 * i + 1].z = pairline[i].second[2];
            }
            publisher_map_[topic].publish(marker);
        }

        template <class ARROWS, class TOPIC>
        // ARROWS: pair<Vector3d, Vector3d>
        void visualize_arrows(const ARROWS& arrows, const TOPIC& topic, const Color& color) {
            auto got = publisher_map_.find(topic);
            if (got == publisher_map_.end()) {
                ros::Publisher pub =
                    nh_.advertise<visualization_msgs::MarkerArray>(topic, 10);
                publisher_map_[topic] = pub;
                type_map_[topic] = "MarkerArray";
                ros::Duration(0.1).sleep();
            }
            visualization_msgs::Marker clear_previous_msg;
            clear_previous_msg.action = visualization_msgs::Marker::DELETEALL;
            visualization_msgs::Marker arrow_msg;
            arrow_msg.type = visualization_msgs::Marker::ARROW;
            arrow_msg.action = visualization_msgs::Marker::ADD;
            arrow_msg.header.frame_id = "world";
            arrow_msg.id = 0;
            arrow_msg.points.resize(2);
            setMarkerPose(arrow_msg, 0, 0, 0);
            setMarkerScale(arrow_msg, 0.4, 0.7, 0);
            setMarkerColor(arrow_msg, color, 0.7);
            visualization_msgs::MarkerArray arrow_list_msg;
            arrow_list_msg.markers.reserve(1 + arrows.size());
            arrow_list_msg.markers.push_back(clear_previous_msg);
            for (const auto& arrow : arrows) {
                arrow_msg.points[0].x = arrow.first[0];
                arrow_msg.points[0].y = arrow.first[1];
                arrow_msg.points[0].z = arrow.first[2];
                arrow_msg.points[1].x = arrow.second[0];
                arrow_msg.points[1].y = arrow.second[1];
                arrow_msg.points[1].z = arrow.second[2];
                arrow_list_msg.markers.push_back(arrow_msg);
                arrow_msg.id += 1;
            }
            publisher_map_[topic].publish(arrow_list_msg);
        }

        template <class TOPIC>
        //
        void visualize_bounding_box(const Eigen::Vector3d& box_min, const Eigen::Vector3d& box_max,
                                    const TOPIC& topic, const Color& color, const double size_x = 0.1) {
            auto got = publisher_map_.find(topic);
            if (got == publisher_map_.end()) {
                ros::Publisher pub =
                    nh_.advertise<visualization_msgs::MarkerArray>(topic, 10);
                publisher_map_[topic] = pub;
                type_map_[topic] = "MarkerArray";
                ros::Duration(0.1).sleep();
            }
            visualization_msgs::Marker clear_previous_msg;
            visualization_msgs::MarkerArray mkrarr;
            clear_previous_msg.action = visualization_msgs::Marker::DELETEALL;
            mkrarr.markers.push_back(clear_previous_msg);
            Eigen::Vector3d size = (box_max - box_min) / 2;
            Eigen::Vector3d vis_pos_world = (box_min + box_max) / 2;
            double width = size.x();
            double length = size.y();
            double hight = size.z();
            //Publish Bounding box
            int id = 0;
            visualization_msgs::Marker line_strip;
            line_strip.header.stamp = ros::Time::now();
            line_strip.header.frame_id = "world";
            line_strip.action = visualization_msgs::Marker::ADD;
            line_strip.ns = topic;
            line_strip.pose.orientation.w = 1.0;
            line_strip.id = id++; //unique id, useful when multiple markers exist.
            line_strip.type = visualization_msgs::Marker::LINE_STRIP; //marker type
            line_strip.scale.x = size_x;
            setMarkerColor(line_strip, color);
            line_strip.color.a = 1.0; //不透明度，设0则全透明
            geometry_msgs::Point p[8];

            //vis_pos_world是目标物的坐标
            p[0].x = vis_pos_world(0) - width;
            p[0].y = vis_pos_world(1) + length;
            p[0].z = vis_pos_world(2) + hight;
            p[1].x = vis_pos_world(0) - width;
            p[1].y = vis_pos_world(1) - length;
            p[1].z = vis_pos_world(2) + hight;
            p[2].x = vis_pos_world(0) - width;
            p[2].y = vis_pos_world(1) - length;
            p[2].z = vis_pos_world(2) - hight;
            p[3].x = vis_pos_world(0) - width;
            p[3].y = vis_pos_world(1) + length;
            p[3].z = vis_pos_world(2) - hight;
            p[4].x = vis_pos_world(0) + width;
            p[4].y = vis_pos_world(1) + length;
            p[4].z = vis_pos_world(2) - hight;
            p[5].x = vis_pos_world(0) + width;
            p[5].y = vis_pos_world(1) - length;
            p[5].z = vis_pos_world(2) - hight;
            p[6].x = vis_pos_world(0) + width;
            p[6].y = vis_pos_world(1) - length;
            p[6].z = vis_pos_world(2) + hight;
            p[7].x = vis_pos_world(0) + width;
            p[7].y = vis_pos_world(1) + length;
            p[7].z = vis_pos_world(2) + hight;
            //LINE_STRIP类型仅仅将line_strip.points中相邻的两个点相连，如0和1，1和2，2和3
            for (int i = 0; i < 8; i++) {
                line_strip.points.push_back(p[i]);
            }
            //为了保证矩形框的八条边都存在：
            line_strip.points.push_back(p[0]);
            line_strip.points.push_back(p[3]);
            line_strip.points.push_back(p[2]);
            line_strip.points.push_back(p[5]);
            line_strip.points.push_back(p[6]);
            line_strip.points.push_back(p[1]);
            line_strip.points.push_back(p[0]);
            line_strip.points.push_back(p[7]);
            line_strip.points.push_back(p[4]);
            mkrarr.markers.push_back(line_strip);
            publisher_map_[topic].publish(mkrarr);
        }

        template <class TRAJ, class TOPIC>
        // TRAJ:
        void visualize_traj(const TRAJ& traj, const TOPIC& topic) {
            std::vector<Eigen::Vector3d> path;
            auto duration = traj.getTotalDuration();
            for (double t = 0; t < duration; t += 0.01) {
                path.push_back(traj.getPos(t));
            }
            visualize_path(path, topic);
            std::vector<Eigen::Vector3d> wayPts;
            for (const auto& piece : traj) {
                wayPts.push_back(piece.getPos(0));
            }
            visualize_pointcloud(wayPts, std::string(topic) + "_wayPts");
        }

        template <class TRAJLIST, class TOPIC>
        // TRAJLIST: std::vector<TRAJ>
        void visualize_traj_list(const TRAJLIST& traj_list, const TOPIC& topic,
                                 const Color color = blue, double scale = 0.1) {
            auto got = publisher_map_.find(topic);
            if (got == publisher_map_.end()) {
                ros::Publisher pub =
                    nh_.advertise<visualization_msgs::MarkerArray>(topic, 10);
                publisher_map_[topic] = pub;
                type_map_[topic] = "MarkerArray";
                ros::Duration(0.1).sleep();
            }
            visualization_msgs::Marker clear_previous_msg;
            clear_previous_msg.action = visualization_msgs::Marker::DELETEALL;
            visualization_msgs::Marker path_msg;
            path_msg.type = visualization_msgs::Marker::LINE_STRIP;
            path_msg.action = visualization_msgs::Marker::ADD;
            path_msg.header.frame_id = "world";
            path_msg.id = 0;
            setMarkerPose(path_msg, 0, 0, 0);
            setMarkerScale(path_msg, scale, scale, scale);
            visualization_msgs::MarkerArray path_list_msg;
            path_list_msg.markers.reserve(1 + traj_list.size());
            path_list_msg.markers.push_back(clear_previous_msg);
            double a_step = 0.8 / traj_list.size();
            double a = 1.0;
            geometry_msgs::Point p_msg;
            for (const auto& traj : traj_list) {
                setMarkerColor(path_msg, color, a);
                // a = a + a_step;
                path_msg.points.clear();
                for (double t = 0; t < traj.getTotalDuration(); t += 0.01) {
                    auto p = traj.getPos(t);
                    p_msg.x = p.x();
                    p_msg.y = p.y();
                    p_msg.z = p.z();
                    path_msg.points.push_back(p_msg);
                }
                path_list_msg.markers.push_back(path_msg);
                path_msg.id += 1;
            }
            publisher_map_[topic].publish(path_list_msg);
        }

        template <class PATHLIST, class TOPIC>
        // PATHLIST: std::vector<PATH>
        void visualize_path_list(const PATHLIST& path_list, const TOPIC& topic,
                                 const Color color = steelblue, double scale = 0.1) {
            auto got = publisher_map_.find(topic);
            if (got == publisher_map_.end()) {
                ros::Publisher pub =
                    nh_.advertise<visualization_msgs::MarkerArray>(topic, 10);
                publisher_map_[topic] = pub;
                type_map_[topic] = "MarkerArray";
                ros::Duration(0.1).sleep();
            }
            visualization_msgs::Marker clear_previous_msg;
            clear_previous_msg.action = visualization_msgs::Marker::DELETEALL;
            visualization_msgs::Marker path_msg;
            path_msg.type = visualization_msgs::Marker::LINE_STRIP;
            path_msg.action = visualization_msgs::Marker::ADD;
            path_msg.header.frame_id = "world";
            path_msg.id = 0;
            setMarkerPose(path_msg, 0, 0, 0);
            setMarkerScale(path_msg, scale, scale, scale);
            visualization_msgs::MarkerArray path_list_msg;
            path_list_msg.markers.reserve(1 + path_list.size());
            path_list_msg.markers.push_back(clear_previous_msg);
            setMarkerColor(path_msg, color);
            for (const auto& path : path_list) {
                path_msg.points.resize(path.size());
                for (size_t i = 0; i < path.size(); ++i) {
                    path_msg.points[i].x = path[i].x();
                    path_msg.points[i].y = path[i].y();
                    path_msg.points[i].z = path[i].z();
                }
                path_list_msg.markers.push_back(path_msg);
                path_msg.id += 1;
            }
            publisher_map_[topic].publish(path_list_msg);
        }

        template <class TOPIC_TYPE, class TOPIC>
        void registe(const TOPIC& topic) {
            auto got = publisher_map_.find(topic);
            if (got == publisher_map_.end()) {
                ros::Publisher pub = nh_.advertise<TOPIC_TYPE>(topic, 10);
                publisher_map_[topic] = pub;
                type_map_[topic] = typeid(TOPIC_TYPE).name();
                ros::Duration(0.1).sleep();
            }
        }
    };
} // namespace visualization


#endif
