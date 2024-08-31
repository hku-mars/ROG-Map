/**
* This file is part of ROG-Map
*
* Copyright 2024 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/ROG-Map>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* ROG-Map is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ROG-Map is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with ROG-Map. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "utils/common_lib.hpp"
#include "std_msgs/ColorRGBA.h"
#include "visualization_msgs/MarkerArray.h"

namespace rog_map {
    class Color : public std_msgs::ColorRGBA {
    public:
        Color() : std_msgs::ColorRGBA() {}

        Color(int hex_color) {
            int _r = (hex_color >> 16) & 0xFF;
            int _g = (hex_color >> 8) & 0xFF;
            int _b = hex_color & 0xFF;
            r = static_cast<double>(_r) / 255.0;
            g = static_cast<double>(_g) / 255.0;
            b = static_cast<double>(_b) / 255.0;
        }

        Color(Color c, double alpha) {
            r = c.r;
            g = c.g;
            b = c.b;
            a = alpha;
        }

        Color(double red, double green, double blue) : Color(red, green, blue, 1.0) {
            r = red > 1.0 ? red / 255.0 : red;
            g = green > 1.0 ? green / 255.0 : green;
            b = blue > 1.0 ? blue / 255.0 : blue;
        }

        Color(double red, double green, double blue, double alpha) : Color() {
            r = red > 1.0 ? red / 255.0 : red;
            g = green > 1.0 ? green / 255.0 : green;
            b = blue > 1.0 ? blue / 255.0 : blue;
            a = alpha;
        }

        static const Color White() { return Color(1.0, 1.0, 1.0); }

        static const Color Black() { return Color(0.0, 0.0, 0.0); }

        static const Color Gray() { return Color(0.5, 0.5, 0.5); }

        static const Color Red() { return Color(1.0, 0.0, 0.0); }

        static const Color Green() { return Color(0.0, 0.96, 0.0); }

        static const Color Blue() { return Color(0.0, 0.0, 1.0); }

        static const Color SteelBlue() { return Color(0.4, 0.7, 1.0); }

        static const Color Yellow() { return Color(1.0, 1.0, 0.0); }

        static Color Orange() { return Color(1.0, 0.5, 0.0); }

        static const Color Purple() { return Color(0.5, 0.0, 1.0); }

        static const Color Chartreuse() { return Color(0.5, 1.0, 0.0); }

        static const Color Teal() { return Color(0.0, 1.0, 1.0); }

        static const Color Pink() { return Color(1.0, 0.0, 0.5); }
    };


/* Type A, directly publish marker in publisher */
    static void visualizeText(const ros::Publisher &pub,
                              const std::string &ns,
                              const std::string &text,
                              const Vec3f &position,
                              const Color &c,
                              const double &size,
                              const int &id = -1) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.ns = ns.c_str();
        if (id >= 0) {
            marker.id = id;
        } else {
            static int id = 0;
            marker.id = id++;
        }
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.scale.z = size;
        marker.color = c;
        marker.text = text;
        marker.pose.position.x = position.x();
        marker.pose.position.y = position.y();
        marker.pose.position.z = position.z();
        marker.pose.orientation.w = 1.0;
        visualization_msgs::MarkerArray arr;
        arr.markers.push_back(marker);
        pub.publish(arr);
    };

    static void visualizePoint(const ros::Publisher &pub_,
                               const Vec3f &pt,
                               Color color = Color::Pink(),
                               std::string ns = "pt",
                               double size = 0.1, int id = -1,
                               const bool &print_ns = true) {
        visualization_msgs::MarkerArray mkr_arr;
        visualization_msgs::Marker marker_ball;
        static int cnt = 0;
        Vec3f cur_pos = pt;
        if (isnan(pt.x()) || isnan(pt.y()) || isnan(pt.z())) {
            return;
        }
        marker_ball.header.frame_id = "world";
        marker_ball.header.stamp = ros::Time::now();
        marker_ball.ns = ns.c_str();
        marker_ball.id = id >= 0 ? id : cnt++;
        marker_ball.action = visualization_msgs::Marker::ADD;
        marker_ball.pose.orientation.w = 1.0;
        marker_ball.type = visualization_msgs::Marker::SPHERE;
        marker_ball.scale.x = size;
        marker_ball.scale.y = size;
        marker_ball.scale.z = size;
        marker_ball.color = color;

        geometry_msgs::Point p;
        p.x = cur_pos.x();
        p.y = cur_pos.y();
        p.z = cur_pos.z();

        marker_ball.pose.position = p;
        mkr_arr.markers.push_back(marker_ball);

        // add test
        if (print_ns) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = ros::Time::now();
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.ns = ns + "_text";
            if (id >= 0) {
                marker.id = id;
            } else {
                static int id = 0;
                marker.id = id++;
            }
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.scale.z = 0.6;
            marker.color = color;
            marker.text = ns;
            marker.pose.position.x = cur_pos.x();
            marker.pose.position.y = cur_pos.y();
            marker.pose.position.z = cur_pos.z() + 0.5;
            marker.pose.orientation.w = 1.0;
            mkr_arr.markers.push_back(marker);
        }

        pub_.publish(mkr_arr);
    }

    static void visualizeBoundingBox(const ros::Publisher &pub,
                                     const Vec3f &box_min,
                                     const Vec3f &box_max,
                                     const string &ns,
                                     const Color &color,
                                     const double &size_x = 0.1,
                                     const double &alpha = 1.0,
                                     const bool &print_ns = true) {
        Vec3f size = (box_max - box_min) / 2;
        Vec3f vis_pos_world = (box_min + box_max) / 2;
        double width = size.x();
        double length = size.y();
        double hight = size.z();
        visualization_msgs::MarkerArray mkrarr;
//Publish Bounding box
        int id = 0;
        visualization_msgs::Marker line_strip;
        line_strip.header.stamp = ros::Time::now();
        line_strip.header.frame_id = "world";
        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.ns = ns;
        line_strip.pose.orientation.w = 1.0;
        line_strip.id = id++; //unique id, useful when multiple markers exist.
        line_strip.type = visualization_msgs::Marker::LINE_STRIP; //marker type
        line_strip.scale.x = size_x;


        line_strip.color = color;
        line_strip.color.a = alpha; //不透明度，设0则全透明
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
        pub.publish(mkrarr);
    }

    /* Type B: Add marker to given marker_arr for later publish */
    static void visualizeText(visualization_msgs::MarkerArray &mkr_arr,
                              const std::string &ns,
                              const std::string &text,
                              const Vec3f &position,
                              const Color &c = Color::White(),
                              const double &size = 0.6,
                              const int &id = -1) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.ns = ns.c_str();
        if (id >= 0) {
            marker.id = id;
        } else {
            static int id = 0;
            marker.id = id++;
        }
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.scale.z = size;
        marker.color = c;
        marker.text = text;
        marker.pose.position.x = position.x();
        marker.pose.position.y = position.y();
        marker.pose.position.z = position.z();
        marker.pose.orientation.w = 1.0;
        mkr_arr.markers.push_back(marker);
    };

    static void visualizePoint(visualization_msgs::MarkerArray &mkr_arr,
                               const Vec3f &pt,
                               Color color = Color::Pink(),
                               std::string ns = "pt",
                               double size = 0.1, int id = -1,
                               const bool &print_ns = true) {
        visualization_msgs::Marker marker_ball;
        static int cnt = 0;
        Vec3f cur_pos = pt;
        if (isnan(pt.x()) || isnan(pt.y()) || isnan(pt.z())) {
            return;
        }
        marker_ball.header.frame_id = "world";
        marker_ball.header.stamp = ros::Time::now();
        marker_ball.ns = ns.c_str();
        marker_ball.id = id >= 0 ? id : cnt++;
        marker_ball.action = visualization_msgs::Marker::ADD;
        marker_ball.pose.orientation.w = 1.0;
        marker_ball.type = visualization_msgs::Marker::SPHERE;
        marker_ball.scale.x = size;
        marker_ball.scale.y = size;
        marker_ball.scale.z = size;
        marker_ball.color = color;

        geometry_msgs::Point p;
        p.x = cur_pos.x();
        p.y = cur_pos.y();
        p.z = cur_pos.z();

        marker_ball.pose.position = p;
        mkr_arr.markers.push_back(marker_ball);

        // add test
        if (print_ns) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = ros::Time::now();
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.ns = ns + "_text";
            if (id >= 0) {
                marker.id = id;
            } else {
                static int id = 0;
                marker.id = id++;
            }
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.scale.z = 0.6;
            marker.color = color;
            marker.text = ns;
            marker.pose.position.x = cur_pos.x();
            marker.pose.position.y = cur_pos.y();
            marker.pose.position.z = cur_pos.z() + 0.5;
            marker.pose.orientation.w = 1.0;
            mkr_arr.markers.push_back(marker);
        }
    }

    static void visualizeBoundingBox(visualization_msgs::MarkerArray &mkrarr,
                                     const Vec3f &box_min,
                                     const Vec3f &box_max,
                                     const string &ns,
                                     const Color &color,
                                     const double &size_x = 0.1,
                                     const double &alpha = 1.0,
                                     const bool &print_ns = true) {
        Vec3f size = (box_max - box_min) / 2;
        Vec3f vis_pos_world = (box_min + box_max) / 2;
        double width = size.x();
        double length = size.y();
        double hight = size.z();

//Publish Bounding box
        int id = 0;
        visualization_msgs::Marker line_strip;
        line_strip.header.stamp = ros::Time::now();
        line_strip.header.frame_id = "world";
        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.ns = ns;
        line_strip.pose.orientation.w = 1.0;
        line_strip.id = id++; //unique id, useful when multiple markers exist.
        line_strip.type = visualization_msgs::Marker::LINE_STRIP; //marker type
        line_strip.scale.x = size_x;


        line_strip.color = color;
        line_strip.color.a = alpha; //不透明度，设0则全透明
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
    }

}

