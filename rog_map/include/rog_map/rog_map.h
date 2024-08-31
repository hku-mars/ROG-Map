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

#include <rog_map/prob_map.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>
#include <utils/common_lib.hpp>
#include <dynamic_reconfigure/server.h>
#include <rog_map/VizConfig.h>
#include <utils/visual_utils.hpp>

namespace rog_map {
    using namespace std;

    typedef pcl::PointXYZINormal PointType;
    typedef pcl::PointCloud<PointType> PointCloudXYZIN;

    class ROGMap : public ProbMap {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        typedef shared_ptr<ROGMap> Ptr;

        ROGMap(const ros::NodeHandle &nh);

        ~ROGMap() = default;


        bool isLineFree(const Vec3f &start_pt, const Vec3f &end_pt,
                        const double &max_dis = 999999,
                        const vec_Vec3i &neighbor_list = vec_Vec3i{}) const;

        bool isLineFree(const Vec3f &start_pt, const Vec3f &end_pt,
                        Vec3f &free_local_goal, const double &max_dis = 999999,
                        const vec_Vec3i &neighbor_list = vec_Vec3i{}) const;

        bool isLineFree(const Vec3f &start_pt, const Vec3f &end_pt,
                        const bool & use_inf_map = false,
                        const bool & use_unk_as_occ = false) const;


        void updateMap(const PointCloud &cloud, const Pose &pose);

        RobotState getRobotState() const;

    private:
        ros::NodeHandle nh_;

        RobotState robot_state_;

        struct ROSCallback {
            ros::Subscriber odom_sub, cloud_sub;
            int unfinished_frame_cnt{0};
            Pose pc_pose;
            PointCloud pc;
            ros::Timer update_timer;
            mutex updete_lock;
        } rc_;

        struct VisualizeMap {
            ros::Publisher occ_pub, unknown_pub,
                    occ_inf_pub, unknown_inf_pub,
                    mkr_arr_pub, frontier_pub,
                    esdf_pub, esdf_neg_pub, esdf_occ_pub;
            visualization_msgs::MarkerArray mkr_arr;
            ros::Timer viz_timer;
            struct VizCfg {
                dynamic_reconfigure::Server<rog_map::VizConfig> vizcfgserver;
                dynamic_reconfigure::Server<rog_map::VizConfig>::CallbackType callback_func;
                bool use_body_center{false};
                Vec3f box_min, box_max;
            } vizcfg;
        } vm_;

        std::ofstream time_log_file_, map_info_log_file_;

        void updateRobotState(const Pose &pose);

        void odomCallback(const nav_msgs::OdometryConstPtr &odom_msg);

        void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);

        void updateCallback(const ros::TimerEvent &event);

        static void vecEVec3fToPC2(const vec_E<Vec3f> &points, sensor_msgs::PointCloud2 &cloud);

        void vizCallback(const ros::TimerEvent &event);

        void VizCfgCallback(rog_map::VizConfig &config, uint32_t level);

    };
}
