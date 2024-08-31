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

#include <rog_map/rog_map_core/counter_map.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

//#define ESDF_MAP_DEBUG

namespace rog_map {

    class ESDFMap : public CounterMap {

    public:
        typedef std::shared_ptr<ESDFMap> Ptr;
        ESDFMap() = default;
        ~ESDFMap() = default;

        void initESDFMap(
                const Vec3i &half_prob_map_size_i, /* The input is half map size, to ensure the map size is always odds*/
                const double &prob_map_resolution,
                const double &temp_counter_map_resolution,
                const Vec3f &local_update_box,
                const bool &map_sliding_en,
                const double &sliding_thresh,
                const Vec3f &fix_map_origin,
                const double &unk_thresh);




        void getUpdatedBbox(Vec3f & box_min,Vec3f & box_max)const;

        void resetLocalMap() override;

        double getDistance(const Vec3f &pos) const;

        double getDistance(const Vec3i &id_g) const;

        void updateESDF3D(const Vec3f &cur_odom);

        void resetOneCell(const int & hash_id) override;


        void evaluateEDT(const Eigen::Vector3d& pos, double& dist);
        void evaluateFirstGrad(const Eigen::Vector3d& pos, Eigen::Vector3d& grad);
        void evaluateSecondGrad(const Eigen::Vector3d& pos, Eigen::Vector3d& grad);
        void visEDTGrad(const Vec3f &box_min_d,
                        const Vec3f &box_max_d,
                        const double &visualize_z,
                        visualization_msgs::MarkerArray &mk_arr);

        /*Only for visualize */
        void getESDFOccPC2(const Vec3f &box_min_d,
                           const Vec3f &box_max_d,
                           sensor_msgs::PointCloud2 &pc2);

        void getPositiveESDFPC2(const Vec3f &box_min_d,
                                const Vec3f &box_max_d,
                                const double &visualize_z,
                                sensor_msgs::PointCloud2 &pc2);

        void getNegativeESDFPC2(const Vec3f &box_min_d,
                                const Vec3f &box_max_d,
                                const double &visualize_z,
                                sensor_msgs::PointCloud2 &pc2) ;

    private:

        template<typename F_get_val, typename F_set_val>
        void fillESDF(F_get_val f_get_val, F_set_val f_set_val,
                      const int &start, const int &end, const int &dim, const int &id_l);

        bool had_been_initialized{false};
        bool map_empty_{true};
        std::vector<double> distance_buffer;
        vector<double> tmp_buffer1_, tmp_buffer2_;
        Vec3i half_local_update_box_i_;
        Vec3i update_local_map_min_i_, update_local_map_max_i_;
        pcl::PointCloud<pcl::PointXYZI> pcl_pc;
        std::mutex update_esdf_mtx;


        void triggerJumpingEdge(const rog_map::Vec3i &id_g, const rog_map::GridType &from_type,
                                const rog_map::GridType &to_type) override {}

        // EDT Environment

        void getSurroundDistance(Eigen::Vector3d pts[2][2][2], double dists[2][2][2]);
        void getSurroundFirstGrad(Eigen::Vector3d pts[2][2][2], double first_grad[2][2][2][3]);
        void interpolateTrilinearEDT(double values[2][2][2], const Eigen::Vector3d& diff, double& value);
        void interpolateTrilinearFirstGrad(double values[2][2][2], const Eigen::Vector3d& diff, Eigen::Vector3d& grad);
        void interpolateTrilinearSecondGrad(double first_grad[2][2][2][3], const Eigen::Vector3d& diff, Eigen::Vector3d& grad);
        void getSurroundPts(const Vec3f& pos, Vec3f pts[2][2][2], Vec3f & diff);


    };

}

