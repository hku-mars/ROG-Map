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

#include <utils/common_lib.hpp>

namespace rog_map {
    using std::string;
    using std::vector;
#define RM_UNKNOWN_FLAG (-99999)
    typedef pcl::PointXYZINormal PclPoint;
    typedef pcl::PointCloud<PclPoint> PointCloud;


    class Config {
    private:
        template<class T>
        bool LoadParam(string param_name, T &param_value, T default_value = T{}, bool required = false) {
            if (nh_.getParam(param_name, param_value)) {
                printf("\033[0;32m Load param %s succes: \033[0;0m", (nh_.getNamespace() + "/" + param_name).c_str());
                std::cout << param_value << std::endl;
                return true;
            } else {
                printf("\033[0;33m Load param %s failed, use default value: \033[0;0m",
                       (nh_.getNamespace() + "/" + param_name).c_str());
                param_value = default_value;
                std::cout << param_value << std::endl;
                if (required) {
                    throw std::invalid_argument(
                            string("Required param " + (nh_.getNamespace() + "/" + param_name) + " not found"));
                }
                return false;
            }
        }

        template<class T>
        bool LoadParam(string param_name, vector<T> &param_value, vector<T> default_value = vector<T>{},
                       bool required = false) {
            if (nh_.getParam(param_name, param_value)) {
                printf("\033[0;32m Load param %s succes: \033[0;0m", (nh_.getNamespace() + "/" + param_name).c_str());
                for (size_t i = 0; i < param_value.size(); i++) {
                    std::cout << param_value[i] << " ";
                }
                std::cout << std::endl;
                return true;
            } else {
                printf("\033[0;33m Load param %s failed, use default value: \033[0;0m",
                       (nh_.getNamespace() + "/" + param_name).c_str());
                param_value = default_value;
                for (size_t i = 0; i < param_value.size(); i++) {
                    std::cout << param_value[i] << " ";
                }
                std::cout << std::endl;
                if (required) {
                    throw std::invalid_argument(
                            string("Required param " + (nh_.getNamespace() + "/" + param_name) + " not found"));
                }
                return false;
            }
        }

    public:
        Config(){};

        Config(const ros::NodeHandle &nh,
               const string &name_space = "rog_map") : nh_(nh) {
            std::cout<<" -- [ROG Config] Current namespace: "<< nh_.getNamespace()<<std::endl;
            LoadParam(name_space + "/esdf/resolution", esdf_resolution, 0.2);
            LoadParam(name_space + "/esdf/enable", esdf_en, false);
            vector<double> temp_esdf_update_box;
            LoadParam(name_space + "/esdf/local_update_box", temp_esdf_update_box, temp_esdf_update_box);

            if (esdf_en) {
                if (temp_esdf_update_box.size() != 3) {
                    ROS_ERROR("Fix map origin size is not 3!");
                    exit(-1);
                } else {
                    esdf_local_update_box = Vec3f(temp_esdf_update_box[0], temp_esdf_update_box[1],
                                                  temp_esdf_update_box[2]);
                }
            }


            LoadParam(name_space + "/load_pcd_en", load_pcd_en, false);
            if (load_pcd_en) {
                LoadParam(name_space + "/pcd_name", pcd_name, string("map.pcd"));
            }

            LoadParam(name_space + "/map_sliding/enable", map_sliding_en, true);
            LoadParam(name_space + "/map_sliding/threshold", map_sliding_thresh, -1.0);

            vector<double> temp_fix_origin;
            LoadParam(name_space + "/fix_map_origin", temp_fix_origin, vector<double>{0, 0, 0});
            if (temp_fix_origin.size() != 3) {
                ROS_ERROR("Fix map origin size is not 3!");
                exit(-1);
            } else {
                fix_map_origin = Vec3f(temp_fix_origin[0], temp_fix_origin[1], temp_fix_origin[2]);
            }

            LoadParam(name_space + "/frontier_extraction_en", frontier_extraction_en, false);

            LoadParam(name_space + "/ros_callback/enable", ros_callback_en, false);
            LoadParam(name_space + "/ros_callback/cloud_topic", cloud_topic, string("/cloud_registered"));
            LoadParam(name_space + "/ros_callback/odom_topic", odom_topic, string("/lidar_slam/odom"));
            LoadParam(name_space + "/ros_callback/odom_timeout", odom_timeout, 0.05);


            LoadParam(name_space + "/visualization/enable", visualization_en, false);
            LoadParam(name_space + "/visualization/use_dynamic_reconfigure", use_dynamic_reconfigure, false);
            LoadParam(name_space + "/visualization/pub_unknown_map_en", pub_unknown_map_en, false);
            LoadParam(name_space + "/visualization/frame_id", frame_id, string("world"));
            LoadParam(name_space + "/visualization/time_rate", viz_time_rate, 0.0);
            LoadParam(name_space + "/visualization/frame_rate", viz_frame_rate, 0);
            vector<double> temp_vis_range;
            LoadParam(name_space + "/visualization/range", temp_vis_range, vector<double>{0, 0, 0});
            if (temp_vis_range.size() != 3) {
                ROS_ERROR("Visualization range size is not 3!");
                exit(-1);
            } else {
                visualization_range = Vec3f(temp_vis_range[0], temp_vis_range[1], temp_vis_range[2]);
                if (visualization_range.minCoeff() <= 0) {
                    std::cout << YELLOW << " -- [ROG] Visualization range is not set, visualization is disabled"
                              << RESET << std::endl;
                    visualization_en = false;
                }
            }


            LoadParam(name_space + "/resolution", resolution, 0.1);
            LoadParam(name_space + "/inflation_resolution", inflation_resolution, 0.1);
            /// Resize the map to ease indexing
            if (resolution > inflation_resolution) {
                ROS_ERROR("The inflation resolution should be equal or larger than the resolution!");
                exit(-1);
            }
//    int scale = floor(inflation_resolution / resolution / 2);
//    inflation_resolution = resolution * (scale * 2 + 1);
//    ROS_ERROR("The inflation resolution is set to %f", inflation_resolution);


            /* For unk inflation */
            LoadParam(name_space + "/unk_inflation_en", unk_inflation_en, false);
            LoadParam(name_space + "/unk_inflation_step", unk_inflation_step, 1);

            LoadParam(name_space + "/inflation_step", inflation_step, 1);
            LoadParam(name_space + "/intensity_thresh", intensity_thresh, -1);

            vector<double> temp_map_size;
            LoadParam(name_space + "/map_size", temp_map_size, vector<double>{10, 10, 0});
            if (temp_map_size.size() != 3) {
                ROS_ERROR("Map size dimension is not 3!");
                exit(-1);
            }
            map_size_d = Vec3f(temp_map_size[0], temp_map_size[1], temp_map_size[2]);


            LoadParam(name_space + "/point_filt_num", point_filt_num, 2);
            if (point_filt_num <= 0) {
                std::cout << RED << " -- [ROG] point_filt_num should be larger or equal than 1, it is set to 1 now."
                          << RESET << std::endl;
                point_filt_num = 1;
            }


            // raycasting
            LoadParam(name_space + "/raycasting/enable", raycasting_en, true);
            LoadParam(name_space + "/raycasting/batch_update_size", batch_update_size, 1);
            if (batch_update_size <= 0) {
                std::cout << RED << " -- [ROG] batch_update_size should be larger or equal than 1, it is set to 1 now."
                          << RESET << std::endl;
                batch_update_size = 1;
            }
            LoadParam(name_space + "/raycasting/unk_thresh", unk_thresh, 0.70);
            LoadParam(name_space + "/raycasting/p_hit", p_hit, 0.70f);
            LoadParam(name_space + "/raycasting/p_miss", p_miss, 0.70f);
            LoadParam(name_space + "/raycasting/p_min", p_min, 0.12f);
            LoadParam(name_space + "/raycasting/p_max", p_max, 0.97f);
            LoadParam(name_space + "/raycasting/p_occ", p_occ, 0.80f);
            LoadParam(name_space + "/raycasting/p_free", p_free, 0.30f);
            LoadParam(name_space + "/raycasting/p_free", p_free, 0.30f);

            vector<double> temp_ray_range;
            LoadParam(name_space + "/raycasting/ray_range", temp_ray_range, vector<double>{0.3, 10});
            if (temp_ray_range.size() != 2) {
                ROS_ERROR("Ray range size is not 2!");
                exit(-1);
            }
            raycast_range_min = temp_ray_range[0];
            raycast_range_max = temp_ray_range[1];
            sqr_raycast_range_max = raycast_range_max * raycast_range_max;
            sqr_raycast_range_min = raycast_range_min * raycast_range_min;
            vector<double> update_box;
            LoadParam(name_space + "/raycasting/local_update_box", update_box, vector<double>{999, 999, 999});
            if (update_box.size() != 3) {
                ROS_ERROR("Update box size is not 3!");
                exit(-1);
            }
            local_update_box_d = Vec3f(update_box[0], update_box[1], update_box[2]);


            LoadParam(name_space + "/virtual_ground_height", virtual_ground_height, -0.1);
            LoadParam(name_space + "/virtual_ceil_height", virtual_ceil_height, -0.1);

            resetMapSize();

            /// Probabilistic Update
#define logit(x) (log((x) / (1 - (x))))
            l_hit = logit(p_hit);
            l_miss = logit(p_miss);
            l_min = logit(p_min);
            l_max = logit(p_max);
            l_occ = logit(p_occ);
            l_free = logit(p_free);

            int n_free = ceil(l_free / l_miss);
            int n_occ = ceil(l_occ / l_hit);

            std::cout << BLUE << "\t[ROG] n_free: " << n_free << RESET << std::endl;
            std::cout << BLUE << "\t[ROG] n_occ: " << n_occ << RESET << std::endl;

            std::cout << BLUE << "\t[ROG] l_hit: " << l_hit << RESET << std::endl;
            std::cout << BLUE << "\t[ROG] l_miss: " << l_miss << RESET << std::endl;
            std::cout << BLUE << "\t[ROG] l_min: " << l_min << RESET << std::endl;
            std::cout << BLUE << "\t[ROG] l_max: " << l_max << RESET << std::endl;
            std::cout << BLUE << "\t[ROG] l_occ: " << l_occ << RESET << std::endl;
            std::cout << BLUE << "\t[ROG] l_free: " << l_free << RESET << std::endl;

            // init spherical neighbor
            for (int dx = -inflation_step; dx <= inflation_step; dx++) {
                for (int dy = -inflation_step; dy <= inflation_step; dy++) {
                    for (int dz = -inflation_step; dz <= inflation_step; dz++) {
                        if (inflation_step == 1 ||
                            dx * dx + dy * dy + dz * dz <= inflation_step * inflation_step) {
                            spherical_neighbor.emplace_back(dx, dy, dz);
                        }
                    }
                }
            }
            std::sort(spherical_neighbor.begin(), spherical_neighbor.end(), [](const Vec3i &a, const Vec3i &b) {
                return a.norm() < b.norm();
            });

            if (unk_inflation_en) {
                unk_spherical_neighbor.clear();
                // init spherical neighbor
                for (int dx = -unk_inflation_step; dx <= unk_inflation_step; dx++) {
                    for (int dy = -unk_inflation_step; dy <= unk_inflation_step; dy++) {
                        for (int dz = -unk_inflation_step; dz <= unk_inflation_step; dz++) {
                            if (unk_inflation_step == 1 ||
                                dx * dx + dy * dy + dz * dz <= unk_inflation_step * unk_inflation_step) {
                                unk_spherical_neighbor.emplace_back(dx, dy, dz);
                            }
                        }
                    }
                }
                std::sort(unk_spherical_neighbor.begin(), unk_spherical_neighbor.end(),
                          [](const Vec3i &a, const Vec3i &b) {
                              return a.norm() < b.norm();
                          });
            }
        }

        // add 24.07.18 add esdf
        bool esdf_en{false};
        Vec3f esdf_local_update_box;
        double esdf_resolution;

        bool load_pcd_en{false};
        bool use_dynamic_reconfigure{false};
        string pcd_name{"map.pcd"};

        double resolution, inflation_resolution;
        int inflation_step;
        Vec3f local_update_box_d, half_local_update_box_d;
        Vec3i local_update_box_i, half_local_update_box_i;
        Vec3f map_size_d, half_map_size_d;
        Vec3i inf_half_map_size_i, half_map_size_i, fro_half_map_size_i;
        double virtual_ceil_height, virtual_ground_height;
        int virtual_ceil_height_id_g, virtual_ground_height_id_g;

        bool visualization_en{false}, frontier_extraction_en{false},
                raycasting_en{true}, ros_callback_en{false}, pub_unknown_map_en{false};

        std::vector<Vec3i> spherical_neighbor;
        std::vector<Vec3i> unk_spherical_neighbor;

        /* intensity noise filter*/
        int intensity_thresh;
        /* aster properties */
        string frame_id;
        bool map_sliding_en{true};
        Vec3f fix_map_origin;
        string odom_topic, cloud_topic;
        /* probability update */
        double raycast_range_min, raycast_range_max;
        double sqr_raycast_range_min, sqr_raycast_range_max;
        int point_filt_num, batch_update_size;
        float p_hit, p_miss, p_min, p_max, p_occ, p_free;
        float l_hit, l_miss, l_min, l_max, l_occ, l_free;

        /* for unknown inflation */
        bool unk_inflation_en{false};
        int unk_inflation_step{0};


        double odom_timeout;
        Vec3f visualization_range;
        double viz_time_rate;
        int viz_frame_rate;

        double unk_thresh;
        double map_sliding_thresh;

        void resetMapSize() {

            int inflation_ratio = ceil(inflation_resolution / resolution);

#ifdef ORIGIN_AT_CENTER
            // When discretized in such a way that the origin is at cell center
    // the inflation ratio should be an odd number
    if (inflation_ratio % 2 == 0) {
        inflation_ratio += 1;
    }
    std::cout << RED << " -- [RM-Config] inflation_ratio: " << inflation_ratio << std::endl;
#endif

            inflation_resolution = resolution * inflation_ratio;
            std::cout << GREEN << " -- [RM-Config] inflation_resolution: " << inflation_resolution << std::endl;

            half_map_size_d = map_size_d / 2;

            // Size_d is only for calculate index number.
            // 1) we calculate the index number of the inf map
            int max_step = 0;
            if (!unk_inflation_en) {
                max_step = inflation_step;
            } else {
                max_step = std::max(inflation_step, unk_inflation_step);
            }
            inf_half_map_size_i = (half_map_size_d / inflation_resolution).cast<int>()
                                  + (max_step + 1) * Vec3i::Ones();

            // 2) we calculate the index number of the prob map, which should be smaller than infmap
            half_map_size_i = (inf_half_map_size_i - (max_step + 1) * Vec3i::Ones()) * inflation_ratio;

            // 3) compute the frontier counter map size
            if (frontier_extraction_en) {
                fro_half_map_size_i = half_map_size_i + Vec3i::Constant(1);
            }

            // 4) re-compute the map_size_d, the map_size_d is not that important.
            map_size_d = (half_map_size_i * 2 + Vec3i::Constant(1)).cast<double>() * resolution;
            half_map_size_d = map_size_d / 2;

            // 5) compute the index of raycasting update box
            half_local_update_box_d = local_update_box_d / 2;
            half_local_update_box_i = (half_local_update_box_d / resolution).cast<int>();
            local_update_box_i = half_local_update_box_i * 2 + Vec3i::Constant(1);
            local_update_box_d = local_update_box_i.cast<double>() * resolution;
        }


        ros::NodeHandle nh_;

    };

}
