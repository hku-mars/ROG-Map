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

#include "rog_map/esdf_map.h"

namespace rog_map {

    void ESDFMap::initESDFMap(const rog_map::Vec3i &half_prob_map_size_i, const double &prob_map_resolution,
                              const double &temp_counter_map_resolution, const rog_map::Vec3f &local_update_box,
                              const bool &map_sliding_en, const double &sliding_thresh,
                              const rog_map::Vec3f &fix_map_origin, const double &unk_thresh) {

        if (had_been_initialized) {
            throw std::runtime_error(" -- [ESDFMap]: init can only be called once!");
        }
        had_been_initialized = true;
        initCounterMap(half_prob_map_size_i,
                       prob_map_resolution,
                       temp_counter_map_resolution,
                       0,
                       map_sliding_en,
                       sliding_thresh,
                       fix_map_origin,
                       unk_thresh);

        distance_buffer.resize(sc_.map_vox_num);
        tmp_buffer1_.resize(sc_.map_vox_num);
        tmp_buffer2_.resize(sc_.map_vox_num);
        posToGlobalIndex(local_update_box, half_local_update_box_i_);
        half_local_update_box_i_ /= 2;

        resetLocalMap();
        std::cout << GREEN << " -- [ESDFMap] Init successfully -- ." << RESET << std::endl;
        printMapInformation();
    }

    void ESDFMap::getUpdatedBbox(rog_map::Vec3f &box_min, rog_map::Vec3f &box_max) const {
        globalIndexToPos(update_local_map_min_i_, box_min);
        globalIndexToPos(update_local_map_max_i_, box_max);
    }


    void ESDFMap::resetLocalMap() {
        std::cout << RED << " -- [ESDFMap] Clear all local map." << RESET << std::endl;
        std::fill(md_.unknown_cnt.begin(), md_.unknown_cnt.end(), md_.sub_grid_num);
        std::fill(md_.occupied_cnt.begin(), md_.occupied_cnt.end(), 0);
    }

    double ESDFMap::getDistance(const rog_map::Vec3f &pos) const {
        return distance_buffer[getHashIndexFromPos(pos)];
    }

    double ESDFMap::getDistance(const Vec3i &id_g) const {
        return distance_buffer[getHashIndexFromGlobalIndex(id_g)];
    }

    void ESDFMap::updateESDF3D(const rog_map::Vec3f &cur_odom) {
        std::lock_guard<std::mutex> lck(update_esdf_mtx);
        using namespace std;
        TimeConsuming up_t("updateESDF3D", false);
        Vec3i id_l, cur_odom_i;
        posToGlobalIndex(cur_odom, cur_odom_i);
        globalIndexToLocalIndex(local_map_bound_min_i_, id_l);
        id_l = id_l + sc_.half_map_size_i;

        Vec3i mem_end = sc_.map_size_i - Vec3i::Ones() - id_l;

        auto getEsdfLocalIndexHash = [&](const int &id_x,
                                         const int &id_y,
                                         const int &id_z, bool print = false) {
            return id_x * sc_.map_size_i(1) * sc_.map_size_i(2) + id_y * sc_.map_size_i(2) + id_z;
        };

        Vec3i min_esdf = cur_odom_i - half_local_update_box_i_;
        Vec3i max_esdf = cur_odom_i + half_local_update_box_i_;
        update_local_map_min_i_ = min_esdf.cwiseMax(local_map_bound_min_i_);
        update_local_map_max_i_ = max_esdf.cwiseMin(local_map_bound_max_i_) -
                                  Vec3i::Ones();
        min_esdf = update_local_map_min_i_ - local_map_bound_min_i_;
        max_esdf = update_local_map_max_i_ - local_map_bound_min_i_;

        for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
            for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
                fillESDF(
                        [&](int z) {
                            return isOccupied(getEsdfLocalIndexHash(
                                    x > mem_end[0] ? x + id_l[0] - sc_.map_size_i[0] : x + id_l[0],
                                    y > mem_end[1] ? y + id_l[1] - sc_.map_size_i[1] : y + id_l[1],
                                    z)) ?
                                   0 :
                                   std::numeric_limits<double>::max();
                        },
                        [&](int z, double val) {
                            tmp_buffer1_[getEsdfLocalIndexHash(
                                    x > mem_end[0] ? x + id_l[0] - sc_.map_size_i[0] : x + id_l[0],
                                    y > mem_end[1] ? y + id_l[1] - sc_.map_size_i[1] : y + id_l[1], z)] = val;
                        },
                        min_esdf[2], max_esdf[2], 2, id_l[2]);
            }
        }

        for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
            for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
                fillESDF([&](int y) {
                             return tmp_buffer1_[getEsdfLocalIndexHash(
                                     x > mem_end[0] ? x + id_l[0] - sc_.map_size_i[0] : x + id_l[0],
                                     y,
                                     z > mem_end[2] ? z + id_l[2] - sc_.map_size_i[2] : z + id_l[2])];
                         },
                         [&](int y, double val) {
                             tmp_buffer2_[getEsdfLocalIndexHash(
                                     x > mem_end[0] ? x + id_l[0] - sc_.map_size_i[0] : x + id_l[0],
                                     y,
                                     z > mem_end[2] ? z + id_l[2] - sc_.map_size_i[2] : z + id_l[2])] = val;
                         },
                         min_esdf[1],
                         max_esdf[1], 1, id_l[1]);
            }
        }

        for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
            for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
                fillESDF([&](int x) {
                             return tmp_buffer2_[getEsdfLocalIndexHash(
                                     x,
                                     y > mem_end[1] ? y + id_l[1] - sc_.map_size_i[1] : y + id_l[1],
                                     z > mem_end[2] ? z + id_l[2] - sc_.map_size_i[2] : z + id_l[2]
                             )];
                         },
                         [&](int x, double val) {
                             distance_buffer[getEsdfLocalIndexHash(
                                     x,
                                     y > mem_end[1] ? y + id_l[1] - sc_.map_size_i[1] : y + id_l[1],
                                     z > mem_end[2] ? z + id_l[2] - sc_.map_size_i[2] : z + id_l[2]
                             )] =
                                     sc_.resolution * std::sqrt(val);
                         },
                         min_esdf[0], max_esdf[0], 0, id_l[0]);
            }
        }

        for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
            for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
                fillESDF(
                        [&](int z) {
                            return isOccupied(getEsdfLocalIndexHash(
                                    x > mem_end[0] ? x + id_l[0] - sc_.map_size_i[0] : x + id_l[0],
                                    y > mem_end[1] ? y + id_l[1] - sc_.map_size_i[1] : y + id_l[1],
                                    z)) ?
                                   std::numeric_limits<double>::max() : 0;
                        },
                        [&](int z, double val) {
                            tmp_buffer1_[getEsdfLocalIndexHash(
                                    x > mem_end[0] ? x + id_l[0] - sc_.map_size_i[0] : x + id_l[0],
                                    y > mem_end[1] ? y + id_l[1] - sc_.map_size_i[1] : y + id_l[1], z)] = val;
                        },
                        min_esdf[2], max_esdf[2], 2, id_l[2]);
            }
        }

        for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
            for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
                fillESDF([&](int y) {
                             return tmp_buffer1_[getEsdfLocalIndexHash(
                                     x > mem_end[0] ? x + id_l[0] - sc_.map_size_i[0] : x + id_l[0],
                                     y,
                                     z > mem_end[2] ? z + id_l[2] - sc_.map_size_i[2] : z + id_l[2])];
                         },
                         [&](int y, double val) {
                             tmp_buffer2_[getEsdfLocalIndexHash(
                                     x > mem_end[0] ? x + id_l[0] - sc_.map_size_i[0] : x + id_l[0],
                                     y,
                                     z > mem_end[2] ? z + id_l[2] - sc_.map_size_i[2] : z + id_l[2])] = val;
                         },
                         min_esdf[1],
                         max_esdf[1], 1, id_l[1]);
            }
        }

        for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
            for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
                fillESDF([&](int x) {
                             return tmp_buffer2_[getEsdfLocalIndexHash(
                                     x,
                                     y > mem_end[1] ? y + id_l[1] - sc_.map_size_i[1] : y + id_l[1],
                                     z > mem_end[2] ? z + id_l[2] - sc_.map_size_i[2] : z + id_l[2]
                             )];
                         },
                         [&](int x, double val) {
                             tmp_buffer1_[getEsdfLocalIndexHash(
                                     x,
                                     y > mem_end[1] ? y + id_l[1] - sc_.map_size_i[1] : y + id_l[1],
                                     z > mem_end[2] ? z + id_l[2] - sc_.map_size_i[2] : z + id_l[2]
                             )] =
                                     sc_.resolution * std::sqrt(val);
                         },
                         min_esdf[0], max_esdf[0], 0, id_l[0]);
            }
        }


        /* ========== combine pos and neg DT ========== */
        for (int x = min_esdf(0); x <= max_esdf(0); ++x)
            for (int y = min_esdf(1); y <= max_esdf(1); ++y)
                for (int z = min_esdf(2); z <= max_esdf(2); ++z) {

                    const int idx = getEsdfLocalIndexHash(x, y, z);

                    if (tmp_buffer1_[idx] > 0.0) {
                        distance_buffer[idx] += (-tmp_buffer1_[idx] + sc_.resolution);
                    }

                }

        double dt = up_t.stop();
        if(dt > 0.1) {
            std::cout << "updateESDF3D time: " << dt << std::endl;
        }
#ifdef ESDF_MAP_DEBUG
        static int cnt = 0;
        static double t = 0.0;
        t += up_t.stop();
        cnt++;
        std::cout << "Average time per point: " << t / cnt / sc_.map_vox_num * 1e6 << "us" << std::endl;
#endif
    }

    void ESDFMap::getESDFOccPC2(const rog_map::Vec3f &box_min_d, const rog_map::Vec3f &box_max_d,
                                sensor_msgs::PointCloud2 &pc2) {
        std::lock_guard<std::mutex> lck(update_esdf_mtx);
        pcl_pc.clear();
        Vec3i box_min_i, box_max_i;
        posToGlobalIndex(box_min_d, box_min_i);
        posToGlobalIndex(box_max_d, box_max_i);
//            box_min_i = box_min_i.cwiseMax(update_local_map_min_i_);
//            box_max_i = box_max_i.cwiseMin(update_local_map_max_i_);


        for (int x = box_min_i.x(); x <= box_max_i.x(); x++) {
            for (int y = box_min_i.y(); y <= box_max_i.y(); y++) {
                for (int z = box_min_i.z(); z <= box_max_i.z(); z++) {
                    Vec3i id_g(x, y, z);
                    if (isOccupied(id_g)) {
                        Vec3f pos;
                        globalIndexToPos(id_g, pos);
                        pcl::PointXYZI pt;
                        pt.x = pos(0);
                        pt.y = pos(1);
                        pt.z = pos(2);
                        pcl_pc.push_back(pt);
                    }
                }
            }
        }

        pcl_pc.width = pcl_pc.points.size();
        pcl_pc.height = 1;
        pcl_pc.is_dense = true;
        pcl::toROSMsg(pcl_pc, pc2);
        pc2.header.frame_id = "world";
    }

    void ESDFMap::resetOneCell(const int &hash_id) {
    }


    void ESDFMap::getPositiveESDFPC2(const rog_map::Vec3f &box_min_d, const rog_map::Vec3f &box_max_d,
                                     const double &visualize_z, sensor_msgs::PointCloud2 &pc2) {
        std::lock_guard<std::mutex> lck(update_esdf_mtx);
        pcl_pc.clear();
        Vec3i box_min_i, box_max_i;
        posToGlobalIndex(box_min_d, box_min_i);
        posToGlobalIndex(box_max_d, box_max_i);
        box_min_i = box_min_i.cwiseMax(update_local_map_min_i_);
        box_max_i = box_max_i.cwiseMin(update_local_map_max_i_);

        int z;
        posToGlobalIndex(visualize_z, z);

        for (int x = box_min_i.x(); x <= box_max_i.x(); x++) {
            for (int y = box_min_i.y(); y <= box_max_i.y(); y++) {
                Vec3i id_g(x, y, z);
                double dist = getDistance(id_g);
                Vec3f pos;
                globalIndexToPos(id_g, pos);
                pcl::PointXYZI pt;
                pt.x = pos(0);
                pt.y = pos(1);
                pt.z = pos(2);
                pt.intensity = dist + 10.0;
                pcl_pc.push_back(pt);
            }
        }
        pcl_pc.width = pcl_pc.points.size();
        pcl_pc.height = 1;
        pcl_pc.is_dense = true;
        pcl::toROSMsg(pcl_pc, pc2);
        pc2.header.frame_id = "world";
    }


    void ESDFMap::getNegativeESDFPC2(const rog_map::Vec3f &box_min_d, const rog_map::Vec3f &box_max_d,
                                     const double &visualize_z, sensor_msgs::PointCloud2 &pc2) {
        std::lock_guard<std::mutex> lck(update_esdf_mtx);
        pcl_pc.clear();
        Vec3i box_min_i, box_max_i;
        posToGlobalIndex(box_min_d, box_min_i);
        posToGlobalIndex(box_max_d, box_max_i);
        box_min_i = box_min_i.cwiseMax(update_local_map_min_i_);
        box_max_i = box_max_i.cwiseMin(update_local_map_max_i_);

        int z;
        posToGlobalIndex(visualize_z, z);

        for (int x = box_min_i.x(); x <= box_max_i.x(); x++) {
            for (int y = box_min_i.y(); y <= box_max_i.y(); y++) {
                Vec3i id_g(x, y, z);
                double dist = -getDistance(id_g);
                Vec3f pos;
                globalIndexToPos(id_g, pos);
                pcl::PointXYZI pt;
                pt.x = pos(0);
                pt.y = pos(1);
                pt.z = pos(2);
                pt.intensity = dist < 0.0 ? 10.0 : dist;
                pcl_pc.push_back(pt);
            }
        }
        pcl_pc.width = pcl_pc.points.size();
        pcl_pc.height = 1;
        pcl_pc.is_dense = true;
        pcl::toROSMsg(pcl_pc, pc2);
        pc2.header.frame_id = "world";
    }

    template<typename F_get_val, typename F_set_val>
    void ESDFMap::fillESDF(F_get_val f_get_val, F_set_val f_set_val, const int &start, const int &end, const int &dim,
                           const int &id_l) {
        if (end < 0)
            return;
        const auto &map_size = sc_.map_size_i(dim);
        // dingdian
        int v[map_size];
        // jiaodian
        double z[map_size + 1];

        const int mem_end = (map_size - 1) - id_l;

        int k = start;
        v[start] = start;
        z[start] = -std::numeric_limits<double>::max();
        z[start + 1] = std::numeric_limits<double>::max();

        for (int q = start + 1; q <= end; q++) {
            k++;
            double s;

            do {
                k--;
                int a, b;
                a = q + id_l;
                b = v[k] + id_l;
                if (q > mem_end) {
                    a -= map_size;
                }
                if (v[k] > mem_end) {
                    b -= map_size;
                }
                s = ((f_get_val(a) + q * q) - (f_get_val(b) + v[k] * v[k])) / (2 * q - 2 * v[k]);
            } while (s <= z[k]);

            k++;

            v[k] = q;
            z[k] = s;
            z[k + 1] = std::numeric_limits<double>::max();
        }

        k = start;
        for (int q = start; q <= end; q++) {
            while (z[k + 1] < q)
                k++;
            int a = q + id_l;
            int b = v[k] + id_l;
            if (q > mem_end) {
                a -= map_size;
            }
            if (v[k] > mem_end) {
                b -= map_size;
            }
            double val = (q - v[k]) * (q - v[k]) + f_get_val(b);
            f_set_val(a, val);
        }
    }

    // EDT Environment
    void ESDFMap::getSurroundPts(const Vec3f& pos, Vec3f pts[2][2][2], Vec3f & diff){
        /* interpolation position */
        Vec3f pos_m = pos - 0.5 * sc_.resolution * Vec3f::Ones();

        Vec3i idx;
        Vec3f idx_pos;

        posToGlobalIndex(pos_m, idx);
        globalIndexToPos(idx, idx_pos);
        diff = (pos - idx_pos) / sc_.resolution;

        for (int x = 0; x < 2; x++) {
            for (int y = 0; y < 2; y++) {
                for (int z = 0; z < 2; z++) {
                    Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
                    Vec3f current_pos;
                    globalIndexToPos(current_idx, current_pos);
                    pts[x][y][z] = current_pos;
                }
            }
        }
    }

    void ESDFMap::getSurroundDistance(Eigen::Vector3d pts[2][2][2], double dists[2][2][2]) {
        for (int x = 0; x < 2; x++) {
            for (int y = 0; y < 2; y++) {
                for (int z = 0; z < 2; z++) {
                    dists[x][y][z] = getDistance(pts[x][y][z]);
                }
            }
        }
    }

    void ESDFMap::getSurroundFirstGrad(Eigen::Vector3d pts[2][2][2], double first_grad[2][2][2][3]){
        for (int x = 0; x < 2; x++) {
            for (int y = 0; y < 2; y++) {
                for (int z = 0; z < 2; z++) {
                    Eigen::Vector3d grad;
                    evaluateFirstGrad(pts[x][y][z], grad);
                    first_grad[x][y][z][0] = grad(0);
                    first_grad[x][y][z][1] = grad(1);
                    first_grad[x][y][z][2] = grad(2);
                }
            }
        }
    }

    void ESDFMap::evaluateEDT(const Eigen::Vector3d& pos,
                                     double& dist) {
        Eigen::Vector3d diff;
        Eigen::Vector3d sur_pts[2][2][2];
        getSurroundPts(pos, sur_pts, diff);

        double dists[2][2][2];
        getSurroundDistance(sur_pts, dists);

        interpolateTrilinearEDT(dists, diff, dist);
    }

    void ESDFMap::evaluateFirstGrad(const Eigen::Vector3d& pos,
                                           Eigen::Vector3d& grad) {
        Eigen::Vector3d diff;
        Eigen::Vector3d sur_pts[2][2][2];
        getSurroundPts(pos, sur_pts, diff);

        double dists[2][2][2];
        getSurroundDistance(sur_pts, dists);

        interpolateTrilinearFirstGrad(dists, diff, grad);
    }

    void ESDFMap::evaluateSecondGrad(const Eigen::Vector3d& pos,
                                            Eigen::Vector3d& grad) {
        Eigen::Vector3d diff;
        Eigen::Vector3d sur_pts[2][2][2];
        getSurroundPts(pos, sur_pts, diff);

        double first_grad[2][2][2][3];
        getSurroundFirstGrad(sur_pts, first_grad);

        interpolateTrilinearSecondGrad(first_grad, diff, grad);
    }

    void ESDFMap::interpolateTrilinearEDT(double values[2][2][2],
                                                 const Eigen::Vector3d& diff,
                                                 double& value){
        // trilinear interpolation
        double v00 = (1 - diff(0)) * values[0][0][0] + diff(0) * values[1][0][0]; // b
        double v01 = (1 - diff(0)) * values[0][0][1] + diff(0) * values[1][0][1]; // d
        double v10 = (1 - diff(0)) * values[0][1][0] + diff(0) * values[1][1][0]; // a
        double v11 = (1 - diff(0)) * values[0][1][1] + diff(0) * values[1][1][1]; // c
        double v0 = (1 - diff(1)) * v00 + diff(1) * v10;  // e
        double v1 = (1 - diff(1)) * v01 + diff(1) * v11;  // f

        value = (1 - diff(2)) * v0 + diff(2) * v1;
    }

    void ESDFMap::interpolateTrilinearFirstGrad(double values[2][2][2],
                                                       const Eigen::Vector3d& diff,
                                                       Eigen::Vector3d& grad) {
        // trilinear interpolation
        double v00 = (1 - diff(0)) * values[0][0][0] + diff(0) * values[1][0][0]; // b
        double v01 = (1 - diff(0)) * values[0][0][1] + diff(0) * values[1][0][1]; // d
        double v10 = (1 - diff(0)) * values[0][1][0] + diff(0) * values[1][1][0]; // a
        double v11 = (1 - diff(0)) * values[0][1][1] + diff(0) * values[1][1][1]; // c
        double v0 = (1 - diff(1)) * v00 + diff(1) * v10;  // e
        double v1 = (1 - diff(1)) * v01 + diff(1) * v11;  // f

        grad[2] = (v1 - v0) * sc_.resolution_inv;
        grad[1] = ((1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01)) * sc_.resolution_inv;
        grad[0] = (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]);
        grad[0] += (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]);
        grad[0] += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]);
        grad[0] += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]);
        grad[0] *= sc_.resolution_inv;
    }

    void ESDFMap::interpolateTrilinearSecondGrad(double first_grad[2][2][2][3],
                                                        const Eigen::Vector3d& diff,
                                                        Eigen::Vector3d& grad) {
        grad[0] =  (1 - diff[1]) * ( (1 - diff[2]) * (first_grad[1][0][0][0] - first_grad[0][0][0][0]) + diff[2] * (first_grad[1][0][1][0] - first_grad[0][0][1][0]) );
        grad[0] += diff[1] * ( (1 - diff[2]) * (first_grad[1][1][0][0] - first_grad[0][1][0][0]) + diff[2] * (first_grad[1][1][1][0] - first_grad[0][1][1][0]) );
        grad[0] *= sc_.resolution_inv;

        grad[1] =  (1 - diff[2]) * ( (1 - diff[0]) * (first_grad[0][1][0][1] - first_grad[0][0][0][1]) + diff[0] * (first_grad[1][1][0][1] - first_grad[1][0][0][1]) );
        grad[1] += diff[2] * ( (1 - diff[0]) * (first_grad[0][1][1][1] - first_grad[0][0][1][1]) + diff[0] * (first_grad[1][1][1][1] - first_grad[1][0][1][1]) );
        grad[1] *= sc_.resolution_inv;

        grad[2] =  (1 - diff[1]) * ( (1 - diff[0]) * (first_grad[0][0][1][2] - first_grad[0][0][0][2]) + diff[0] * (first_grad[1][0][1][2] - first_grad[1][0][0][2]) );
        grad[2] += diff[1] * ( (1 - diff[0]) * (first_grad[0][1][1][2] - first_grad[0][1][0][2]) + diff[0] * (first_grad[1][1][1][2] - first_grad[1][1][0][2]) );
        grad[2] *= sc_.resolution_inv;

    }



}

