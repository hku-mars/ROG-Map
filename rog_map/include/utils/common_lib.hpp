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

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <algorithm>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <vector>
#include <Eigen/Eigen>


#define DEBUG_FILE_DIR(name) (string(string(ROOT_DIR) + "log/"+name))
#define PCD_FILE_DIR(name) (string(string(ROOT_DIR) + "pcd/"+name))

namespace rog_map {
    typedef double decimal_t;
    using Vec3i = Eigen::Matrix<int, 3, 1>;
    using Vec3f = Eigen::Matrix<decimal_t, 3, 1>;
    using Quatf = Eigen::Quaterniond;
    using Pose = std::pair<Vec3f, Eigen::Quaterniond>;
    template<typename T>
    using vec_E = std::vector<T, Eigen::aligned_allocator<T>>;
    using vec_Vec3i = vec_E<Vec3i>;
    using vec_Vec3f = vec_E<Vec3f>;

    using std::vector;
    using std::string;
    using std::cout;
    using std::endl;


    constexpr int AXIS_X = 0;
    constexpr int AXIS_Y = 1;
    constexpr int AXIS_Z = 2;
    const Vec3f ZeroPointFive3d(0.5, 0.5, 0.5);

    const std::string RED = "\033[1;31m";
    const std::string GREEN = "\033[1;32m";
    const std::string YELLOW = "\033[1;33m";
    const std::string BLUE = "\033[1;34m";
    const std::string RESET = "\033[0m";

    typedef pcl::PointXYZINormal PclPoint;
    typedef pcl::PointCloud<PclPoint> PointCloud;


    enum GridType {
        UNDEFINED = 0,
        UNKNOWN = 1,
        OUT_OF_MAP,
        OCCUPIED,
        KNOWN_FREE,
        FRONTIER, // The frontier is an unknown grid which is adjacent to the known free grid
    };

    const static std::vector<std::string> GridTypeStr{"UNDEFINED",
                                                      "UNKNOWN",
                                                      "OUT_OF_MAP",
                                                      "OCCUPIED",
                                                      "KNOWN_FREE",
                                                      "FRONTIER"};


#define SIGN(x) ((x > 0) - (x < 0))
#define DEBUG_FILE_DIR(name) (string(string(ROOT_DIR) + "log/"+name))
#define PCD_FILE_DIR(name) (string(string(ROOT_DIR) + "pcd/"+name))

    template<typename T>
    std::ostream &operator<<(std::ostream &out, const std::vector<T> &v) {
        out << "[";
        for (typename std::vector<T>::const_iterator it = v.begin(); it != v.end(); ++it) {
            out << *it;
            if (it != v.end() - 1) {
                out << ", ";
            }
        }
        out << "]";
        return out;
    }

    struct RobotState {
        Vec3f p, v, a, j;
        double yaw;
        double rcv_time;
        bool rcv{false};
        Quatf q;
    };


    template<typename Scalar_t>
    static Eigen::Matrix<Scalar_t, 3, 1> quaternion_to_ypr(const Eigen::Quaternion<Scalar_t> &q_) {
        Eigen::Quaternion<Scalar_t> q = q_.normalized();

        Eigen::Matrix<Scalar_t, 3, 1> ypr;
        ypr(2) = atan2(2 * (q.w() * q.x() + q.y() * q.z()), 1 - 2 * (q.x() * q.x() + q.y() * q.y()));
        ypr(1) = asin(2 * (q.w() * q.y() - q.z() * q.x()));
        ypr(0) = atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.y() * q.y() + q.z() * q.z()));

        return ypr;
    }

    template<typename Scalar_t>
    static Scalar_t get_yaw_from_quaternion(const Eigen::Quaternion<Scalar_t> &q) {
        return quaternion_to_ypr(q)(0);
    }




    static double computePathLength(const vec_E<Vec3f> &path) {
        if (path.size() < 2) {
            return 0.0;
        }
        double len = 0.0;
        for (size_t i = 0; i < path.size() - 1; i++) {
            len += (path[i] - path[i + 1]).norm();
        }
        return len;
    }

    static Vec3f lineBoxIntersectPoint(const Vec3f &pt, const Vec3f &pos,
                                       const Vec3f &box_min, const Vec3f &box_max) {
        Eigen::Vector3d diff = pt - pos;
        Eigen::Vector3d max_tc = box_max - pos;
        Eigen::Vector3d min_tc = box_min - pos;

        double min_t = 1000000;

        for (int i = 0; i < 3; ++i) {
            if (fabs(diff[i]) > 0) {

                double t1 = max_tc[i] / diff[i];
                if (t1 > 0 && t1 < min_t)
                    min_t = t1;

                double t2 = min_tc[i] / diff[i];
                if (t2 > 0 && t2 < min_t)
                    min_t = t2;
            }
        }

        return pos + (min_t - 1e-3) * diff;
    }

    static bool GetIntersection(float fDst1, float fDst2, Vec3f P1, Vec3f P2, Vec3f &Hit) {
        if ((fDst1 * fDst2) >= 0.0f) return false;
        if (fDst1 == fDst2) return false;
        Hit = P1 + (P2 - P1) * (-fDst1 / (fDst2 - fDst1));
        return true;
    }

    static bool InBox(Vec3f Hit, Vec3f B1, Vec3f B2, const int Axis) {
        if (Axis == 1 && Hit.z() > B1.z() && Hit.z() < B2.z() && Hit.y() > B1.y() && Hit.y() < B2.y()) return true;
        if (Axis == 2 && Hit.z() > B1.z() && Hit.z() < B2.z() && Hit.x() > B1.x() && Hit.x() < B2.x()) return true;
        if (Axis == 3 && Hit.x() > B1.x() && Hit.x() < B2.x() && Hit.y() > B1.y() && Hit.y() < B2.y()) return true;
        return false;
    }

//The box in this article is Axis-Aligned and so can be defined by only two 3D points:
// B1 - the smallest values of X, Y, Z
//        B2 - the largest values of X, Y, Z
// returns true if line (L1, L2) intersects with the box (B1, B2)
// returns intersection point in Hit
    static bool lineIntersectBox(Vec3f L1, Vec3f L2, Vec3f B1, Vec3f B2, Vec3f &Hit) {
        if (L2.x() < B1.x() && L1.x() < B1.x()) return false;
        if (L2.x() > B2.x() && L1.x() > B2.x()) return false;
        if (L2.y() < B1.y() && L1.y() < B1.y()) return false;
        if (L2.y() > B2.y() && L1.y() > B2.y()) return false;
        if (L2.z() < B1.z() && L1.z() < B1.z()) return false;
        if (L2.z() > B2.z() && L1.z() > B2.z()) return false;

        // inside box seems intersect
//        if (L1.x() > B1.x() && L1.x() < B2.x() &&
//            L1.y() > B1.y() && L1.y() < B2.y() &&
//            L1.z() > B1.z() && L1.z() < B2.z()) {
//            Hit = L1;
//            return true;
//        }

        if ((GetIntersection(L1.x() - B1.x(), L2.x() - B1.x(), L1, L2, Hit) && InBox(Hit, B1, B2, 1))
            || (GetIntersection(L1.y() - B1.y(), L2.y() - B1.y(), L1, L2, Hit) && InBox(Hit, B1, B2, 2))
            || (GetIntersection(L1.z() - B1.z(), L2.z() - B1.z(), L1, L2, Hit) && InBox(Hit, B1, B2, 3))
            || (GetIntersection(L1.x() - B2.x(), L2.x() - B2.x(), L1, L2, Hit) && InBox(Hit, B1, B2, 1))
            || (GetIntersection(L1.y() - B2.y(), L2.y() - B2.y(), L1, L2, Hit) && InBox(Hit, B1, B2, 2))
            || (GetIntersection(L1.z() - B2.z(), L2.z() - B2.z(), L1, L2, Hit) && InBox(Hit, B1, B2, 3)))
            return true;

        return false;
    }
}

