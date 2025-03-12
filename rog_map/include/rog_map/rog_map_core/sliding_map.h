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

#include <rog_map/rog_map_core/config.hpp>
#include <utils/scope_timer.hpp>

namespace rog_map {
    /// The policy of ORIGIN_AT_CORNER is:
    //    for all cells, the pos is defined at the center of the cell
    //    including the map boundaries.
    //    [ORIGIN] And for the origin, it is defined on the cornerQ of the cell
    //      bd_min       origin           bd_max
    //        |              |              |
    //        v              V              V
    //        -2        -1        0         1
    //   |---------|---------|---------|---------|
    //  -2  -1.5  -1    0.5  0   0.5   1   1.5   2

    class SlidingMap {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        SlidingMap(const Vec3i &half_map_size_i,
                   const double &resolution,
                   const bool &map_sliding_en,
                   const double &sliding_thresh,
                   const Vec3f &fix_map_origin);

        SlidingMap() = default;

        void initSlidingMap(const Vec3i &half_map_size_i,
                  const double &resolution,
                  const bool &map_sliding_en,
                  const double &sliding_thresh,
                  const Vec3f &fix_map_origin);

        void printMapInformation();

        void mapSliding(const Vec3f &odom);

        bool insideLocalMap(const Vec3f &pos) const;

        bool insideLocalMap(const Vec3i &id_g) const;

    protected:
        struct SlidingConfig {
            double resolution{0.0};
            double resolution_inv{0.0};
            double sliding_thresh{0.0};
            bool map_sliding_en{false};
            Vec3f fix_map_origin{};
            Vec3i visualization_range_i{};
            Vec3i map_size_i{};
            Vec3i half_map_size_i{};
            int virtual_ceil_height_id_g{0};
            int virtual_ground_height_id_g{0};
            int map_vox_num{0};
        } sc_;

        Vec3f local_map_origin_d_, local_map_bound_min_d_, local_map_bound_max_d_;
        Vec3i local_map_origin_i_, local_map_bound_min_i_, local_map_bound_max_i_;

        virtual void resetLocalMap() = 0;

        /* When map sliding, the memory out of the local map should be cleared,
         * which cause a cell state jumped to unknown. Thus, the method of resetCell
         * should be implemented in the derived class.
         *
         * In ROG Map, this api is only for ProbMap and CounterMap, for the derived classes
         * from counter map, check the resetOneCell method in CounterMap.
         * */

        virtual void resetCell(const int & hash_id) = 0;

        void clearMemoryOutOfMap(const vector<int> &clear_id, const int &i);


        int getLocalIndexHash(const Vec3i &id_in) const;

        void posToGlobalIndex(const Vec3f &pos, Vec3i &id) const;

        void posToGlobalIndex(const double &pos, int &id) const;

        void globalIndexToPos(const Vec3i &id_g, Vec3f &pos) const;

        void globalIndexToLocalIndex(const Vec3i &id_g, Vec3i &id_l) const;

        /* Only used in clearMemoryOutOfMap and */
        void localIndexToGlobalIndex(const Vec3i &id_l, Vec3i &id_g) const;

        void localIndexToPos(const Vec3i &id_l, Vec3f &pos) const;

        void hashIdToLocalIndex(const int &hash_id,
                                Vec3i &id) const;

        void hashIdToPos(const int &hash_id,
                         Vec3f &pos) const;

        void hashIdToGlobalIndex(const int &hash_id,
                                 Vec3i &id_g) const;

        int getHashIndexFromPos(const Vec3f &pos) const;

        int getHashIndexFromGlobalIndex(const Vec3i &id_g) const;

        void updateLocalMapOriginAndBound(const Vec3f &new_origin_d,
                                          const Vec3i &new_origin_i);


    private:
        bool had_been_initialized{false};

    };


}