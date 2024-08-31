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

namespace rog_map {

    class InfMap : public CounterMap {
    public:
        typedef std::shared_ptr<InfMap> Ptr;
        InfMap(rog_map::Config &cfg);
        ~InfMap() = default;


        double getResolution()const {
            return sc_.resolution;
        }

        bool isOccupiedInflate(const Vec3f &pos) const;

        bool isUnknownInflate(const Vec3f &pos) const;

        bool isKnownFreeInflate(const Vec3f &pos) const;

        void getInflationNumAndTime(double &inf_n, double &inf_t);

        void writeMapInfoToLog(std::ofstream &log_file);

        void boxSearch(const Vec3f &box_min, const Vec3f &box_max,
                       const GridType &gt, vec_E<Vec3f> &out_points) const;

        void resetLocalMap() override;

        GridType getGridType(const Vec3f &pos) const;

        GridType getGridType(const Vec3i &id_g) const ;

    private:
        struct InfMapData {
            std::vector<int16_t> occ_inflate_cnt;
            std::vector<int16_t> unk_inflate_cnt;
            int unk_neighbor_num;
            int occ_neighbor_num;
        } imd_;

        rog_map::Config cfg_;
        int inf_num_{0};
        double inf_t_{0.0};

        void triggerJumpingEdge(const rog_map::Vec3i &id_g,
                                const rog_map::GridType &from_type,
                                const rog_map::GridType &to_type) override;

        bool isOccupiedInflate(const Vec3i &id_g) const;

        void updateInflation(const Vec3i &id_g, const bool is_hit);

        void updateUnkInflation(const Vec3i &id_g, const bool is_add);

        void resetOneCell(const int &hash_id) override;
    };
}

