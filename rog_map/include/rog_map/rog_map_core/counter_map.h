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

#include <rog_map/rog_map_core/sliding_map.h>

// #define COUNTER_MAP_DEBUG
namespace rog_map {

    class CounterMap : public SlidingMap {
    public:
        typedef std::shared_ptr<CounterMap> Ptr;

        CounterMap() = default;

        ~CounterMap() = default;


        void updateGridCounter(const Vec3f &pos,
                               const GridType &from_type,
                               const GridType &to_type);


    protected:
        bool map_empty_{true};

        struct MapData {
            std::vector<int16_t> occupied_cnt;
            std::vector<int16_t> unknown_cnt;
            int sub_grid_num;
            /* If the number of unk sub grid larger than unk thresh
             * this grid is considered to be unknown
             * */
            int unk_thresh;
        } md_;


        void initCounterMap(
                const Vec3i &half_prob_map_size_i, /* The input is half map size, to ensure the map size is always odds*/
                const double &prob_map_resolution,
                const double &temp_counter_map_resolution,
                const int &inflation_step,
                const bool &map_sliding_en,
                const double &sliding_thresh,
                const Vec3f &fix_map_origin,
                const double &unk_thresh);


        /* The '=' is necessary. When unk thresh is set to 1.0, a cell in counter map
         * is considered to be unknown when all sub-cells are unknown, which means
         * a cell is UNKNOWN only when md_.unknown_cnt[hash_id] == md_.unk_thresh;
         * without '=', a cell is always known, which is wrong.
         * */


        GridType getGridType(const int &hash_id) const {
            /* The grid type defined in ROG-Map counter map is defined as follows:
             * 1) Any sub-cell is occupied, the grid is occupied
             * 2) is non of the sub-cell is occupied:
             *  2.1) if the number of unknown sub-cell is larger than unk_thresh, the grid is unknown
             *  2.2) else this grid is known free
             * */
            if (isOccupied(hash_id)) {
                return GridType::OCCUPIED;
            } else {
                if (isUnknown(hash_id)) {
                    return GridType::UNKNOWN;
                } else {
                    return GridType::KNOWN_FREE;
                }
            }
        }

        virtual void triggerJumpingEdge(const Vec3i &id_g,
                                        const GridType &from_type,
                                        const GridType &to_type) = 0;

        /* When map sliding, a reset command is called, the counter map
         * should reset the unk and occ counter and at the same time call
         * reset to the derived class to reset the map
         * */

        virtual void resetOneCell(const int &hash_id) = 0;

        bool isUnknown(const Vec3i &id_g) const;

        bool isOccupied(const Vec3i &id_g) const;

        bool isKnownFree(const Vec3i &id_g) const;

        bool isUnknown(const int &hash_id) const {
            return md_.unknown_cnt[hash_id] >= md_.unk_thresh;
        }

        bool isOccupied(const int &hash_id) const {
            return md_.occupied_cnt[hash_id] > 0;
        }

        bool isKnownFree(const int &hash_id) const {
            return (md_.unknown_cnt[hash_id] < md_.unk_thresh) && /* 1) not known */
                   (md_.occupied_cnt[hash_id] == 0); /* 2) not occupied */
        }

    private:
        void resetCell(const int &hash_id) override {
            md_.occupied_cnt[hash_id] = 0;
            md_.unknown_cnt[hash_id] = md_.sub_grid_num;
            resetOneCell(hash_id);
        }

        bool had_been_initialized{false};


    };
}

