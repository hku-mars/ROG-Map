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

#include <rog_map/rog_map_core/counter_map.h>

using namespace rog_map;


namespace rog_map {

    void CounterMap::initCounterMap(
            const Vec3i &half_prob_map_size_i, /* The input is half map size, to ensure the map size is always odds*/
            const double &prob_map_resolution,
            const double &temp_counter_map_resolution,
            const int &inflation_step,
            const bool &map_sliding_en,
            const double &sliding_thresh,
            const Vec3f &fix_map_origin,
            const double &unk_thresh) {

        if (had_been_initialized) {
            throw std::runtime_error(" -- [CounterMap]: init can only be called once!");
        }
        had_been_initialized = true;

        /* 1) compute the resolution ratio, ensure the resolution is an integer (for CORNER) or odd numbered multiple (for CENTER)
         * */
        if (prob_map_resolution > temp_counter_map_resolution) {
            throw std::invalid_argument(
                    " -- [CounterMap]: prob_map_resolution should be smaller than or equal to counter_map_resolution!");
        }


        if (unk_thresh < 0.0 || unk_thresh > 1.0) {
            throw std::invalid_argument(" -- [InfMap]: unk_thresh should be in [0, 1]!");
        }

        int inflation_ratio = std::round(temp_counter_map_resolution / prob_map_resolution);
#ifdef ORIGIN_AT_CENTER
        // When discretized in ORIGIN_AT_CENTER way that the origin is at cell center
        // the inflation ratio should be an odd number
    if (inflation_ratio % 2 == 0) {
        inflation_ratio += 1;
    }
    std::cout << RED << " -- [CounterMap] inflation_ratio: " << inflation_ratio << std::endl;
#endif
        /* 2) compute the counter sliding map size i, which should be larger than prob map
         *   and consider the inflation step, to ensure the counter is correctly updated
         * */

        double counter_map_resolution = prob_map_resolution * inflation_ratio;
        // compute the prob map size in double
        Vec3f half_prob_map_size_d = half_prob_map_size_i.cast<double>() * prob_map_resolution;
        // compute the counter map size in double
        Vec3i half_counter_map_size_i = (half_prob_map_size_d / counter_map_resolution).cast<int>()
                                        + (inflation_step + 1) * Vec3i::Ones();

        /* 3) Initialize the SlidingMap */
        initSlidingMap(half_counter_map_size_i, counter_map_resolution, map_sliding_en, sliding_thresh, fix_map_origin);

        int map_size = sc_.map_size_i.prod();
        md_.sub_grid_num = pow(std::round(counter_map_resolution / prob_map_resolution), 3);
        md_.unk_thresh = ceil(unk_thresh * md_.sub_grid_num);
        md_.unk_thresh = std::min(std::max(1, md_.unk_thresh), md_.sub_grid_num);
        md_.unknown_cnt.resize(map_size, md_.sub_grid_num);
        md_.occupied_cnt.resize(map_size, 0);

        resetLocalMap();
        std::cout << GREEN << " -- [CounterMap] Init successfully -- ." << RESET << std::endl;
        printMapInformation();
    }


    void CounterMap::updateGridCounter(const Vec3f &pos, const GridType &from_type, const GridType &to_type) {
        /*
         * This function update the counter in inflation map
         * all counter should be 0 ~ sub_grid_num
         * when occ_cnt > 0, this grid is considered to be occupied
         * when unk_cnt > sub_grid_num * unk_ratio, this grid is considered to be unknown?
         * */
        TimeConsuming update_t("updateGridCounter", false);
        map_empty_ = false;
        Vec3i id_l, id_g;
        posToGlobalIndex(pos, id_g);

#ifdef COUNTER_MAP_DEBUG
        if (!insideLocalMap(id_g)) {
            throw std::runtime_error(" -- [InfMap]: Update a counter which is not inside the local map.");
        }

        if (from_type == to_type) {
            throw std::runtime_error(" -- [InfMap]: From type is equal to to type.");
        }
#endif
        const int addr = getHashIndexFromGlobalIndex(id_g);

        GridType counter_cell_from_type = getGridType(addr);
        /* Update the counter map */
        if (from_type == GridType::OCCUPIED) {
            md_.occupied_cnt[addr] -= 1;
        }
        if (to_type == GridType::OCCUPIED) {
            md_.occupied_cnt[addr] += 1;
        }
        if (from_type == GridType::UNKNOWN) {
            md_.unknown_cnt[addr] -= 1;
        }
        if (to_type == GridType::UNKNOWN) {
            md_.unknown_cnt[addr] += 1;
        }
#ifdef COUNTER_MAP_DEBUG
        if (md_.occupied_cnt[addr] < 0 || md_.occupied_cnt[addr] > md_.sub_grid_num) {
            std::cout << RED << "From type: " << GridTypeStr[from_type] << RESET << std::endl;
            std::cout << RED << "To type: " << GridTypeStr[to_type] << RESET << std::endl;
            std::cout << RED << "Occupied counter: " << md_.occupied_cnt[addr] << RESET << std::endl;
            throw std::runtime_error(" -- [CouterMap]: Occupied counter is out of range.");
        }
        if (md_.unknown_cnt[addr] < 0 || md_.unknown_cnt[addr] > md_.sub_grid_num) {
            std::cout << RED << "From type: " << from_type << RESET << std::endl;
            std::cout << RED << "To type: " << to_type << RESET << std::endl;
            std::cout << RED << "Unknown counter: " << md_.unknown_cnt[addr] << RESET << std::endl;
            throw std::runtime_error(" -- [CouterMap]: Unknown counter is out of range.");
        }
#endif

        GridType counter_cell_to_type = getGridType(addr);

        if (counter_cell_from_type != counter_cell_to_type) {
            triggerJumpingEdge(id_g, counter_cell_from_type, counter_cell_to_type);
        }
    }

    bool CounterMap::isUnknown(const Vec3i &id_g) const {
        return isUnknown(getHashIndexFromGlobalIndex(id_g));
    }

    bool CounterMap::isOccupied(const Vec3i &id_g) const {
        return isOccupied(getHashIndexFromGlobalIndex(id_g));
    }

    bool CounterMap::isKnownFree(const Vec3i &id_g) const {
        return isKnownFree(getHashIndexFromGlobalIndex(id_g));
    }

}