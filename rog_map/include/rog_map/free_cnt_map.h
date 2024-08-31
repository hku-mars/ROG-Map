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

///TODO inherit from counter map

#pragma once

#include <rog_map/rog_map_core/sliding_map.h>

namespace rog_map {

    class FreeCntMap : public SlidingMap {
    public:
        typedef std::shared_ptr<FreeCntMap> Ptr;

        FreeCntMap(const Vec3i &half_map_size_i,
                   const double &resolution,
                   const bool &sliding_en,
                   const double &sliding_thresh,
                   const Vec3f &fix_map_origin) : SlidingMap(half_map_size_i,
                                                         resolution,
                                                         sliding_en,
                                                         sliding_thresh,
                                                         fix_map_origin) {
            int map_size = sc_.map_size_i.prod();
            neighbor_free_cnt.resize(map_size, 0);
            resetLocalMap();
            std::cout << GREEN << " -- [InfMap] Init successfully -- ." << RESET << std::endl;
            printMapInformation();
        }

        ~FreeCntMap() = default;


        void resetLocalMap() override {
            std::cout << RED << " -- [Fro-Map] Clear all local map."<<RESET << std::endl;
            std::fill(neighbor_free_cnt.begin(), neighbor_free_cnt.end(), 0);
        }

        int getFreeCnt(const Vec3f &pos) {
            return neighbor_free_cnt[getHashIndexFromPos(pos)];
        }

        int getFreeCnt(const Vec3i &id_g) {
            return neighbor_free_cnt[getHashIndexFromGlobalIndex(id_g)];
        }

        void updateFrontierCounter(const Vec3i &id_g, bool add) {
            if (!insideLocalMap(id_g)) {
                return;
            }
            Vec3i neighbor_id_g;
            for (int i = -1; i <= 1; ++i) {
                neighbor_id_g[0] = id_g[0] + i;
                for (int j = -1; j <= 1; ++j) {
                    neighbor_id_g[1] = id_g[1] + j;
                    for (int k = -1; k <= 1; ++k) {
                        neighbor_id_g[2] = id_g[2] + k;
                        int hash_id = getHashIndexFromGlobalIndex(neighbor_id_g);

                        if (add) {
                            neighbor_free_cnt[hash_id] += 1;
                            // only for bug report
                            if (neighbor_free_cnt[hash_id] > 27) {
                                throw std::runtime_error("Frontier counter overflow with larger than 26");
                            }
                        } else {
                            neighbor_free_cnt[hash_id] -= 1;
                            if (neighbor_free_cnt[hash_id] < 0) {
                                throw std::runtime_error("Frontier counter overflow with smaller than 0");
                            }
                        }
                    }
                }
            }
        }

        void resetCell(const int &hash_id) override {
            neighbor_free_cnt[hash_id] = 0;
        }

    private:
        bool map_empty_{true};
        std::vector<int16_t> neighbor_free_cnt;
        rog_map::Config cfg_;

    };

}

