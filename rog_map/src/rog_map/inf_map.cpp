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

#include <rog_map/inf_map.h>

namespace rog_map {
// Public Query Function ========================================================================
    bool InfMap::isOccupiedInflate(const Vec3f &pos) const {
        if (!insideLocalMap(pos)) return false;
        if (pos.z() > cfg_.virtual_ceil_height) return false;
        if (pos.z() < cfg_.virtual_ground_height) return false;
        return imd_.occ_inflate_cnt[getHashIndexFromPos(pos)] > 0;
    }

    bool InfMap::isOccupiedInflate(const Vec3i &id_g) const {
        if (!insideLocalMap(id_g)) return false;
        if (id_g.z() > cfg_.virtual_ceil_height_id_g) return false;
        if (id_g.z() < cfg_.virtual_ground_height_id_g) return false;
        return imd_.occ_inflate_cnt[getHashIndexFromGlobalIndex(id_g)] > 0;
    }

    bool InfMap::isKnownFreeInflate(const Vec3f& pos) const {
        if(!cfg_.unk_inflation_en) {
            return !isOccupiedInflate(pos);
        } else {
            return (!isOccupiedInflate(pos) && !isUnknownInflate(pos));
        }
    }


    bool InfMap::isUnknownInflate(const Vec3f &pos) const {
        if (!cfg_.unk_inflation_en) {
            throw std::runtime_error(
                    "Unknown inflation is not enabled, but the isUnknownInflate API is called, which should not happen.");
        }
        // 1. check virtual ceil and ground
        if (pos.z() >= cfg_.virtual_ceil_height - cfg_.inflation_resolution ||
            pos.z() <= cfg_.virtual_ground_height + cfg_.inflation_resolution) {
            return false;
        }
        return imd_.unk_inflate_cnt[getHashIndexFromPos(pos)] > 0;
    }

    InfMap::InfMap(rog_map::Config &cfg)  {

        cfg_ = cfg;
        int max_step = cfg_.inflation_step;
        if (cfg_.unk_inflation_en) {
            max_step = std::max(max_step, cfg_.unk_inflation_step);
        }

        initCounterMap(cfg.half_map_size_i,
                       cfg.resolution,
                       cfg.inflation_resolution,
                       max_step,
                       cfg.map_sliding_en,
                       cfg.map_sliding_thresh,
                       cfg.fix_map_origin,
                       cfg.unk_thresh);

        posToGlobalIndex(cfg.visualization_range, sc_.visualization_range_i);
        cfg.virtual_ceil_height_id_g =
                int(cfg.virtual_ceil_height / cfg.inflation_resolution + SIGN(cfg.inflation_resolution) * 0.5) -
                cfg.inflation_step;
        cfg.virtual_ground_height_id_g =
                int(cfg.virtual_ground_height / cfg.inflation_resolution + SIGN(cfg.inflation_resolution) * 0.5) +
                cfg.inflation_step;
        cfg.virtual_ceil_height = cfg.virtual_ceil_height_id_g * cfg.inflation_resolution;
        cfg.virtual_ground_height = cfg.virtual_ground_height_id_g * cfg.inflation_resolution;

        imd_.occ_inflate_cnt.resize(sc_.map_vox_num);
        imd_.occ_neighbor_num = cfg.spherical_neighbor.size();
        if (cfg.unk_inflation_en) {
            imd_.unk_neighbor_num = cfg.unk_spherical_neighbor.size();
            // Considering the all grids are unknown at the beginning
            // the unk inf cnt should be the size of inflation queue
            imd_.unk_inflate_cnt.resize(sc_.map_vox_num);
        }
        posToGlobalIndex(cfg.visualization_range, sc_.visualization_range_i);

        resetLocalMap();
        cfg_ = cfg;
        std::cout << GREEN << " -- [InfMap] Init successfully -- ." << RESET << std::endl;
        printMapInformation();
    }

    void InfMap::getInflationNumAndTime(double &inf_n, double &inf_t) {
        inf_n = inf_num_;
        inf_num_ = 0;
        inf_t = inf_t_;
        inf_t_ = 0;
    }

    void InfMap::writeMapInfoToLog(std::ofstream &log_file) {
        log_file << "[InfMap]" << std::endl;
        log_file << "\tresolution: " << sc_.resolution << std::endl;
        log_file << "\tmap_size_i: " << sc_.map_size_i.transpose() << std::endl;
        log_file << "\tmap_size_d: " << (sc_.map_size_i.cast<double>() * sc_.resolution).transpose() << std::endl;
    }

    void
    InfMap::boxSearch(const Vec3f &box_min, const Vec3f &box_max, const GridType &gt, vec_E<Vec3f> &out_points) const {
        out_points.clear();
        if (map_empty_) {
            std::cout << RED << " -- [ROG] Map is empty, cannot perform box search." << RESET << std::endl;
            return;
        }
        Vec3i box_min_id_g, box_max_id_g;
        posToGlobalIndex(box_min, box_min_id_g);
        posToGlobalIndex(box_max, box_max_id_g);
        Vec3i box_size = box_max_id_g - box_min_id_g;
        if (gt == UNKNOWN) {
            if (!cfg_.unk_inflation_en) {
                out_points.clear();
                std::cout << RED << " -- [ROG] Unknown inflation is not enabled, cannot perform box search." << RESET
                          << std::endl;
                return;
            }
            out_points.reserve(box_size.prod());
            for (int i = box_min_id_g.x(); i <= box_max_id_g.x(); i++) {
                for (int j = box_min_id_g.y(); j <= box_max_id_g.y(); j++) {
                    for (int k = box_min_id_g.z(); k <= box_max_id_g.z(); k++) {
                        Vec3i id_g(i, j, k);
                        if (isUnknown(id_g)) {
                            Vec3f pos;
                            globalIndexToPos(id_g, pos);
                            out_points.push_back(pos);
                        }
                    }
                }
            }
        } else if (gt == OCCUPIED) {
            out_points.reserve(box_size.prod() / 3);
            for (int i = box_min_id_g.x(); i <= box_max_id_g.x(); i++) {
                for (int j = box_min_id_g.y(); j <= box_max_id_g.y(); j++) {
                    for (int k = box_min_id_g.z(); k <= box_max_id_g.z(); k++) {
                        Vec3i id_g(i, j, k);
                        if (isOccupiedInflate(id_g)) {
                            Vec3f pos;
                            globalIndexToPos(id_g, pos);
                            out_points.push_back(pos);
                        }
                    }
                }
            }
        } else {
            throw std::runtime_error(" -- [ROG-Map] Box search does not support KNOWN_FREE.");
        }

    }


    void InfMap::resetLocalMap() {
        std::cout << RED << " -- [Inf-Map] Clear all local map." << RESET << std::endl;
        std::fill(md_.unknown_cnt.begin(), md_.unknown_cnt.end(), md_.sub_grid_num);
        std::fill(md_.occupied_cnt.begin(), md_.occupied_cnt.end(), 0);
        std::fill(imd_.occ_inflate_cnt.begin(), imd_.occ_inflate_cnt.end(), 0);
        if (cfg_.unk_inflation_en) {
            std::fill(imd_.unk_inflate_cnt.begin(), imd_.unk_inflate_cnt.end(), imd_.unk_neighbor_num);
        }
    }

    void InfMap::updateInflation(const Vec3i &id_g, const bool is_hit) {
        TimeConsuming tc("updateInflation", false);
        for (const auto &nei: cfg_.spherical_neighbor) {
            const Vec3i &id_shift = id_g + nei;
#ifdef COUNTER_MAP_DEBUG
            if (!insideLocalMap(id_shift)) {
                throw std::runtime_error(" -- [IM] inflation out of map.");
            }
#endif
            inf_num_++;
            const int &addr = getHashIndexFromGlobalIndex(id_shift);
            if (is_hit) {
                imd_.occ_inflate_cnt[addr]++;
            } else {
                imd_.occ_inflate_cnt[addr]--;
            }

#ifdef COUNTER_MAP_DEBUG
            if (imd_.occ_inflate_cnt[addr] < 0 || imd_.occ_inflate_cnt[addr] > imd_.occ_neighbor_num) {
                imd_.occ_inflate_cnt[addr] = 0;
                throw std::runtime_error(" -- [IM] Negative occupancy counter, which should not happened.!");
            }
#endif

        }
        inf_t_ += tc.stop();
    }

    void InfMap::updateUnkInflation(const Vec3i &id_g, const bool is_add) {
        TimeConsuming tc("updateInflation", false);
        if (!cfg_.unk_inflation_en) {
            std::cout << RED << "Cannot updateUnkInflation of InfMap when unk_inflation_en is false." << RESET
                      << std::endl;
            return;
        }

        for (const auto &nei: cfg_.unk_spherical_neighbor) {
            const Vec3i &id_shift = id_g + nei;
#ifdef COUNTER_MAP_DEBUG
            if (!insideLocalMap(id_shift)) {
                throw std::runtime_error(" -- [IM] Unknown inflation out of map.");
            }
#endif
            inf_num_++;
            const int &addr = getHashIndexFromGlobalIndex(id_shift);
            if (is_add) {
                imd_.unk_inflate_cnt[addr]++;
            } else {
                imd_.unk_inflate_cnt[addr]--;
            }

#ifdef COUNTER_MAP_DEBUG
            // only for bug report
            if (imd_.unk_inflate_cnt[addr] < 0 || imd_.unk_inflate_cnt[addr] > imd_.unk_neighbor_num) {
                std::cout << "unk_inflate_cnt: " << imd_.unk_inflate_cnt[addr] << " unk_neighbor_num: "
                          << imd_.unk_neighbor_num << std::endl;
                throw std::runtime_error(" -- [IM] Negative occupancy counter, which should not happened.!");
            }
#endif
        }
        inf_t_ += tc.stop();
    }

    void InfMap::triggerJumpingEdge(const rog_map::Vec3i& id_g,
        const rog_map::GridType& from_type,
        const rog_map::GridType& to_type) {
        if (from_type == GridType::OCCUPIED) {
            updateInflation(id_g, false);
        }
        if (to_type == GridType::OCCUPIED) {
            updateInflation(id_g, true);
        }
        if (cfg_.unk_inflation_en) {
            if (from_type == GridType::UNKNOWN) {
                updateUnkInflation(id_g, false);
            }
            if (to_type == GridType::UNKNOWN) {
                updateUnkInflation(id_g, true);
            }
        }
    }

    void InfMap::resetOneCell(const int& hash_id) {
        GridType cur_grid_type = CounterMap::getGridType(hash_id);
        if (cur_grid_type != GridType::UNKNOWN) {
            Vec3i id_g;
            hashIdToGlobalIndex(hash_id, id_g);
            triggerJumpingEdge(id_g, cur_grid_type, GridType::UNKNOWN);
        }
    }

    GridType InfMap::getGridType(const Vec3i& id_g) const {
        if (!insideLocalMap(id_g)) {
            return OUT_OF_MAP;
        }
        Vec3i id_l;
        globalIndexToLocalIndex(id_g, id_l);
        int addr = getLocalIndexHash(id_l);
        // The Occupied is defined by inflation layer
        if (imd_.occ_inflate_cnt[addr] > 0) {
            return OCCUPIED;
        } else if (cfg_.unk_inflation_en && imd_.unk_inflate_cnt[addr] > 0) {
            return UNKNOWN;
        } else {
            return KNOWN_FREE;
        }
    }

    GridType InfMap::getGridType(const Vec3f& pos) const  {
        Vec3i id_g, id_l;
        // 1. check virtual ceil and ground
        if (pos.z() >= cfg_.virtual_ceil_height  - cfg_.inflation_resolution ||
            pos.z() <= cfg_.virtual_ground_height  + cfg_.inflation_resolution) {
            return OCCUPIED;
            }
        posToGlobalIndex(pos, id_g);
        // 2. get true grid type
        return getGridType(id_g);
    }







}