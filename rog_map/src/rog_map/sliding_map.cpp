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

#include <rog_map/rog_map_core/sliding_map.h>

using namespace rog_map;

SlidingMap::SlidingMap(const Vec3i &half_map_size_i, const double &resolution, const bool &map_sliding_en,
                       const double &sliding_thresh, const Vec3f &fix_map_origin) {
    std::cout<<"half_map_size_i: "<<half_map_size_i.transpose()<<std::endl;
    std::cout<<"resolution: "<<resolution<<std::endl;
    std::cout<<"map_sliding_en: "<<map_sliding_en<<std::endl;
    std::cout<<"sliding_thresh: "<<sliding_thresh<<std::endl;
    std::cout<<"fix_map_origin: "<<fix_map_origin.transpose()<<std::endl;
    initSlidingMap(half_map_size_i, resolution, map_sliding_en, sliding_thresh, fix_map_origin);
}

void
SlidingMap::initSlidingMap(const rog_map::Vec3i &half_map_size_i, const double &resolution, const bool &map_sliding_en,
                           const double &sliding_thresh, const rog_map::Vec3f &fix_map_origin) {
    if (had_been_initialized) {
        throw std::runtime_error(" -- [SlidingMap]: init can only be called once!");
    }
#ifdef ORIGIN_AT_CORNER
#ifdef ORIGIN_AT_CENTER
    throw std::runtime_error(" -- [SlidingMap]: ORIGIN_AT_CORNER and ORIGIN_AT_CENTER cannot be both true!");
#endif
#endif
    sc_.resolution = resolution;
    sc_.resolution_inv = 1.0 / resolution;
    sc_.map_sliding_en = map_sliding_en;
    sc_.sliding_thresh = sliding_thresh;
    sc_.fix_map_origin = fix_map_origin;
    sc_.half_map_size_i = half_map_size_i;
    sc_.map_size_i = 2 * sc_.half_map_size_i + Vec3i::Constant(1);
    sc_.map_vox_num = sc_.map_size_i.prod();
    if (!map_sliding_en) {
        local_map_origin_d_ = fix_map_origin;
        posToGlobalIndex(local_map_origin_d_, local_map_origin_i_);
    }
    had_been_initialized = true;
}

void SlidingMap::printMapInformation() {
    std::cout << GREEN << "\tresolution: " << sc_.resolution << RESET << std::endl;
    std::cout << GREEN << "\tmap_sliding_en: " << sc_.map_sliding_en << RESET << std::endl;
    std::cout << GREEN << "\tlocal_map_size_i: " << sc_.map_size_i.transpose() << RESET << std::endl;
    std::cout << GREEN << "\tlocal_map_size_d: " << sc_.map_size_i.cast<double>().transpose() * sc_.resolution << RESET
              << std::endl;
}

bool SlidingMap::insideLocalMap(const Vec3f &pos) const {
    Vec3i id_g;
    posToGlobalIndex(pos, id_g);
    return insideLocalMap(id_g);
}

bool SlidingMap::insideLocalMap(const Vec3i &id_g) const {
    if (((id_g - local_map_origin_i_).cwiseAbs() - sc_.half_map_size_i).maxCoeff() > 0) {
        return false;
    }
    return true;
}

void SlidingMap::updateLocalMapOriginAndBound(const rog_map::Vec3f &new_origin_d, const rog_map::Vec3i &new_origin_i) {
    // update local map origin and local map bound
    local_map_origin_i_ = new_origin_i;
    local_map_origin_d_ = new_origin_d;

    local_map_bound_max_i_ = local_map_origin_i_ + sc_.half_map_size_i;
    local_map_bound_min_i_ = local_map_origin_i_ - sc_.half_map_size_i;

    // the double map bound only consider the closed cell center
    globalIndexToPos(local_map_bound_min_i_, local_map_bound_min_d_);
    globalIndexToPos(local_map_bound_max_i_, local_map_bound_max_d_);
}

void SlidingMap::clearMemoryOutOfMap(const vector<int> &clear_id, const int &i) {
    vector<int> ids{i, (i + 1) % 3, (i + 2) % 3};
    for (const auto &idd: clear_id) {
        for (int x = -sc_.half_map_size_i(ids[1]); x <= sc_.half_map_size_i(ids[1]); x++) {
            for (int y = -sc_.half_map_size_i(ids[2]); y <= sc_.half_map_size_i(ids[2]); y++) {
                Vec3i temp_clear_id;
                temp_clear_id(ids[0]) = idd;
                temp_clear_id(ids[1]) = x;
                temp_clear_id(ids[2]) = y;
                resetCell(getLocalIndexHash(temp_clear_id));
            }
        }
    }
}

void SlidingMap::mapSliding(const Vec3f &odom) {
    TimeConsuming update_local_shift_timer("updateLocalShift", false);
    /// Compute the shifting index
    Vec3i new_origin_i;
    posToGlobalIndex(odom, new_origin_i);
    Vec3f new_origin_d = new_origin_i.cast<double>() * sc_.resolution;
    /// Compute the delta shift
    Vec3i shift_num = new_origin_i - local_map_origin_i_;
    for (long unsigned int i = 0; i < 3; i++) {
        if (fabs(shift_num[i]) > sc_.map_size_i[i]) {
            // Clear all map
            resetLocalMap();
            updateLocalMapOriginAndBound(new_origin_d, new_origin_i);
            return;
        }
    }
    static auto normalize = [](int x, int a, int b) -> int {
        int range = b - a + 1;
        int y = (x - a) % range;
        return (y < 0 ? y + range : y) + a;
    };

    /// Clear the memory out of the map size
    for (int i = 0; i < 3; i++) {
        if (shift_num[i] == 0) {
            continue;
        }
        int min_id_g = -sc_.half_map_size_i(i) + local_map_origin_i_(i);
        int min_id_l = min_id_g % sc_.map_size_i(i);
        vector<int> clear_id;
        if (shift_num(i) > 0) {
            /// forward shift, the min id should be cut
            for (int k = 0; k < shift_num(i); k++) {
                int temp_id = min_id_l + k;
                temp_id = normalize(temp_id, -sc_.half_map_size_i(i), sc_.half_map_size_i(i));
                clear_id.push_back(temp_id);
            }
        } else {
            /// backward shift, the max should be shifted
            for (int k = -1; k >= shift_num(i); k--) {
                int temp_id = min_id_l + k;
                temp_id = normalize(temp_id, -sc_.half_map_size_i(i), sc_.half_map_size_i(i));
                clear_id.push_back(temp_id);
            }
        }

        if (clear_id.empty()) {
            continue;
        }
        clearMemoryOutOfMap(clear_id, i);
    }

    updateLocalMapOriginAndBound(new_origin_d, new_origin_i);
}

int SlidingMap::getLocalIndexHash(const Vec3i &id_in) const {
    Vec3i id = id_in + sc_.half_map_size_i;
    return id(0) * sc_.map_size_i(1) * sc_.map_size_i(2) +
           id(1) * sc_.map_size_i(2) +
           id(2);
}

void SlidingMap::posToGlobalIndex(const Vec3f &pos, Vec3i &id) const {

#ifdef ORIGIN_AT_CENTER
    id = (sc_.resolution_inv * pos + pos.cwiseSign() * 0.5).cast<int>();
#endif

#ifdef ORIGIN_AT_CORNER
    id = (pos.array() * sc_.resolution_inv).floor().cast<int>();
#endif
}

void SlidingMap::posToGlobalIndex(const double &pos, int &id) const {
#ifdef ORIGIN_AT_CENTER
    id = static_cast<int>((sc_.resolution_inv * pos + SIGN(pos) * 0.5));
#endif
#ifdef ORIGIN_AT_CORNER
    id = floor(pos * sc_.resolution_inv);
#endif
}

void SlidingMap::globalIndexToPos(const Vec3i &id_g, Vec3f &pos) const {

#ifdef ORIGIN_AT_CENTER
    pos = id_g.cast<double>() * sc_.resolution;
#endif
#ifdef ORIGIN_AT_CORNER
    pos = (id_g.cast<double>() + Vec3f(0.5, 0.5, 0.5)) * sc_.resolution;
#endif
}

void SlidingMap::globalIndexToLocalIndex(const Vec3i &id_g, Vec3i &id_l) const {
    for (int i = 0; i < 3; ++i) {
        /// [eq. (7) in paper] Compute the i_k
        id_l(i) = id_g(i) % sc_.map_size_i(i);
        /// [eq. (8) in paper] Normalize the local index
        if (id_l(i) > sc_.half_map_size_i(i)) {
            id_l(i) -= sc_.map_size_i(i);
        } else if (id_l(i) < -sc_.half_map_size_i(i)) {
            id_l(i) += sc_.map_size_i(i);
        }
//        id_l(i) += id_l(i) > sc_.half_map_size_i(i) ? -sc_.map_size_i(i) : 0;
//        id_l(i) += id_l(i) < -sc_.half_map_size_i(i) ? sc_.map_size_i(i) : 0;
    }
}

void SlidingMap::localIndexToGlobalIndex(const Vec3i &id_l, Vec3i &id_g) const {
    for (int i = 0; i < 3; ++i) {
        int min_id_g = -sc_.half_map_size_i(i) + local_map_origin_i_(i);
        int min_id_l = min_id_g % sc_.map_size_i(i);
        min_id_l -= min_id_l > sc_.half_map_size_i(i) ? sc_.map_size_i(i) : 0;
        min_id_l += min_id_l < -sc_.half_map_size_i(i) ? sc_.map_size_i(i) : 0;
        int cur_dis_to_min_id = id_l(i) - min_id_l;
        cur_dis_to_min_id =
                (cur_dis_to_min_id) < 0 ? (sc_.map_size_i(i) + cur_dis_to_min_id) : cur_dis_to_min_id;
        int cur_id = cur_dis_to_min_id + min_id_g;
        id_g(i) = cur_id;
    }
}

void SlidingMap::localIndexToPos(const Vec3i &id_l, Vec3f &pos) const {
#ifdef ORIGIN_AT_CENTER
    for (int i = 0; i < 3; ++i) {
        int min_id_g = -sc_.half_map_size_i(i) + local_map_origin_i_(i);

        int min_id_l = min_id_g % sc_.map_size_i(i);
        min_id_l -= min_id_l > sc_.half_map_size_i(i) ? sc_.map_size_i(i) : 0;
        min_id_l += min_id_l < -sc_.half_map_size_i(i) ? sc_.map_size_i(i) : 0;

        int cur_dis_to_min_id = id_l(i) - min_id_l;
        cur_dis_to_min_id =
                (cur_dis_to_min_id) < 0 ? (sc_.map_size_i(i) + cur_dis_to_min_id) : cur_dis_to_min_id;
        int cur_id = cur_dis_to_min_id + min_id_g;
        pos(i) = cur_id * sc_.resolution;
    }
#endif

#ifdef ORIGIN_AT_CORNER
    for (int i = 0; i < 3; ++i) {
        int min_id_g = -sc_.half_map_size_i(i) + local_map_origin_i_(i);

        int min_id_l = min_id_g % sc_.map_size_i(i);
        min_id_l -= min_id_l > sc_.half_map_size_i(i) ? sc_.map_size_i(i) : 0;
        min_id_l += min_id_l < -sc_.half_map_size_i(i) ? sc_.map_size_i(i) : 0;

        int cur_dis_to_min_id = id_l(i) - min_id_l;
        cur_dis_to_min_id =
                (cur_dis_to_min_id) < 0 ? (sc_.map_size_i(i) + cur_dis_to_min_id) : cur_dis_to_min_id;
        int cur_id = cur_dis_to_min_id + min_id_g;
        pos(i) = (cur_id + 0.5) * sc_.resolution;
    }
#endif

}

void SlidingMap::hashIdToLocalIndex(const int &hash_id, Vec3i &id) const {
    id(0) = hash_id / (sc_.map_size_i(1) * sc_.map_size_i(2));
    id(1) = (hash_id - id(0) * sc_.map_size_i(1) * sc_.map_size_i(2)) / sc_.map_size_i(2);
    id(2) = hash_id - id(0) * sc_.map_size_i(1) * sc_.map_size_i(2) - id(1) * sc_.map_size_i(2);
    id -= sc_.half_map_size_i;
}

void SlidingMap::hashIdToGlobalIndex(const int &hash_id, rog_map::Vec3i &id_g) const {
    Vec3i id;
    id(0) = hash_id / (sc_.map_size_i(1) * sc_.map_size_i(2));
    id(1) = (hash_id - id(0) * sc_.map_size_i(1) * sc_.map_size_i(2)) / sc_.map_size_i(2);
    id(2) = hash_id - id(0) * sc_.map_size_i(1) * sc_.map_size_i(2) - id(1) * sc_.map_size_i(2);
    id -= sc_.half_map_size_i;
    localIndexToGlobalIndex(id, id_g);
}

void SlidingMap::hashIdToPos(const int &hash_id, Vec3f &pos) const {
    Vec3i id;
    hashIdToLocalIndex(hash_id, id);
    localIndexToPos(id, pos);
}

int SlidingMap::getHashIndexFromPos(const Vec3f &pos) const {
    Vec3i id_g, id_l;
    posToGlobalIndex(pos, id_g);
    globalIndexToLocalIndex(id_g, id_l);
    return getLocalIndexHash(id_l);
}

int SlidingMap::getHashIndexFromGlobalIndex(const Vec3i &id_g) const {
    Vec3i id_l;
    globalIndexToLocalIndex(id_g, id_l);
    return getLocalIndexHash(id_l);
}