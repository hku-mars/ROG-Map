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

#include <rog_map/prob_map.h>
using namespace rog_map;

void ProbMap::initProbMap() {
    static bool init_once{false};
    if(init_once) {
        throw std::runtime_error(" -- [ROGMap] ProbMap can only init once.");
    }
    init_once = true;
    initSlidingMap(cfg_.half_map_size_i, cfg_.resolution,
                                                      cfg_.map_sliding_en, cfg_.map_sliding_thresh,
                                                      cfg_.fix_map_origin);
    time_consuming_.resize(7);
    inf_map_ = std::make_shared<InfMap>(cfg_);


    if (cfg_.frontier_extraction_en) {
        fcnt_map_ = std::make_shared<FreeCntMap>(cfg_.half_map_size_i + Vec3i::Constant(2),
                                       cfg_.resolution,
                                       cfg_.map_sliding_en,
                                       cfg_.map_sliding_thresh,
                                       cfg_.fix_map_origin);
    }

    if (cfg_.esdf_en) {
        esdf_map_ = std::make_shared<ESDFMap>();
        esdf_map_->initESDFMap(cfg_.half_map_size_i,
                               cfg_.resolution,
                               cfg_.esdf_resolution,
                               cfg_.esdf_local_update_box,
                               cfg_.map_sliding_en,
                               cfg_.map_sliding_thresh,
                               cfg_.fix_map_origin,
                               cfg_.unk_thresh);
    }


    posToGlobalIndex(cfg_.visualization_range, sc_.visualization_range_i);
    posToGlobalIndex(cfg_.virtual_ceil_height, sc_.virtual_ceil_height_id_g);
    posToGlobalIndex(cfg_.virtual_ground_height, sc_.virtual_ground_height_id_g);

    cfg_.virtual_ceil_height = sc_.virtual_ceil_height_id_g * cfg_.resolution;
    cfg_.virtual_ground_height = sc_.virtual_ground_height_id_g * cfg_.resolution;

    if (!cfg_.map_sliding_en) {
        cout << YELLOW << " -- [ProbMap] Map sliding disabled, set origin to [" << cfg_.fix_map_origin.transpose()
            << "] -- " << RESET << endl;
        slideAllMap(cfg_.fix_map_origin);
    }


    int map_size = sc_.map_size_i.prod();


    occupancy_buffer_.resize(map_size, 0);
    raycast_data_.raycaster.setResolution(cfg_.resolution);
    raycast_data_.operation_cnt.resize(map_size, 0);
    raycast_data_.hit_cnt.resize(map_size, 0);

    resetLocalMap();

    cout << GREEN << " -- [ProbMap] Init successfully -- ." << RESET << endl;
    printMapInformation();
}

Vec3f ProbMap::getLocalMapOrigin() const {
    return local_map_origin_d_;
}

Vec3f ProbMap::getLocalMapSize() const {
    return cfg_.map_size_d;
}

// Query====================================================
bool ProbMap::isOccupied(const Vec3f& pos) const {
    if (!insideLocalMap(pos)) {
        return false;
    }
    if (pos.z() > cfg_.virtual_ceil_height ||
        pos.z() < cfg_.virtual_ground_height) {
        return true;
    }
    return isOccupied(occupancy_buffer_[getHashIndexFromPos(pos)]);
}

bool ProbMap::isUnknown(const Vec3f& pos) const {
    if (!insideLocalMap(pos)) {
        return true;
    }
    if (pos.z() > cfg_.virtual_ceil_height ||
        pos.z() < cfg_.virtual_ground_height) {
        return false;
    }

    return isUnknown(occupancy_buffer_[getHashIndexFromPos(pos)]);
}

bool ProbMap::isKnownFree(const Vec3f& pos) const {
    if (!insideLocalMap(pos)) {
        return false;
    }
    if (pos.z() > cfg_.virtual_ceil_height ||
        pos.z() < cfg_.virtual_ground_height) {
        return false;
    }
    return isKnownFree(occupancy_buffer_[getHashIndexFromPos(pos)]);
}

bool ProbMap::isFrontier(const Vec3f& pos) const {
    // 1) Check local map
    if (!insideLocalMap(pos)) {
        return false;
    }

    // 2) Check virtual ceil and ground
    if (pos.z() > cfg_.virtual_ceil_height ||
        pos.z() < cfg_.virtual_ground_height) {
        return false;
    }

    // 3) Check frontier
    if (isUnknown(occupancy_buffer_[getHashIndexFromPos(pos)])) {
        if (fcnt_map_->getFreeCnt(pos)) {
            return true;
        }
    }
    return false;
}

// ======================================================
bool ProbMap::isOccupied(const Vec3i& id_g) const {
    if (!insideLocalMap(id_g)) {
        return false;
    }
    if (id_g.z() > sc_.virtual_ceil_height_id_g ||
        id_g.z() < sc_.virtual_ground_height_id_g ) {
        return true;
    }
    return isOccupied(occupancy_buffer_[getHashIndexFromGlobalIndex(id_g)]);
}

bool ProbMap::isUnknown(const Vec3i& id_g) const {
    if (!insideLocalMap(id_g)) {
        return true;
    }
    if (id_g.z() > sc_.virtual_ceil_height_id_g ||
        id_g.z() < sc_.virtual_ground_height_id_g ) {
        return false;
    }
    return isUnknown(occupancy_buffer_[getHashIndexFromGlobalIndex(id_g)]);
}

bool ProbMap::isKnownFree(const Vec3i& id_g) const {
    if (!insideLocalMap(id_g)) {
        return false;
    }
    if (id_g.z() > sc_.virtual_ceil_height_id_g ||
        id_g.z() < sc_.virtual_ground_height_id_g ) {
        return true;
    }
    return isKnownFree(occupancy_buffer_[getHashIndexFromGlobalIndex(id_g)]);
}

bool ProbMap::isFrontier(const Vec3i& id_g) const {
    // 1) Check local map
    if (!insideLocalMap(id_g)) {
        return false;
    }
    if (id_g.z() > sc_.virtual_ceil_height_id_g ||
        id_g.z() < sc_.virtual_ground_height_id_g ) {
        return false;
    }

    if (isUnknown(occupancy_buffer_[getHashIndexFromGlobalIndex(id_g)])) {
        if (fcnt_map_->getFreeCnt(id_g) > 0) {
            return true;
        }
    }
    return false;
}

bool ProbMap::isKnownFreeInflate(const Vec3f& pos) const {
    return inf_map_->isKnownFreeInflate(pos);
}


bool ProbMap::isOccupiedInflate(const Vec3f& pos) const {
    return inf_map_->isOccupiedInflate(pos);
}

bool ProbMap::isUnknownInflate(const Vec3f& pos) const {
    return inf_map_->isUnknownInflate(pos);
}

// Query====================================================


void ProbMap::writeTimeConsumingToLog(std::ofstream& log_file) {
    for (long unsigned int i = 0; i < time_consuming_.size(); i++) {
        log_file << time_consuming_[i];
        if (i != time_consuming_.size() - 1)
            log_file << ", ";
    }
    log_file << endl;
}

void ProbMap::writeMapInfoToLog(std::ofstream& log_file) {
    log_file << "[ProbMap]" << endl;
    log_file << "\tmap_size_d: " << cfg_.map_size_d.transpose() << endl;
    log_file << "\tresolution: " << cfg_.resolution << endl;
    log_file << "\tmap_size_i: " << sc_.map_size_i.transpose() << endl;
    log_file << "\tlocal_update_box_size: " << cfg_.local_update_box_d.transpose() << endl;
    log_file << "\tp_min: " << cfg_.p_min << endl;
    log_file << "\tp_max: " << cfg_.p_max << endl;
    log_file << "\tp_hit: " << cfg_.p_hit << endl;
    log_file << "\tp_miss: " << cfg_.p_miss << endl;
    log_file << "\tp_occ: " << cfg_.p_occ << endl;
    log_file << "\tp_free: " << cfg_.p_free << endl;
    log_file << "\tunk_thresh: " << cfg_.unk_thresh << endl;
    log_file << "\tmap_sliding_thresh: " << cfg_.map_sliding_thresh << endl;
    log_file << "\tmap_sliding_en: " << cfg_.map_sliding_en << endl;
    log_file << "\tfix_map_origin: " << cfg_.fix_map_origin.transpose() << endl;
    log_file << "\tvisualization_range: " << cfg_.visualization_range.transpose() << endl;
    log_file << "\tvirtual_ceil_height: " << cfg_.virtual_ceil_height << endl;
    log_file << "\tvirtual_ground_height: " << cfg_.virtual_ground_height << endl;
    log_file << "\tbatch_update_size: " << cfg_.batch_update_size << endl;
    log_file << "\tfrontier_extraction_en: " << cfg_.frontier_extraction_en << endl;
    inf_map_->writeMapInfoToLog(log_file);
}

void ProbMap::updateOccPointCloud(const PointCloud& input_cloud) {
    /// Step 1; Raycast and add to update cache.
    const int cloud_in_size = input_cloud.size();
    Vec3f localmap_min = local_map_bound_min_d_;
    Vec3f localmap_max = local_map_bound_max_d_;
    for (int i = 0; i < cloud_in_size; i++) {
        static Vec3f p, ray_pt;
        static Vec3i pt_id_g, pt_id_l;
        if (cfg_.intensity_thresh > 0 &&
            input_cloud[i].intensity < cfg_.intensity_thresh) {
            continue;
        }

        p.x() = input_cloud[i].x;
        p.y() = input_cloud[i].y;
        p.z() = input_cloud[i].z;

        posToGlobalIndex(p, pt_id_g);

        if (p.z() > cfg_.virtual_ceil_height || p.z() < cfg_.virtual_ground_height) {
            continue;
        }
        if (insideLocalMap(pt_id_g)) {
            const int occ_hit_num = ceil(cfg_.l_occ / cfg_.l_hit);
            for (int j = 0; j < occ_hit_num; j++) {
                insertUpdateCandidate(pt_id_g, true);
            }
            localmap_max = localmap_max.cwiseMax(p);
            localmap_min = localmap_min.cwiseMin(p);
        }
    }
    local_map_bound_min_d_ = localmap_min;
    local_map_bound_max_d_ = localmap_max;
    probabilisticMapFromCache();
    map_empty_ = false;
}

void ProbMap::slideAllMap(const rog_map::Vec3f& pos) {
    mapSliding(pos);
    inf_map_->mapSliding(pos);
    if (cfg_.frontier_extraction_en) {
        fcnt_map_->mapSliding(pos);
    }
    if (cfg_.esdf_en) {
        esdf_map_->mapSliding(pos);
    }
}

void ProbMap::updateProbMap(const PointCloud& cloud, const Pose& pose) {
    TimeConsuming tc("updateMap", false);
    const Vec3f& pos = pose.first;
    time_consuming_[4] = cloud.size();
    if (cfg_.map_sliding_en && !insideLocalMap(pos) && raycast_data_.batch_update_counter == 0) {
        cout << YELLOW << " -- [ROGMapCore] cur_pose out of map range, reset the map." << RESET << endl;
        cout << YELLOW << " -- [ROGMapCore] Sliding to map center at: " << pos.transpose() << RESET << endl;
        slideAllMap(pos);
        return;
    }

    if (pos.z() > cfg_.virtual_ceil_height) {
        cout << RED << " -- [ROGMapCore] Odom above virtual ceil, please check map parameter -- ." << RESET
            << endl;
        return;
    }
    else if (pos.z() < cfg_.virtual_ground_height) {
        cout << RED << " -- [ROGMapCore] Odom below virtual ground, please check map parameter -- ." << RESET
            << endl;
        return;
    }

    if (raycast_data_.batch_update_counter == 0
        && (map_empty_ ||
            (cfg_.map_sliding_en && (pos - local_map_origin_d_).norm() > cfg_.map_sliding_thresh))) {
        slideAllMap(pos);
    }

    updateLocalBox(pos);
    TimeConsuming t_raycast("raycast", false);
    raycastProcess(cloud, pos);
    time_consuming_[1] = t_raycast.stop();
    raycast_data_.batch_update_counter++;
    if (raycast_data_.batch_update_counter >= cfg_.batch_update_size) {
        raycast_data_.batch_update_counter = 0;
        time_consuming_[5] = raycast_data_.update_cache_id_g.size();
        TimeConsuming t_update("update", false);
        probabilisticMapFromCache();
        time_consuming_[2] = t_update.stop();
        map_empty_ = false;
    }
    inf_map_->getInflationNumAndTime(time_consuming_[6], time_consuming_[3]);
    time_consuming_[0] = tc.stop();

    /* Update ESDF map */
    if (cfg_.esdf_en) {
        esdf_map_->updateESDF3D(pos);
    }

    /* For the first frame, clear all unknown around the robot */
    static bool first = true;
    if (first) {
        first = false;
        for (double dx = -cfg_.raycast_range_min; dx <= cfg_.raycast_range_min; dx += cfg_.resolution) {
            for (double dy = -cfg_.raycast_range_min; dy <= cfg_.raycast_range_min; dy += cfg_.resolution) {
                for (double dz = -cfg_.raycast_range_min; dz <= cfg_.raycast_range_min; dz += cfg_.resolution) {
                    Vec3f p(dx, dy, dz);
                    if (p.norm() <= cfg_.raycast_range_min) {
                        Vec3f pp = pos + p;
                        int hash_id = getHashIndexFromPos(pp);
                        missPointUpdate(pp, hash_id, 999);
                    }
                }
            }
        }
    }
}

GridType ProbMap::getGridType(Vec3i& id_g) const {
    if (id_g.z() <= sc_.virtual_ground_height_id_g ||
        id_g.z() >= sc_.virtual_ceil_height_id_g) {
        return OCCUPIED;
    }
    if (!insideLocalMap(id_g)) {
        return OUT_OF_MAP;
    }
    Vec3i id_l;
    globalIndexToLocalIndex(id_g, id_l);
    int hash_id = getLocalIndexHash(id_l);
    double ret = occupancy_buffer_[hash_id];
    if (isKnownFree(ret)) {
        return GridType::KNOWN_FREE;
    }
    else if (isOccupied(ret)) {
        return GridType::OCCUPIED;
    }
    else {
        return GridType::UNKNOWN;
    }
}

GridType ProbMap::getGridType(const Vec3f& pos) const {
    if (pos.z() <= cfg_.virtual_ground_height ||
        pos.z() >= cfg_.virtual_ceil_height) {
        return OCCUPIED;
    }
    if (!insideLocalMap(pos)) {
        return OUT_OF_MAP;
    }
    Vec3i id_g, id_l;
    posToGlobalIndex(pos, id_g);
    return getGridType(id_g);
}

GridType ProbMap::getInfGridType(const Vec3f& pos) const {
    return inf_map_->getGridType(pos);
}

double ProbMap::getMapValue(const Vec3f& pos) const {
    if (!insideLocalMap(pos)) {
        return 0;
    }
    return occupancy_buffer_[getHashIndexFromPos(pos)];
}

void
ProbMap::boxSearch(const Vec3f& _box_min, const Vec3f& _box_max, const GridType& gt, vec_E<Vec3f>& out_points) const {
    out_points.clear();
    if (map_empty_) {
        cout << YELLOW << " -- [ROG] Map is empty, cannot perform box search." << RESET << endl;
        return;
    }
    if ((_box_max - _box_min).minCoeff() <= 0) {
        cout << YELLOW << " -- [ROG] Box search failed, box size is zero." << RESET << endl;
        return;
    }
    Vec3f box_min_d = _box_min, box_max_d = _box_max;
    boundBoxByLocalMap(box_min_d, box_max_d);
    if ((box_max_d - box_min_d).minCoeff() <= 0) {
        cout << YELLOW << " -- [ROG] Box search failed, box size is zero." << RESET << endl;
        return;
    }
    Vec3i box_min_id_g, box_max_id_g;
    posToGlobalIndex(box_min_d, box_min_id_g);
    posToGlobalIndex(box_max_d, box_max_id_g);
    Vec3i box_size = box_max_id_g - box_min_id_g;
    if (gt == UNKNOWN) {
        out_points.reserve(box_size.prod());
        for (int i = box_min_id_g.x() + 1; i < box_max_id_g.x(); i++) {
            for (int j = box_min_id_g.y() + 1; j < box_max_id_g.y(); j++) {
                for (int k = box_min_id_g.z() + 1; k < box_max_id_g.z(); k++) {
                    Vec3i id_g(i, j, k);
                    if (isUnknown(id_g)) {
                        Vec3f pos;
                        globalIndexToPos(id_g, pos);
                        out_points.push_back(pos);
                    }
                }
            }
        }
    }
    else if (gt == OCCUPIED) {
        out_points.reserve(box_size.prod() / 3);
        for (int i = box_min_id_g.x() + 1; i < box_max_id_g.x(); i++) {
            for (int j = box_min_id_g.y() + 1; j < box_max_id_g.y(); j++) {
                for (int k = box_min_id_g.z() + 1; k < box_max_id_g.z(); k++) {
                    Vec3i id_g(i, j, k);
                    if (isOccupied(id_g)) {
                        Vec3f pos;
                        globalIndexToPos(id_g, pos);
                        out_points.push_back(pos);
                    }
                }
            }
        }
    }
    else if (gt == FRONTIER) {
        out_points.reserve(box_size.prod() / 3);
        for (int i = box_min_id_g.x() + 1; i < box_max_id_g.x(); i++) {
            for (int j = box_min_id_g.y() + 1; j < box_max_id_g.y(); j++) {
                for (int k = box_min_id_g.z() + 1; k < box_max_id_g.z(); k++) {
                    Vec3i id_g(i, j, k);
                    if (isFrontier(id_g)) {
                        Vec3f pos;
                        globalIndexToPos(id_g, pos);
                        out_points.push_back(pos);
                    }
                }
            }
        }
    }
    else {
        throw std::runtime_error(" -- [ROG-Map] Box search does not support KNOWN_FREE.");
    }
}

void ProbMap::boxSearchInflate(const Vec3f& box_min, const Vec3f& box_max, const GridType& gt,
                               vec_E<Vec3f>& out_points) const {
    inf_map_->boxSearch(box_min, box_max, gt, out_points);
}

void ProbMap::boundBoxByLocalMap(Vec3f& box_min, Vec3f& box_max) const {
    if ((box_max - box_min).minCoeff() <= 0) {
        box_min = box_max;
        cout << RED << "-- [ROG] Bound box is invalid." << RESET << endl;
        return;
    }

    box_min = box_min.cwiseMax(local_map_bound_min_d_);
    box_max = box_max.cwiseMin(local_map_bound_max_d_);
    box_max.z() = std::min(box_max.z(), cfg_.virtual_ceil_height);
    box_min.z() = std::max(box_min.z(), cfg_.virtual_ground_height);
}

void ProbMap::resetCell(const int& hash_id) {
    float& ret = occupancy_buffer_[hash_id];
    if (isOccupied(ret)) {
        /// if current state is occupied
        Vec3f pos;
        hashIdToPos(hash_id, pos);
        inf_map_->updateGridCounter(pos, OCCUPIED, UNKNOWN);
        if (cfg_.esdf_en) {
            esdf_map_->updateGridCounter(pos, OCCUPIED, UNKNOWN);
        }
    }
    else if (isKnownFree(ret)) {
        /// if current state is free
        Vec3f pos;
        hashIdToPos(hash_id, pos);
        inf_map_->updateGridCounter(pos, KNOWN_FREE, UNKNOWN);
        if (cfg_.esdf_en) {
            esdf_map_->updateGridCounter(pos, KNOWN_FREE, UNKNOWN);
        }
        if (cfg_.frontier_extraction_en) {
            Vec3i id_g;
            posToGlobalIndex(pos, id_g);
            if (cfg_.frontier_extraction_en) {
                fcnt_map_->updateFrontierCounter(id_g, false);
            }
        }
    }
    else {
        // nothing need to do
    }
    ret = 0;
}

void ProbMap::probabilisticMapFromCache() {
    //    int addr = getHashIndexFromGlobalIndex(Vec3i(41,
    //                                                 -216,
    //                                                 -6));
    //    float ret = occupancy_buffer_[addr];
    //    std::cout << "ret: " << ret << std::endl;
    while (!raycast_data_.update_cache_id_g.empty()) {
        Vec3f pos;
        Vec3i id_g = raycast_data_.update_cache_id_g.front();
        raycast_data_.update_cache_id_g.pop();
        Vec3i id_l;
        globalIndexToLocalIndex(id_g, id_l);
        int hash_id = getLocalIndexHash(id_l);
        globalIndexToPos(id_g, pos);
        if (raycast_data_.hit_cnt[hash_id] > 0) {
            hitPointUpdate(pos, hash_id, raycast_data_.hit_cnt[hash_id]);
        }
        else {
            missPointUpdate(pos, hash_id,
                            raycast_data_.operation_cnt[hash_id] - raycast_data_.hit_cnt[hash_id]);
        }
        raycast_data_.hit_cnt[hash_id] = 0;
        raycast_data_.operation_cnt[hash_id] = 0;
    }
}

void ProbMap::hitPointUpdate(const Vec3f& pos, const int& hash_id, const int& hit_num) {
    float& ret = occupancy_buffer_[hash_id];
    GridType from_type = UNDEFINED;

    if (isOccupied(ret)) {
        from_type = GridType::OCCUPIED;
    }
    else if (isKnownFree(ret)) {
        from_type = GridType::KNOWN_FREE;
    }
    else {
        from_type = GridType::UNKNOWN;
    }


    ret += cfg_.l_hit * hit_num;
    if (ret > cfg_.l_max) {
        ret = cfg_.l_max;
    }

    GridType to_type;
    if (isOccupied(ret)) {
        to_type = GridType::OCCUPIED;
    }
    else if (isKnownFree(ret)) {
        to_type = GridType::KNOWN_FREE;
    }
    else {
        to_type = GridType::UNKNOWN;
    }

    if (from_type != to_type) {
        Vec3f center_pos;
        Vec3i id_g;
        posToGlobalIndex(pos, id_g);
        globalIndexToPos(id_g, center_pos);
        inf_map_->updateGridCounter(center_pos, from_type, to_type);
        if (cfg_.esdf_en) {
            esdf_map_->updateGridCounter(center_pos, from_type, to_type);
        }
        if (cfg_.frontier_extraction_en && from_type == KNOWN_FREE) {
            Vec3i id_g;
            posToGlobalIndex(pos, id_g);
            fcnt_map_->updateFrontierCounter(id_g, false);
        }
    }
}

void ProbMap::missPointUpdate(const Vec3f& pos, const int& hash_id, const int& hit_num) {
    float& ret = occupancy_buffer_[hash_id];
    GridType from_type;
    if (isOccupied(ret)) {
        from_type = GridType::OCCUPIED;
    }
    else if (isKnownFree(ret)) {
        from_type = GridType::KNOWN_FREE;
    }
    else {
        from_type = GridType::UNKNOWN;
    }
    ret += cfg_.l_miss * hit_num;
    if (ret < cfg_.l_min) {
        ret = cfg_.l_min;
    }

    GridType to_type;
    if (isOccupied(ret)) {
        to_type = GridType::OCCUPIED;
    }
    else if (isKnownFree(ret)) {
        to_type = GridType::KNOWN_FREE;
    }
    else {
        to_type = GridType::UNKNOWN;
    }
    // Catch the jump edge
    if (from_type != to_type) {
        Vec3f center_pos;
        Vec3i id_g;
        posToGlobalIndex(pos, id_g);
        globalIndexToPos(id_g, center_pos);
        // Update inf map
        inf_map_->updateGridCounter(center_pos, from_type, to_type);
        if (cfg_.esdf_en) {
            esdf_map_->updateGridCounter(center_pos, from_type, to_type);
        }


        if (cfg_.frontier_extraction_en && to_type == KNOWN_FREE) {
            Vec3i id_g;
            posToGlobalIndex(pos, id_g);
            fcnt_map_->updateFrontierCounter(id_g, true);
        }
    }
}

void ProbMap::raycastProcess(const PointCloud& input_cloud, const Vec3f& cur_odom) {
    // bounding box of updated region
    raycast_data_.cache_box_min = cur_odom;
    raycast_data_.cache_box_max = cur_odom;
    Vec3f raycast_box_min, raycast_box_max;

    {
        std::lock_guard<std::mutex> lck{raycast_data_.raycast_range_mtx};
        raycast_box_max = raycast_data_.local_update_box_max;
        raycast_box_min = raycast_data_.local_update_box_min;
    }

    /// Step 1; Raycast and add to update cache.
    const int& cloud_in_size = input_cloud.size();
    // new version of raycasting process
    auto raycasting_cloud = vec_Vec3f{};
    raycasting_cloud.reserve(cloud_in_size);

    // 1) process all non-inf points, update occupied probability
    int temperol_cnt{0};
    for (const auto& pcl_p : input_cloud) {
        // 1.1) intensity filter
        if (cfg_.intensity_thresh > 0 &&
            pcl_p.intensity < cfg_.intensity_thresh) {
            continue;
        }

        // 1.2) temporal filter
        if (temperol_cnt++ % cfg_.point_filt_num) {
            continue;
        }

        Vec3f p(pcl_p.x, pcl_p.y, pcl_p.z);
        Vec3i pt_id_g;

        // no raycasting, purely add occ pints
        if (!cfg_.raycasting_en) {
            if(insideLocalMap(p)) {
                posToGlobalIndex(p, pt_id_g);
                insertUpdateCandidate(pt_id_g, true);
                // record cache box size;
                raycast_data_.cache_box_min = raycast_data_.cache_box_min.cwiseMin(p);
                raycast_data_.cache_box_max = raycast_data_.cache_box_max.cwiseMax(p);
            }
            continue;
        }

        bool update_hit{true};
        // 1.3) filter for virtual ceil and ground
        if (p.z() > cfg_.virtual_ceil_height ) {
            update_hit = false;
            // find the intersect point with the ceil
            const double dz = p.z() - cur_odom.z();
            const double pc = cfg_.virtual_ceil_height - cur_odom.z();
            p = cur_odom + (p - cur_odom).normalized() * pc / dz;
        }else if (p.z() < cfg_.virtual_ground_height) {
            update_hit = false;
            // find the intersect point with the ground
            const double dz = p.z() - cur_odom.z();
            const double pc = cfg_.virtual_ground_height - cur_odom.z();
            p = cur_odom + (p - cur_odom).normalized() * pc / dz;
        }

        // 1.4) bounding box filter
        // raycasting max
        const double sqr_dis = (p - cur_odom).squaredNorm();
        if (sqr_dis > cfg_.sqr_raycast_range_max) {
            double k = cfg_.raycast_range_max / sqrt(sqr_dis);
            p = k * (p - cur_odom) + cur_odom;
            update_hit = false;
        }

        // local map bound
        if (((p - raycast_box_min).minCoeff() < 0) ||
            ((p - raycast_box_max).maxCoeff() > 0)) {
            p = lineBoxIntersectPoint(p,
                                      cur_odom,
                                      raycast_box_min,
                                      raycast_box_max);
            update_hit = false;
        }


        // record cache box size;
        raycast_data_.cache_box_min = raycast_data_.cache_box_min.cwiseMin(p);
        raycast_data_.cache_box_max = raycast_data_.cache_box_max.cwiseMax(p);

        // 1.4) for all validate hit points, update probability
        raycasting_cloud.push_back(p);

        if (update_hit) {
            posToGlobalIndex(p, pt_id_g);
            insertUpdateCandidate(pt_id_g, true);
        }
    }

    if(cfg_.raycasting_en) {
        // 4) process all inf points, updae free probability
        for (const auto& p : raycasting_cloud) {
            Vec3f raycast_start = (p - cur_odom).normalized() * cfg_.raycast_range_min + cur_odom;
            raycast_data_.raycaster.setInput(raycast_start, p);
            Vec3f ray_pt;
            while (raycast_data_.raycaster.step(ray_pt)) {
                Vec3i cur_ray_id_g;
                posToGlobalIndex(ray_pt, cur_ray_id_g);
                if (!insideLocalMap(cur_ray_id_g)) {
                    break;
                }
                insertUpdateCandidate(cur_ray_id_g, false);
            }
        }
    }

}

void ProbMap::insertUpdateCandidate(const Vec3i& id_g, bool is_hit) {
    const auto& hash_id = getHashIndexFromGlobalIndex(id_g);
    raycast_data_.operation_cnt[hash_id]++;
    if (raycast_data_.operation_cnt[hash_id] == 1) {
        raycast_data_.update_cache_id_g.push(id_g);
    }
    if (is_hit) {
        raycast_data_.hit_cnt[hash_id]++;
    }
}

void ProbMap::updateLocalBox(const Vec3f& cur_odom) {
    /* This function is only used for decide the local update range
     * and do not related to the map origin and map bound
     * the map origin and map bound is only update in [SlidingMap::mapSliding(const Vec3f &odom)]
     * */
    // The local map should be inside in index wise
    // update: the virtual floor and ceil should not influence the raycasting.
    // 2) local map size
    // The update box should follow odom.
    // The local map should follow current map center.
    std::lock_guard<std::mutex> lck(raycast_data_.raycast_range_mtx);

    Vec3i cur_odom_i;
    posToGlobalIndex(cur_odom, cur_odom_i);
    Vec3i local_updatebox_min_i, local_updatebox_max_i;

    if (cfg_.raycasting_en) {
        local_updatebox_max_i = cur_odom_i + cfg_.half_local_update_box_i;
        local_updatebox_min_i = cur_odom_i - cfg_.half_local_update_box_i;
    }

    globalIndexToPos(local_updatebox_min_i, raycast_data_.local_update_box_min);
    globalIndexToPos(local_updatebox_max_i, raycast_data_.local_update_box_max);

    // the local update box must inside the local map
    raycast_data_.local_update_box_max = raycast_data_.local_update_box_max.cwiseMin(
        local_map_bound_max_d_);
    raycast_data_.local_update_box_min = raycast_data_.local_update_box_min.cwiseMax(
        local_map_bound_min_d_);
}

void ProbMap::resetLocalMap() {
    std::cout << RED << " -- [Prob-Map] Clear all local map." << RESET << std::endl;
    // Clear local map
    std::fill(occupancy_buffer_.begin(), occupancy_buffer_.end(), 0);
    while (!raycast_data_.update_cache_id_g.empty()) {
        raycast_data_.update_cache_id_g.pop();
    }
    raycast_data_.batch_update_counter = 0;
    std::fill(raycast_data_.operation_cnt.begin(), raycast_data_.operation_cnt.end(), 0);
    std::fill(raycast_data_.hit_cnt.begin(), raycast_data_.hit_cnt.end(), 0);
}
