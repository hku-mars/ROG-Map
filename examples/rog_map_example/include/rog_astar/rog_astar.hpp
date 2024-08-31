//
// Created by yunfan on 8/27/24.
//

#ifndef SRC_ROG_ASTAR_HPP
#define SRC_ROG_ASTAR_HPP

#include "rog_map/rog_map.h"
#include "rog_astar/config.hpp"
#include "rog_astar/types.hpp"
#include "memory"
#include "utils/visualization.hpp"
#define PRINT_CURRENT_LOCATION() std::cout << "Function: " << __FUNCTION__ << ", Line: " << __LINE__ << std::endl;

namespace rog_astar {
    using Vec3f = Eigen::Vector3d;
    using std::shared_ptr;
    using std::cout;
    using std::endl;

    class AStar {
    private:
        rog_map::ROGMap::Ptr map_ptr_;
        ros::NodeHandle nh_;
        rog_astar::Config cfg_;
        std::shared_ptr<visualization::Visualization> vis_ptr_;
        vector<GridNodePtr> grid_node_buffer_;

        struct PlanSetting {
            Vec3f start;
            Vec3f goal;
            double max_time;
            bool use_inf_map;
            bool use_prob_map;
            bool unknown_as_occ;
            bool unknown_as_free;

            double resolution;
            double resolution_inv;
            rog_map::Vec3i local_map_center_id_g;
            rog_map::Vec3f local_map_center_d;
            double mission_rcv_WT{0};
            rog_map::Vec3f local_map_max_d, local_map_min_d;
            std::mutex mission_mtx;
        } ps_;

    public:
        typedef shared_ptr<AStar> Ptr;

        void getExampleStartGoal(Vec3f& start,
                                 Vec3f& goal) {
            start = cfg_.example_start;
            goal = cfg_.example_goal;
        }


        AStar(const ros::NodeHandle& nh,
              const rog_map::ROGMap::Ptr& map_ptr) : nh_(nh), cfg_(nh) {
            map_ptr_ = map_ptr;
            vis_ptr_ = std::make_shared<visualization::Visualization>(nh_);
            int map_buffer_size = cfg_.map_voxel_num(0) * cfg_.map_voxel_num(1) * cfg_.map_voxel_num(2);
            grid_node_buffer_.resize(map_buffer_size);
            for (size_t i = 0; i < grid_node_buffer_.size(); i++) {
                grid_node_buffer_[i] = new GridNode;
                grid_node_buffer_[i]->rounds = 0;
            }
        }

        ~AStar() {};

        bool pathSearch(const Vec3f& start,
                        const Vec3f& goal,
                        const double& max_time,
                        const int& flag) {
            if (!decodeFlage(flag)) {
                return false;
            }

            ps_.start = start;
            ps_.goal = goal;
            ps_.max_time = max_time;

            vis_ptr_->visualize_a_ball(ps_.start, 0.2, "start", visualization::Color::orange, 1);
            vis_ptr_->visualize_a_ball(ps_.goal, 0.2, "goal", visualization::Color::green, 1);

            if (!isStateValid(ps_.start)) {
                ROS_ERROR("[A*]: Start pos collide or out of bound");
                return false;
            }
            if (!isStateValid(ps_.goal)) {
                ROS_ERROR("[A*]: Goal pos collide or out of bound");
                return false;
            }

            if (ps_.use_prob_map) {
                ps_.resolution = map_ptr_->getResolution();
            }
            else {
                ps_.resolution = map_ptr_->getInfResolution();
            }

            ps_.resolution_inv = 1.0 / ps_.resolution;

            ps_.local_map_center_d = (ps_.start + ps_.goal) / 2;


            posToGlobalIndex(ps_.local_map_center_d, ps_.local_map_center_id_g);
            ps_.local_map_min_d = ps_.local_map_center_d - ps_.resolution * cfg_.map_size_i.cast<double>();
            ps_.local_map_max_d = ps_.local_map_center_d + ps_.resolution * cfg_.map_size_i.cast<double>();

            if (cfg_.visualize_process_en) {
                vis_ptr_->visualize_bounding_box(ps_.local_map_min_d,
                                                 ps_.local_map_max_d,
                                                 "astar_local_map",
                                                 visualization::Color::steelblue,
                                                 0.1);
            }

            ROS_INFO("[RRT*]: RRT starts planning a path");
            ROS_INFO("\tStart at: %f, %f, %f", ps_.start(0), ps_.start(1), ps_.start(2));
            ROS_INFO("\tGoal at: %f, %f, %f", ps_.goal(0), ps_.goal(1), ps_.goal(2));

            return astar_search();
        }

        bool runExample() {
            return pathSearch(cfg_.example_start, cfg_.example_goal,
                              1, ON_INF_MAP | UNKNOWN_AS_FREE);
        }

        rog_map::vec_Vec3f getPath() const {
            return final_path_;
        }

    private:
        rog_map::vec_Vec3f final_path_{};
        int rounds_{0};
        double tie_breaker_{1.0 + 1e-5};


        bool isStateValid(const Vec3f& pos) {
            if (ps_.unknown_as_free) {
                if (ps_.use_inf_map) {
                    return !map_ptr_->isOccupiedInflate(pos);
                }
                else if (ps_.use_prob_map) {
                    return !map_ptr_->isOccupied(pos);
                }
            }
            else if (ps_.unknown_as_occ) {
                if (ps_.use_inf_map) {
                    return (!map_ptr_->isUnknownInflate(pos) && !map_ptr_->isOccupiedInflate(pos));
                }
                else if (ps_.use_prob_map) {
                    return map_ptr_->isKnownFree(pos);
                }
            }
        }

        bool decodeFlage(const int& flag) {
            ps_.use_inf_map = (flag & rog_astar::ON_INF_MAP);
            ps_.use_prob_map = flag & rog_astar::ON_PROB_MAP;
            ps_.unknown_as_occ = flag & rog_astar::UNKNOWN_AS_OCCUPIED;
            ps_.unknown_as_free = flag & rog_astar::UNKNOWN_AS_FREE;
            if (ps_.use_inf_map && ps_.use_prob_map) {
                cout << rog_map::RED << " -- [A*]: cannot use both inf map and prob map."
                    << rog_map::RESET << endl;
                return false;
            }
            if (ps_.unknown_as_occ && ps_.unknown_as_free) {
                cout << rog_map::RED << " -- [A*]: cannot use both unknown_as_occupied and unknown_as_free."
                    << rog_map::RESET << endl;
                return false;
            }
            return true;
        }


        double getHeu(GridNodePtr node1, GridNodePtr node2, int type = DIAG) const {
            switch (type) {
            case DIAG: {
                double dx = std::abs(node1->id_g(0) - node2->id_g(0));
                double dy = std::abs(node1->id_g(1) - node2->id_g(1));
                double dz = std::abs(node1->id_g(2) - node2->id_g(2));

                double h = 0.0;
                int diag = std::min(std::min(dx, dy), dz);
                dx -= diag;
                dy -= diag;
                dz -= diag;

                if (dx == 0) {
                    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * std::min(dy, dz) + 1.0 * std::abs(dy - dz);
                }
                if (dy == 0) {
                    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * std::min(dx, dz) + 1.0 * std::abs(dx - dz);
                }
                if (dz == 0) {
                    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * std::min(dx, dy) + 1.0 * std::abs(dx - dy);
                }
                return tie_breaker_ * h;
            }
            case MANH: {
                double dx = std::abs(node1->id_g(0) - node2->id_g(0));
                double dy = std::abs(node1->id_g(1) - node2->id_g(1));
                double dz = std::abs(node1->id_g(2) - node2->id_g(2));

                return tie_breaker_ * (dx + dy + dz);
            }
            case EUCL: {
                return tie_breaker_ * (node2->id_g - node1->id_g).norm();
            }
            default: {
                ROS_ERROR(" -- [A*] Wrong hue type");
                return 0;
            }
            }
        }

        bool astar_search() {
            final_path_.clear();
            vis_ptr_->visualize_path(final_path_, "astar_final_path");
            ros::Time time_1 = ros::Time::now();
            ++rounds_;
            rog_map::GridType start_pt_type, end_pt_type;

            /// 2) Switch both start and end point to local map
            rog_map::Vec3i start_idx, end_idx;
            posToGlobalIndex(ps_.start, start_idx);
            posToGlobalIndex(ps_.goal, end_idx);

            if (!insideLocalMap(start_idx) || !insideLocalMap(end_idx)) {
                cout << rog_map::RED << " -- [RM] Start or end point is out of astar's map, which should not happen." <<
                    rog_map::RESET
                    << endl;
                return false;
            }

            GridNodePtr startPtr = grid_node_buffer_[getLocalIndexHash(start_idx)];
            GridNodePtr endPtr = grid_node_buffer_[getLocalIndexHash(end_idx)];
            endPtr->id_g = end_idx;

            std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> open_set;


            GridNodePtr neighborPtr = NULL;
            GridNodePtr current = NULL;

            startPtr->id_g = start_idx;
            startPtr->rounds = rounds_;
            startPtr->distance_score = 0;
            startPtr->total_score = getHeu(startPtr, endPtr, cfg_.heu_type);
            startPtr->state = GridNode::OPENSET; //put start node in open set
            startPtr->father_ptr = NULL;
            open_set.push(startPtr); //put start in open set
            int num_iter = 0;
            vector<GridNodePtr> node_path;

            rog_map::vec_Vec3f expand_points;

            while (!open_set.empty()) {
                num_iter++;
                current = open_set.top();
                open_set.pop();
                if (cfg_.visualize_process_en) {
                    rog_map::Vec3f local_pt;
                    globalIndexToPos(current->id_g, local_pt);
                    expand_points.push_back(local_pt);
                    if (expand_points.size() % 100 == 0) {
                        vis_ptr_->visualize_pointcloud(expand_points, "astar_expanded");
                        ros::Duration(0.01).sleep();
                    }
                }
                if (current->id_g(0) == endPtr->id_g(0) &&
                    current->id_g(1) == endPtr->id_g(1) &&
                    current->id_g(2) == endPtr->id_g(2)) {
                    retrievePath(current, node_path);
                    ConvertNodePathToPointPath(node_path, final_path_);
                    vis_ptr_->visualize_path(final_path_, "astar_final_path");
                    return true;
                }


                current->state = GridNode::CLOSEDSET; //move current node from open set to closed set.

                for (int dx = -1; dx <= 1; dx++)
                    for (int dy = -1; dy <= 1; dy++)
                        for (int dz = -1; dz <= 1; dz++) {
                            if (dx == 0 && dy == 0 && dz == 0) {
                                continue;
                            }
                            if (!cfg_.allow_diag &&
                                (std::abs(dx) + std::abs(dy) + std::abs(dz) > 1)) {
                                continue;
                            }

                            rog_map::Vec3i neighborIdx;
                            rog_map::Vec3f neighborPos;
                            neighborIdx(0) = (current->id_g)(0) + dx;
                            neighborIdx(1) = (current->id_g)(1) + dy;
                            neighborIdx(2) = (current->id_g)(2) + dz;
                            globalIndexToPos(neighborIdx, neighborPos);

                            if (!insideLocalMap(neighborIdx)) {
                                continue;
                            }

                            if (!isStateValid(neighborPos)) {
                                continue;
                            }


                            neighborPtr = grid_node_buffer_[getLocalIndexHash(neighborIdx)];
                            if (neighborPtr == nullptr) {
                                cout << rog_map::RED << " -- [RM] neighborPtr is null, which should not happen." <<
                                    rog_map::RESET
                                    << endl;
                                continue;
                            }

                            neighborPtr->id_g = neighborIdx;

                            bool flag_explored = neighborPtr->rounds == rounds_;

                            if (flag_explored && neighborPtr->state == GridNode::CLOSEDSET) {
                                continue; //in closed set.
                            }

                            neighborPtr->rounds = rounds_;
                            double distance_score = sqrt(dx * dx + dy * dy + dz * dz);
                            distance_score = current->distance_score + distance_score;
                            rog_map::Vec3f pos;
                            globalIndexToPos(neighborIdx, pos);
                            double heu_score = getHeu(neighborPtr, endPtr, cfg_.heu_type);

                            if (!flag_explored) {
                                //discover a new node
                                neighborPtr->state = GridNode::OPENSET;
                                neighborPtr->father_ptr = current;
                                neighborPtr->distance_score = distance_score;
                                neighborPtr->distance_to_goal = heu_score;
                                neighborPtr->total_score = distance_score + heu_score;
                                open_set.push(neighborPtr); //put neighbor in open set and record it.
                            }
                            else if (distance_score < neighborPtr->distance_score) {
                                neighborPtr->father_ptr = current;
                                neighborPtr->distance_score = distance_score;
                                neighborPtr->distance_to_goal = heu_score;
                                neighborPtr->total_score = distance_score + heu_score;
                            }
                        }
                ros::Time time_2 = ros::Time::now();
                if (!cfg_.visualize_process_en && (time_2 - time_1).toSec() > ps_.max_time) {
                    ROS_WARN("Failed in A star path searching !!! %lf seconds time limit exceeded.", ps_.max_time);
                    return false;
                }
            }
            ros::Time time_2 = ros::Time::now();
            if ((time_2 - time_1).toSec() > ps_.max_time) {
                ROS_WARN("Time consume in A star path finding is %.3fs, iter=%d", (time_2 - time_1).toSec(),
                         num_iter);
                return false;
            }

            cout << rog_map::RED << " -- [A*] Point to point path cannot find path with iter num: " << num_iter <<
                ", return."
                << rog_map::RESET << endl;
            return false;
        }

        void retrievePath(GridNodePtr current, vector<GridNodePtr>& path) {
            path.push_back(current);
            while (current->father_ptr != NULL) {
                current = current->father_ptr;
                path.push_back(current);
            }
        }


        void ConvertNodePathToPointPath(const vector<GridNodePtr>& node_path, rog_map::vec_Vec3f& point_path) {
            point_path.clear();
            for (auto ptr : node_path) {
                rog_map::Vec3f pos;
                globalIndexToPos(ptr->id_g, pos);
                point_path.push_back(pos);
            }
            reverse(point_path.begin(), point_path.end());
        }

#define SIGN(x) ((x > 0) - (x < 0))

        void posToGlobalIndex(const rog_map::Vec3f& pos, rog_map::Vec3i& id) const {
            // add resolution/2 for rounding
#ifdef ORIGIN_AT_CENTER
            id = (ps_.resolution_inv * pos + pos.cwiseSign() * 0.5).cast<int>();
#endif

#ifdef ORIGIN_AT_CORNER
            id = (pos.array() * ps_.resolution_inv).floor().cast<int>();
#endif
        }

        void globalIndexToPos(const rog_map::Vec3i& id_g, rog_map::Vec3f& pos) const {
#ifdef ORIGIN_AT_CENTER
            pos = id_g.cast<double>() * ps_.resolution;
#endif
#ifdef ORIGIN_AT_CORNER
            pos = (id_g.cast<double>() + Vec3f(0.5, 0.5, 0.5)) * ps_.resolution;
#endif
        }

        int getLocalIndexHash(const rog_map::Vec3i& id_in) const {
            rog_map::Vec3i id = id_in - ps_.local_map_center_id_g + cfg_.map_size_i;
            return id(0) * cfg_.map_voxel_num(1) * cfg_.map_voxel_num(2) +
                id(1) * cfg_.map_voxel_num(2) +
                id(2);
        }

        bool insideLocalMap(const rog_map::Vec3f& pos) const {
            rog_map::Vec3i id_g;
            posToGlobalIndex(pos, id_g);
            return insideLocalMap(id_g);
        }

        bool insideLocalMap(const rog_map::Vec3i& id_g) const {
            rog_map::Vec3i delta = id_g - ps_.local_map_center_id_g;
            if (fabs(delta.x()) > cfg_.map_size_i.x() ||
                fabs(delta.y()) > cfg_.map_size_i.y() ||
                fabs(delta.z()) > cfg_.map_size_i.z()) {
                return false;
            }
            return true;
        }
    };
}

#endif //SRC_ROG_ASTAR_HPP
