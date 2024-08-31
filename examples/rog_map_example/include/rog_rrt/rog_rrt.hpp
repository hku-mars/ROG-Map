/*
Copyright (C) 2022 Hongkai Ye (kyle_yeh@163.com)
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#ifndef SRC_ROG_RRT_HPP
#define SRC_ROG_RRT_HPP

#include "rog_map/rog_map.h"
#include "rog_rrt/config.hpp"
#include "utils/sampler.h"
#include "utils/node.h"
#include "utils/visualization.hpp"
#include "utils/kdtree.h"
#include "memory"
#define PRINT_CURRENT_LOCATION() std::cout << "Function: " << __FUNCTION__ << ", Line: " << __LINE__ << std::endl;

namespace rog_rrt {
    using Vec3f = Eigen::Vector3d;
    using std::shared_ptr;
    using std::cout;
    using std::endl;
    const int ON_INF_MAP = (1 << 0);
    const int ON_PROB_MAP = (1 << 1);
    const int UNKNOWN_AS_OCCUPIED = (1 << 2);
    const int UNKNOWN_AS_FREE = (1 << 3);
    const int USE_INF_NEIGHBOR = (1 << 4);

    class RRTStar {
    private:
        rog_map::ROGMap::Ptr map_ptr_;
        ros::NodeHandle nh_;
        rog_rrt::Config cfg_;

        BiasSampler sampler_;
        // for informed sampling
        Eigen::Vector3d trans_, scale_;
        Eigen::Matrix3d rot_;

        // for GUILD sampling
        Eigen::Vector3d scale1_, scale2_;
        Eigen::Vector3d trans1_, trans2_;
        Eigen::Matrix3d rot1_, rot2_;


        int valid_tree_node_nums_;
        double first_path_use_time_;
        double final_path_use_time_;

        std::vector<TreeNode*> nodes_pool_;
        TreeNode* start_node_;
        TreeNode* goal_node_;
        vector<Eigen::Vector3d> final_path_;
        vector<vector<Eigen::Vector3d>> path_list_;
        vector<std::pair<double, double>> solution_cost_time_pair_list_;

        std::shared_ptr<visualization::Visualization> vis_ptr_;

    public:
        typedef shared_ptr<RRTStar> Ptr;

        struct PlanSetting {
            Vec3f start;
            Vec3f goal;
            double max_time;
            bool use_inf_map;
            bool use_prob_map;
            bool unknown_as_occ;
            bool unknown_as_free;
        } ps_;

        void getExampleStartGoal(Vec3f& start,
                             Vec3f& goal) {
            start = cfg_.example_start;
            goal = cfg_.example_goal;
        }


        RRTStar(const ros::NodeHandle& nh,
                const rog_map::ROGMap::Ptr& map_ptr) : nh_(nh), cfg_(nh) {
            map_ptr_ = map_ptr;


            auto rog_map_range = map_ptr_->getLocalMapSize();
            auto rog_center = map_ptr_->getLocalMapOrigin() - rog_map_range / 2;

            sampler_.setSamplingRange(rog_center, rog_map_range);

            ROS_INFO("Set map origin: %f, %f, %f", map_ptr_->getLocalMapOrigin()(0), map_ptr_->getLocalMapOrigin()(1),
                     map_ptr_->getLocalMapOrigin()(2));
            ROS_INFO("Set map size: %f, %f, %f", map_ptr_->getLocalMapSize()(0), map_ptr_->getLocalMapSize()(1),
                     map_ptr_->getLocalMapSize()(2));

            valid_tree_node_nums_ = 0;
            nodes_pool_.resize(cfg_.max_tree_node_nums);
            for (int i = 0; i < cfg_.max_tree_node_nums; ++i) {
                nodes_pool_[i] = new TreeNode;
            }

            vis_ptr_ = std::make_shared<visualization::Visualization>(nh_);
        }

        ~RRTStar() {};

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

            reset();

            if (!isStateValid(ps_.start)) {
                ROS_ERROR("[RRT*]: Start pos collide or out of bound");
                return false;
            }
            if (!isStateValid(ps_.goal)) {
                ROS_ERROR("[RRT*]: Goal pos collide or out of bound");
                return false;
            }
            /* construct start and goal nodes */
            start_node_ = nodes_pool_[1];
            start_node_->x = ps_.start;
            start_node_->cost_from_start = 0.0;
            goal_node_ = nodes_pool_[0];
            goal_node_->x = ps_.goal;
            goal_node_->cost_from_start = DBL_MAX; // important
            valid_tree_node_nums_ = 2; // put start and goal in tree

            ROS_INFO("[RRT*]: RRT starts planning a path");
            ROS_INFO("\tStart at: %f, %f, %f", ps_.start(0), ps_.start(1), ps_.start(2));
            ROS_INFO("\tGoal at: %f, %f, %f", ps_.goal(0), ps_.goal(1), ps_.goal(2));

            sampler_.reset(); // !important
            if (cfg_.use_informed_sampling) {
                calInformedSet(10000000000.0, ps_.start, ps_.goal, scale_, trans_, rot_);
                sampler_.setInformedTransRot(trans_, rot_);
            }
            ROS_INFO("\tCompute informed set done.");
            return rrt_star(ps_.start, ps_.goal);
        }

        bool runExample() {
            return pathSearch(cfg_.example_start, cfg_.example_goal,
                       1, ON_INF_MAP | UNKNOWN_AS_FREE);
        }

        vector<Eigen::Vector3d> getPath() const {
            return final_path_;
        }

    private:
        bool decodeFlage(const int& flag) {
            ps_.use_inf_map = (flag & rog_rrt::ON_INF_MAP);
            ps_.use_prob_map = flag & rog_rrt::ON_PROB_MAP;
            ps_.unknown_as_occ = flag & rog_rrt::UNKNOWN_AS_OCCUPIED;
            ps_.unknown_as_free = flag & rog_rrt::UNKNOWN_AS_FREE;
            if (ps_.use_inf_map && ps_.use_prob_map) {
                cout << rog_map::RED << " -- [RRT*]: cannot use both inf map and prob map."
                    << rog_map::RESET << endl;
                return false;
            }
            if (ps_.unknown_as_occ && ps_.unknown_as_free) {
                cout << rog_map::RED << " -- [RRT*]: cannot use both unknown_as_occupied and unknown_as_free."
                    << rog_map::RESET << endl;
                return false;
            }
            return true;
        }

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

        bool isSegmentValid(const Vec3f& p1, const Vec3f& p2) {
            return map_ptr_->isLineFree(p1, p2,
                                        ps_.use_inf_map,
                                        ps_.unknown_as_occ);
        }

        void reset() {
            final_path_.clear();
            path_list_.clear();
            solution_cost_time_pair_list_.clear();
            for (int i = 0; i < valid_tree_node_nums_; i++) {
                nodes_pool_[i]->parent = nullptr;
                nodes_pool_[i]->children.clear();
            }
            valid_tree_node_nums_ = 0;
        }

        double calDist(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) {
            return (p1 - p2).norm();
        }

        Eigen::Vector3d steer(const Eigen::Vector3d& nearest_node_p, const Eigen::Vector3d& rand_node_p, double len) {
            Eigen::Vector3d diff_vec = rand_node_p - nearest_node_p;
            double dist = diff_vec.norm();
            if (diff_vec.norm() <= len)
                return rand_node_p;
            else
                return nearest_node_p + diff_vec * len / dist;
        }

        RRTNode3DPtr addTreeNode(RRTNode3DPtr& parent, const Eigen::Vector3d& state,
                                 const double& cost_from_start, const double& cost_from_parent) {
            RRTNode3DPtr new_node_ptr = nodes_pool_[valid_tree_node_nums_];
            valid_tree_node_nums_++;
            new_node_ptr->parent = parent;
            parent->children.push_back(new_node_ptr);
            new_node_ptr->x = state;
            new_node_ptr->cost_from_start = cost_from_start;
            new_node_ptr->cost_from_parent = cost_from_parent;
            return new_node_ptr;
        }

        void changeNodeParent(RRTNode3DPtr& node, RRTNode3DPtr& parent, const double& cost_from_parent) {
            if (node->parent)
                node->parent->children.remove(node); // DON'T FORGET THIS, remove it form its parent's children list
            node->parent = parent;
            node->cost_from_parent = cost_from_parent;
            node->cost_from_start = parent->cost_from_start + cost_from_parent;
            parent->children.push_back(node);

            // for all its descedants, change the cost_from_start and tau_from_start;
            RRTNode3DPtr descendant(node);
            std::queue<RRTNode3DPtr> Q;
            Q.push(descendant);
            while (!Q.empty()) {
                descendant = Q.front();
                Q.pop();
                for (const auto& leafptr : descendant->children) {
                    leafptr->cost_from_start = leafptr->cost_from_parent + descendant->cost_from_start;
                    Q.push(leafptr);
                }
            }
        }

        void fillPath(const RRTNode3DPtr& n, vector<Eigen::Vector3d>& path) {
            path.clear();
            RRTNode3DPtr node_ptr = n;
            while (node_ptr->parent) {
                path.push_back(node_ptr->x);
                node_ptr = node_ptr->parent;
            }
            path.push_back(start_node_->x);
            std::reverse(std::begin(path), std::end(path));
        }

        bool rrt_star(const Eigen::Vector3d& s, const Eigen::Vector3d& g) {
            ros::Time rrt_start_time = ros::Time::now();
            bool goal_found = false;
            double c_square = (g - s).squaredNorm() / 4.0;

            /* kd tree init */
            kdtree* kd_tree = kd_create(3);
            // Add start and goal nodes to kd tree
            kd_insert3(kd_tree, start_node_->x[0], start_node_->x[1], start_node_->x[2], start_node_);


            /* main loop */
            int idx = 0;
            for (idx = 0;
                 (cfg_.visualize_process_en
                      ? true
                      : (ros::Time::now() - rrt_start_time).toSec() < ps_.max_time) &&
                 valid_tree_node_nums_ < cfg_.max_tree_node_nums; ++idx) {
                /* biased random sampling */
                Eigen::Vector3d x_rand;
                sampler_.samplingOnce(x_rand);
                // samplingOnce(x_rand);
                if (!isStateValid(x_rand)) {
                    continue;
                }

                struct kdres* p_nearest = kd_nearest3(kd_tree, x_rand[0], x_rand[1], x_rand[2]);
                if (p_nearest == nullptr) {
                    ROS_ERROR("nearest query error");
                    continue;
                }
                RRTNode3DPtr nearest_node = (RRTNode3DPtr)kd_res_item_data(p_nearest);
                kd_res_free(p_nearest);

                Eigen::Vector3d x_new = steer(nearest_node->x, x_rand, cfg_.steer_length);
                if (!isSegmentValid(nearest_node->x, x_new)) {
                    continue;
                }

                /* 1. find parent */
                /* kd_tree bounds search for parent */
                Neighbour neighbour_nodes;
                neighbour_nodes.nearing_nodes.reserve(50);
                neighbour_nodes.center = x_new;
                struct kdres* nbr_set;
                nbr_set = kd_nearest_range3(kd_tree, x_new[0], x_new[1], x_new[2], cfg_.search_radius);
                if (nbr_set == nullptr) {
                    ROS_ERROR("bkwd kd range query error");
                    break;
                }

                while (!kd_res_end(nbr_set)) {
                    RRTNode3DPtr curr_node = (RRTNode3DPtr)kd_res_item_data(nbr_set);
                    neighbour_nodes.nearing_nodes.emplace_back(curr_node, false, false);
                    // store range query result so that we dont need to query again for rewire;
                    kd_res_next(nbr_set); // go to next in kd tree range query result
                }
                kd_res_free(nbr_set); // reset kd tree range query

                /* choose parent from kd tree range query result*/
                double dist2nearest = calDist(nearest_node->x, x_new);
                double min_dist_from_start(nearest_node->cost_from_start + dist2nearest);
                double cost_from_p(dist2nearest);
                RRTNode3DPtr min_node(nearest_node); // set the nearest_node as the default parent
                // TODO sort by potential cost-from-start

                for (auto& curr_node : neighbour_nodes.nearing_nodes) {
                    if (curr_node.node_ptr ==
                        nearest_node) // the nearest_node already calculated and checked collision free
                    {
                        continue;
                    }

                    // check potential first, then check edge collision
                    double curr_dist = calDist(curr_node.node_ptr->x, x_new);
                    double potential_dist_from_start = curr_node.node_ptr->cost_from_start + curr_dist;
                    if (min_dist_from_start > potential_dist_from_start) {
                        bool connected = isSegmentValid(curr_node.node_ptr->x, x_new);
                        curr_node.is_checked = true;
                        if (connected) {
                            curr_node.is_valid = true;
                            cost_from_p = curr_dist;
                            min_dist_from_start = potential_dist_from_start;
                            min_node = curr_node.node_ptr;
                        }
                    }
                }

                if (cfg_.visualize_process_en) {
                    vector<std::pair<rog_map::Vec3f, rog_map::Vec3f>> pairs;
                    pairs.emplace_back(nearest_node->x, x_new);
                    vis_ptr_->visualize_pairline(pairs, "rrt_line", visualization::Color::green, 0.1, idx);
                    vis_ptr_->visualize_a_ball(x_new, 0.15, "rrt_ball", visualization::Color::blue, 1.0, idx);
                }

                /* parent found within radius, then add a node to rrt and kd_tree */
                // sample-rejection
                double dist_to_goal = calDist(x_new, goal_node_->x);
                // if (min_dist_from_start + dist_to_goal >= goal_node_->cost_from_start)
                // {
                //   // ROS_WARN("parent found but sample rejected");
                //   continue;
                // }

                /* 1.1 add the randomly sampled node to rrt_tree */
                RRTNode3DPtr new_node(nullptr);
                new_node = addTreeNode(min_node, x_new, min_dist_from_start, cost_from_p);

                /* 1.2 add the randomly sampled node to kd_tree */
                kd_insert3(kd_tree, x_new[0], x_new[1], x_new[2], new_node);
                // end of find parent

                /* 2. try to connect to goal if possible */
                if (dist_to_goal <= cfg_.search_radius) {
                    bool is_connected2goal = isSegmentValid(x_new, goal_node_->x);
                    // this test can be omitted if sample-rejction is applied
                    bool is_better_path = goal_node_->cost_from_start > dist_to_goal + new_node->cost_from_start;
                    if (is_connected2goal && is_better_path) {
                        if (!goal_found) {
                            first_path_use_time_ = (ros::Time::now() - rrt_start_time).toSec();
                            ROS_INFO("[RRT*]: First path found, cost: %f, time: %f",
                                     dist_to_goal + new_node->cost_from_start,
                                     first_path_use_time_);
                        }
                        else {
                            ROS_INFO("[RRT*]: Find a better path, cost: %f, time: %f",
                                     dist_to_goal + new_node->cost_from_start,
                                     (ros::Time::now() - rrt_start_time).toSec());
                        }
                        goal_found = true;
                        changeNodeParent(goal_node_, new_node, dist_to_goal);
                        vector<Eigen::Vector3d> curr_best_path;
                        fillPath(goal_node_, curr_best_path);
                        path_list_.emplace_back(curr_best_path);
                        solution_cost_time_pair_list_.emplace_back(goal_node_->cost_from_start,
                                                                   (ros::Time::now() - rrt_start_time).toSec());
                        vis_ptr_->visualize_path(curr_best_path, "rrt_star_final_path");
                        vis_ptr_->visualize_pointcloud(curr_best_path, "rrt_star_final_wpts");
                        if (cfg_.use_informed_sampling) {
                            scale_[0] = goal_node_->cost_from_start / 2.0;
                            scale_[1] = sqrt(scale_[0] * scale_[0] - c_square);
                            scale_[2] = scale_[1];
                            sampler_.setInformedSacling(scale_);
                            std::vector<visualization::ELLIPSOID> ellps;
                            ellps.emplace_back(trans_, scale_, rot_);
                            if (cfg_.visualize_process_en) {
                                vis_ptr_->visualize_ellipsoids(ellps, "informed_set", visualization::yellow, 0.2);
                            }
                        }
                        else if (cfg_.use_GUILD_sampling) {
                            RRTNode3DPtr beacon_node = beaconSelect();
                            calInformedSet(beacon_node->cost_from_start, start_node_->x, beacon_node->x, scale1_,
                                           trans1_, rot1_);
                            calInformedSet(goal_node_->cost_from_start - beacon_node->cost_from_start, beacon_node->x,
                                           goal_node_->x, scale2_, trans2_, rot2_);
                            sampler_.setGUILDInformed(scale1_, trans1_, rot1_, scale2_, trans2_, rot2_);
                            std::vector<visualization::ELLIPSOID> ellps;
                            ellps.emplace_back(trans1_, scale1_, rot1_);
                            ellps.emplace_back(trans2_, scale2_, rot2_);
                            if (cfg_.visualize_process_en) {
                                vis_ptr_->visualize_ellipsoids(ellps, "local_set", visualization::green, 0.2);
                            }
                        }
                    }
                }

                /* 3.rewire */
                for (auto& curr_node : neighbour_nodes.nearing_nodes) {
                    // new_node->x = x_new
                    double dist_to_potential_child = calDist(new_node->x, curr_node.node_ptr->x);
                    bool not_consistent =
                        new_node->cost_from_start + dist_to_potential_child < curr_node.node_ptr->cost_from_start
                            ? 1
                            : 0;
                    bool promising = new_node->cost_from_start + dist_to_potential_child +
                                     calDist(curr_node.node_ptr->x, goal_node_->x) < goal_node_->cost_from_start
                                         ? 1
                                         : 0;
                    if (not_consistent && promising) {
                        bool connected(false);
                        if (curr_node.is_checked)
                            connected = curr_node.is_valid;
                        else
                            connected = isSegmentValid(new_node->x, curr_node.node_ptr->x);

                        // If we can get to a node via the sampled_node faster than via it's existing parent then change the parent
                        if (connected) {
                            double best_cost_before_rewire = goal_node_->cost_from_start;
                            changeNodeParent(curr_node.node_ptr, new_node, dist_to_potential_child);
                            if (best_cost_before_rewire > goal_node_->cost_from_start) {
                                vector<Eigen::Vector3d> curr_best_path;
                                fillPath(goal_node_, curr_best_path);
                                path_list_.emplace_back(curr_best_path);
                                solution_cost_time_pair_list_.emplace_back(goal_node_->cost_from_start,
                                                                           (ros::Time::now() - rrt_start_time).toSec());
                                if (cfg_.visualize_process_en) {
                                    vis_ptr_->clearMarker("rrt_line");
                                    vis_ptr_->clearMarker("rrt_ball");
                                    vis_ptr_->visualize_path(curr_best_path, "rrt_star_final_path");
                                    vis_ptr_->visualize_pointcloud(curr_best_path, "rrt_star_final_wpts");
                                }
                                if (cfg_.use_informed_sampling) {
                                    scale_[0] = goal_node_->cost_from_start / 2.0;
                                    scale_[1] = sqrt(scale_[0] * scale_[0] - c_square);
                                    scale_[2] = scale_[1];
                                    sampler_.setInformedSacling(scale_);
                                    std::vector<visualization::ELLIPSOID> ellps;
                                    ellps.emplace_back(trans_, scale_, rot_);
                                    if (cfg_.visualize_process_en) {
                                        vis_ptr_->visualize_ellipsoids(ellps, "informed_set", visualization::yellow,
                                                                       0.2);
                                    }
                                }
                                else if (cfg_.use_GUILD_sampling) {
                                    RRTNode3DPtr beacon_node = beaconSelect();
                                    calInformedSet(beacon_node->cost_from_start, start_node_->x, beacon_node->x,
                                                   scale1_, trans1_, rot1_);
                                    calInformedSet(goal_node_->cost_from_start - beacon_node->cost_from_start,
                                                   beacon_node->x, goal_node_->x, scale2_, trans2_, rot2_);
                                    sampler_.setGUILDInformed(scale1_, trans1_, rot1_, scale2_, trans2_, rot2_);
                                    std::vector<visualization::ELLIPSOID> ellps;
                                    ellps.emplace_back(trans1_, scale1_, rot1_);
                                    ellps.emplace_back(trans2_, scale2_, rot2_);
                                    if (cfg_.visualize_process_en) {
                                        vis_ptr_->visualize_ellipsoids(ellps, "local_set", visualization::green, 0.2);
                                    }
                                }
                            }
                        }
                    }
                    /* go to the next entry */
                }
                /* end of rewire */
                if (cfg_.visualize_process_en) {
                    ros::Duration(0.0005).sleep();
                }
            }
            /* end of sample once */
            if (cfg_.visualize_process_en) {
                vis_ptr_->clearMarker("rrt_line");
                vis_ptr_->clearMarker("rrt_ball");
            }
            vector<Eigen::Vector3d> vertice;
            vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> edges;
            sampleWholeTree(start_node_, vertice, edges);
            std::vector<visualization::BALL> balls;
            balls.reserve(vertice.size());
            visualization::BALL node_p;
            node_p.radius = 0.2;
            for (size_t i = 0; i < vertice.size(); ++i) {
                node_p.center = vertice[i];
                balls.push_back(node_p);
            }
            if (cfg_.visualize_process_en) {
                vis_ptr_->visualize_balls(balls, "tree_vertice", visualization::Color::blue, 1.0);
                vis_ptr_->visualize_pairline(edges, "tree_edges", visualization::Color::green, 0.1);
            }
            if (goal_found) {
                final_path_use_time_ = (ros::Time::now() - rrt_start_time).toSec();
                fillPath(goal_node_, final_path_);
                vis_ptr_->visualize_path(final_path_, "rrt_path");
                ROS_INFO_STREAM("[RRT*]: first_path_use_time: " << first_path_use_time_ << ", length: "
                    << solution_cost_time_pair_list_.front().first);
            }
            else if (valid_tree_node_nums_ == cfg_.max_tree_node_nums) {
                ROS_ERROR_STREAM(
                    "[RRT*]: NOT CONNECTED TO GOAL after " << cfg_.max_tree_node_nums << " nodes added to rrt-tree");
            }
            else {
                ROS_ERROR_STREAM("[RRT*]: NOT CONNECTED TO GOAL after " << (ros::Time::now() - rrt_start_time).toSec()
                    << " seconds");
            }
            return goal_found;
        }

        void sampleWholeTree(const RRTNode3DPtr& root, vector<Eigen::Vector3d>& vertice,
                             vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& edges) {
            if (root == nullptr)
                return;

            // whatever dfs or bfs
            RRTNode3DPtr node = root;
            std::queue<RRTNode3DPtr> Q;
            Q.push(node);
            while (!Q.empty()) {
                node = Q.front();
                Q.pop();
                for (const auto& leafptr : node->children) {
                    vertice.push_back(leafptr->x);
                    edges.emplace_back(std::make_pair(node->x, leafptr->x));
                    Q.push(leafptr);
                }
            }
        }

        void calInformedSet(double a2, const Eigen::Vector3d& foci1, const Eigen::Vector3d& foci2,
                            Eigen::Vector3d& scale, Eigen::Vector3d& trans, Eigen::Matrix3d& rot) {
            trans = (foci1 + foci2) / 2.0;
            scale[0] = a2 / 2.0;
            Eigen::Vector3d diff(foci2 - foci1);
            double c_square = diff.squaredNorm() / 4.0;
            scale[1] = sqrt(scale[0] * scale[0] - c_square);
            scale[2] = scale[1];
            rot.col(0) = diff.normalized();
            diff[2] = 0.0;
            rot.col(1) = Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitZ()) *
                diff.normalized(); // project to the x-y plane and then rotate 90 degree;
            rot.col(2) = rot.col(0).cross(rot.col(1));
        }

        RRTNode3DPtr beaconSelect() {
            double rand = sampler_.getUniRandNum();
            rand *= valid_tree_node_nums_;
            rand = std::floor(rand);

            RRTNode3DPtr beacon = nodes_pool_[rand];
            while (beacon->cost_from_start == calDist(start_node_->x, beacon->x) &&
                beacon->cost_from_start + calDist(beacon->x, goal_node_->x) >= goal_node_->cost_from_start) {
                rand = sampler_.getUniRandNum();
                rand *= valid_tree_node_nums_;
                rand = std::floor(rand);
                beacon = nodes_pool_[rand];
            }
            return beacon;
        }
    };
}


#endif //SRC_ROG_ASTAR_HPP
