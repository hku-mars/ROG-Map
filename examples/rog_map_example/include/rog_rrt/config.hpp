#ifndef ROG_RRT_CONFIG_HPP
#define ROG_RRT_CONFIG_HPP

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace rog_rrt {
    using std::string;
    using std::vector;
    typedef pcl::PointXYZINormal PclPoint;
    typedef pcl::PointCloud<PclPoint> PointCloud;
    using Vec3f = Eigen::Vector3d;

    class Config {
    private:
        ros::NodeHandle nh_;

        template<class T>
        bool LoadParam(string param_name, T &param_value, T default_value = T{}, bool required = false) {
            if (nh_.getParam(param_name, param_value)) {
                printf("\033[0;32m Load param %s succes: \033[0;0m", (nh_.getNamespace() + "/" + param_name).c_str());
                std::cout << param_value << std::endl;
                return true;
            } else {
                printf("\033[0;33m Load param %s failed, use default value: \033[0;0m",
                       (nh_.getNamespace() + "/" + param_name).c_str());
                param_value = default_value;
                std::cout << param_value << std::endl;
                if (required) {
                    throw std::invalid_argument(
                            string("Required param " + (nh_.getNamespace() + "/" + param_name) + " not found"));
                }
                return false;
            }
        }

        template<class T>
        bool LoadParam(string param_name, vector<T> &param_value, vector<T> default_value = vector<T>{},
                       bool required = false) {
            if (nh_.getParam(param_name, param_value)) {
                printf("\033[0;32m Load param %s succes: \033[0;0m", (nh_.getNamespace() + "/" + param_name).c_str());
                for (size_t i = 0; i < param_value.size(); i++) {
                    std::cout << param_value[i] << " ";
                }
                std::cout << std::endl;
                return true;
            } else {
                printf("\033[0;33m Load param %s failed, use default value: \033[0;0m",
                       (nh_.getNamespace() + "/" + param_name).c_str());
                param_value = default_value;
                for (size_t i = 0; i < param_value.size(); i++) {
                    std::cout << param_value[i] << " ";
                }
                std::cout << std::endl;
                if (required) {
                    throw std::invalid_argument(
                            string("Required param " + (nh_.getNamespace() + "/" + param_name) + " not found"));
                }
                return false;
            }
        }

    public:
        double steer_length;
        double search_radius;
        double search_time;
        int max_tree_node_nums;
        bool use_informed_sampling;
        bool use_GUILD_sampling;
        bool visualize_process_en;

        Vec3f example_start;
        Vec3f example_goal;

        Config(const ros::NodeHandle & nh) : nh_(nh){
            LoadParam("rrt_star/visualize_process_en", visualize_process_en, false);
            LoadParam("rrt_star/steer_length", steer_length, 0.5);
            LoadParam("rrt_star/search_radius", search_radius, 0.5);
            LoadParam("rrt_star/max_tree_node_nums", max_tree_node_nums, 10000);
            LoadParam("rrt_star/use_informed_sampling", use_informed_sampling, false);
            LoadParam("rrt_star/use_GUILD_sampling", use_GUILD_sampling, false);

            vector<double> tmp;
            LoadParam("rrt_star/example_start",tmp,tmp);
            if(tmp.size() == 3) {
                example_start = Vec3f(tmp[0], tmp[1], tmp[2]);
                tmp.clear();
            }
            LoadParam("rrt_star/example_goal",tmp,tmp);
            if(tmp.size() == 3) {
                example_goal = Vec3f(tmp[0], tmp[1], tmp[2]);
                tmp.clear();
            }
        }

    };


}

#endif