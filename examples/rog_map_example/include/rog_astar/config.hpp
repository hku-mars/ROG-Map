#ifndef ROG_ASTAR_CONFIG_HPP
#define ROG_ASTAR_CONFIG_HPP

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace rog_astar {
    using std::string;
    using std::vector;
    using Vec3f = Eigen::Vector3d;
    using Vec3i = Eigen::Vector3i;
    typedef pcl::PointXYZINormal PclPoint;
    typedef pcl::PointCloud<PclPoint> PointCloud;



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
        bool visualize_process_en;
        bool allow_diag;
        Vec3i map_voxel_num, map_size_i;
        int heu_type;

        Vec3f example_start;
        Vec3f example_goal;

        Config(const ros::NodeHandle & nh) : nh_(nh){
            vector<int> vox;
            LoadParam( "astar/visualize_process_en", visualize_process_en, false);
            LoadParam("astar/map_voxel_num", vox, vox);
            if(vox.size() == 3) {
                map_voxel_num = Vec3i(vox[0], vox[1], vox[2]);
                vox.clear();
            }
            LoadParam( "astar/allow_diag", allow_diag, false);
            LoadParam( "astar/heu_type", heu_type, 0);

            vector<double> tmp;
            LoadParam("astar/example_start",tmp,tmp);
            if(tmp.size() == 3) {
                example_start = Vec3f(tmp[0], tmp[1], tmp[2]);
                tmp.clear();
            }
            LoadParam("astar/example_goal",tmp,tmp);
            if(tmp.size() == 3) {
                example_goal = Vec3f(tmp[0], tmp[1], tmp[2]);
                tmp.clear();
            }

            map_size_i = map_voxel_num / 2;
            map_voxel_num = map_size_i * 2 + rog_map::Vec3i::Constant(1);
        }

    };


}

#endif