#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common.h>
#include <vector>
#include <cmath>
#include <tr1/unordered_map>
#include "FOV_Checker/FOV_Checker.h"
#include <ros/package.h>
#include <stdlib.h>
#include <time.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Image.h>
#include <pcl/common/transforms.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <deque>
#include <numeric>

// #define DEBUG

#define likely(x) __builtin_expect(!!(x), 1) // gcc function, for if optimization
#define unlikely(x) __builtin_expect(!!(x), 0)

using namespace std;
using namespace Eigen;

#define MAX_INTENSITY 255
#define MIN_INTENSITY 199

std::string pkg_path;
std::ofstream myfile, collision_checktime_file;
deque<double> comp_time_vec;

std::string quad_name;
std::tr1::unordered_map<int, std::vector<PointType>> point_hashmap;
std::tr1::unordered_map<int, std::vector<int>> pointindex_hashmap;

pcl::PointCloud<PointType> cloud_explored;

FOV_Checker fov_checker;
BoxPointType env_box;

PointType min_center, max_center;

int cube_numx, cube_numy, cube_numz;

struct polar3D
{
  int theta;
  int fi;
  float r;
};

ros::Publisher pub_cloud, pub_pose, pub_intercloud, pub_dyncloud, pub_uavcloud, depth_img_pub_, comp_time_pub;

sensor_msgs::PointCloud2 local_map_pcl;
sensor_msgs::PointCloud2 local_depth_pcl;

ros::Subscriber odom_sub, UAV_odom_sub;
ros::Subscriber global_map_sub, local_map_sub;

ros::Timer local_sensing_timer, pose_timer, dynobj_timer;

bool has_global_map(false);
bool has_local_map(false);
bool has_odom(false);
bool has_dyn_map(false);

nav_msgs::Odometry odom_;
Eigen::Matrix4d sensor2body, sensor2world;

int output_pcd;
int collisioncheck_enable;
int is_360lidar;
int use_avia_pattern, use_vlp32_pattern, use_minicf_pattern,use_os128_pattern;
int livox_linestep;
double sensing_horizon, sensing_rate, estimation_rate, polar_resolution, yaw_fov, vertical_fov,\
         min_raylength, downsample_res, curvature_limit, hash_cubesize, collision_range;
double x_size, y_size, z_size;
double gl_xl, gl_yl, gl_zl;
double resolution, inv_resolution;
int GLX_SIZE, GLY_SIZE, GLZ_SIZE;

int plane_interline = 1;

// multi uav variables
int drone_id = 0;
vector<Eigen::Vector3d> other_uav_pos;
vector<double> other_uav_rcv_time;
vector<vector<PointType>> otheruav_points, otheruav_points_inrender;
vector<vector<int>> otheruav_pointsindex, otheruav_pointsindex_inrender;
pcl::PointCloud<PointType> otheruav_points_vis;
double uav_size[3];
int uav_points_num;
int drone_num = 0;
int drone_drawpoints_num[3];

int use_uav_extra_model = 1;
pcl::PointCloud<PointType> uav_extra_model;

// dynamic objects variables
int dynobj_enable;
double dynobject_size;
vector<Eigen::Vector3d> dynobj_poss;
vector<int> dynobj_move_modes;
vector<int> dynobj_types;
vector<Eigen::Vector3d> dynobj_dir;
vector<PointType> dynobj_points;
pcl::PointCloud<PointType> dynobj_points_vis;
vector<int> dynobj_pointsindex;
double dyn_velocity;
int dynobject_num;
int dyn_mode;
Eigen::Vector3d map_min, map_max;
int dyn_obs_diff_size_on = 1;
vector<double> dyn_obs_size_vec;
vector<ros::Time> dyn_start_time_vec;

int origin_mapptcount = 0;

pcl::PointCloud<pcl::Normal>::Ptr all_normals(new pcl::PointCloud<pcl::Normal>);
pcl::NormalEstimation<PointType, pcl::Normal> normalEstimation;

ros::Time last_odom_stamp = ros::TIME_MAX;

ros::Time start_time, dyn_start_time;

pcl::PointCloud<PointType> cloud_all_map, local_map, local_map_filled, point_in_sensor;
pcl::VoxelGrid<PointType> _voxel_sampler;
sensor_msgs::PointCloud2 local_map_pcd, sensor_map_pcd;

pcl::search::KdTree<PointType> _kdtreeLocalMap, kdtree_dyn;
vector<int> pointIdxRadiusSearch;
vector<float> pointRadiusSquaredDistance;
double collision_check_time_sum = 0;
int collision_check_time_count = 0;

sensor_msgs::PointCloud2 dynobj_points_pcd;

pcl::PointCloud<PointType> generate_sphere_cloud(double size)
{
  pcl::PointCloud<PointType> sphere_cloud;
  PointType temp_point;
  double radius = size / 2;
  double x, y, z;

  // use downsample resolution to compute how many points in a sphere
  int theta_num = 2 * M_PI * radius / downsample_res;
  int fi_num = M_PI * radius / downsample_res;

  for (int i = 0; i < theta_num; i++)
  {
    for (int j = 0; j < fi_num; j++)
    {
      x = radius * sin(i * 2 * M_PI / theta_num) * cos(j * M_PI / fi_num);
      y = radius * sin(i * 2 * M_PI / theta_num) * sin(j * M_PI / fi_num);
      z = radius * cos(i * 2 * M_PI / theta_num);
      temp_point.x = x;
      temp_point.y = y;
      temp_point.z = z;
      temp_point.intensity = MIN_INTENSITY + (MAX_INTENSITY - MIN_INTENSITY) * 1.0 / 2.0;
      sphere_cloud.push_back(temp_point);
    }
  }

  return sphere_cloud;
}

pcl::PointCloud<PointType> generate_box_cloud(double size)
{
  pcl::PointCloud<PointType> box_cloud;
  PointType temp_point;
  double x, y, z;

  // use downsample resolution to compute how many points in a box
  int x_num = size / downsample_res;
  int y_num = size / downsample_res;
  int z_num = size / downsample_res;

  // for (int i = 0; i < x_num; i++)
  // {
  //   for (int j = 0; j < y_num; j++)
  //   {
  //     for (int k = 0; k < z_num; k++)
  //     {
  //       x = i * downsample_res - size / 2;
  //       y = j * downsample_res - size / 2;
  //       z = k * downsample_res - size / 2;
  //       temp_point.x = x;
  //       temp_point.y = y;
  //       temp_point.z = z;
  //       temp_point.intensity = MIN_INTENSITY + (MAX_INTENSITY - MIN_INTENSITY) * 2.0 / 2.0;
  //       box_cloud.push_back(temp_point);
  //     }
  //   }
  // }

  //draw 2 xy plane
  for (int i = 0;i <= x_num;i++)
  {
    for (int j = 0;j <= y_num;j++)
    {
        x = i * downsample_res - size / 2;
        y = j * downsample_res - size / 2;
        z = - size / 2;
        temp_point.x = x;
        temp_point.y = y;
        temp_point.z = z;
        temp_point.intensity = MIN_INTENSITY + (MAX_INTENSITY - MIN_INTENSITY) * 2.0 / 2.0;
        box_cloud.push_back(temp_point);
        z = size / 2;
        temp_point.z = z;
        temp_point.intensity = MIN_INTENSITY + (MAX_INTENSITY - MIN_INTENSITY) * 2.0 / 2.0;
        box_cloud.push_back(temp_point);
    }
  }
  //draw 4 plane
  for (int k = 1;k < z_num;k++)
  {
    //draw x two lines
    for (int i = 0;i <= x_num;i++)
    {
        x = i * downsample_res - size / 2;
        y = - size / 2;
        z = k * downsample_res - size / 2;
        temp_point.x = x;
        temp_point.y = y;
        temp_point.z = z;
        temp_point.intensity = MIN_INTENSITY + (MAX_INTENSITY - MIN_INTENSITY) * 2.0 / 2.0;
        box_cloud.push_back(temp_point);

        x = i * downsample_res - size / 2;
        y = size / 2;
        z = k * downsample_res - size / 2;
        temp_point.x = x;
        temp_point.y = y;
        temp_point.z = z;
        temp_point.intensity = MIN_INTENSITY + (MAX_INTENSITY - MIN_INTENSITY) * 2.0 / 2.0;
        box_cloud.push_back(temp_point);
    }
    for (int j = 0; j <= y_num; j++)
    {
        x = - size / 2;
        y = j * downsample_res- size / 2;
        z = k * downsample_res - size / 2;
        temp_point.x = x;
        temp_point.y = y;
        temp_point.z = z;
        temp_point.intensity = MIN_INTENSITY + (MAX_INTENSITY - MIN_INTENSITY) * 2.0 / 2.0;
        box_cloud.push_back(temp_point);

        x = size / 2;
        y = j * downsample_res- size / 2;
        z = k * downsample_res - size / 2;
        temp_point.x = x;
        temp_point.y = y;
        temp_point.z = z;
        temp_point.intensity = MIN_INTENSITY + (MAX_INTENSITY - MIN_INTENSITY) * 2.0 / 2.0;
        box_cloud.push_back(temp_point);
    }
  }

  return box_cloud;
}

void generate_ptclouds_by_pos(Eigen::Vector3d obs_pos, int obs_type, pcl::PointCloud<PointType> &obs_cloud, vector<PointType> &obs_points)
{
  // 0 for using uav model, 1 for using sphere model, 2 for using box model
  switch (obs_type)
  {
  case 0:
    obs_cloud = uav_extra_model;
    for (int i = 0; i < obs_cloud.size(); i++)
    {
      obs_cloud.points[i].intensity = MIN_INTENSITY + (MAX_INTENSITY - MIN_INTENSITY) * 0.0 / 2.0;
    }
    break;

  case 1:
    obs_cloud = generate_sphere_cloud(dynobject_size);
    break;

  case 2:
    obs_cloud = generate_box_cloud(0.2);
    break;
  
  default:
    break;
  }

  for (int i = 0; i < obs_cloud.size(); i++)
  {
    obs_cloud.points[i].x = obs_cloud.points[i].x + obs_pos(0);
    obs_cloud.points[i].y = obs_cloud.points[i].y + obs_pos(1);
    obs_cloud.points[i].z = obs_cloud.points[i].z + obs_pos(2);
    obs_points.push_back(obs_cloud.points[i]);
  }
}


void dynobjGenerate(const ros::TimerEvent &event)
{
  if (has_global_map == true && dynobj_enable == 1)
  {
    dynobj_points.clear();
    dynobj_pointsindex.clear();
    dynobj_points_vis.clear();
    PointType temp_point;
    int dynpt_count = 0;
    double fly_time;
    Eigen::Vector3d gravity_vec;
    gravity_vec<<0,0,-1;
    Eigen::Vector3d dyntemp_dir_polar;    

    // rewrite generate dynamic obstacles
    for (int n = 0; n < dynobject_num; n++)
    {
      // according to motion mode, compute the positions of dynamic obstacle
      switch (dynobj_move_modes[n])
      {
        case 0:
          {
          // constant speed mode
          fly_time = (ros::Time::now() - dyn_start_time_vec[n]).toSec();
          dynobj_poss[n] = dynobj_poss[n] + fly_time * dynobj_dir[n];
          break;
          }

        case 1:
          {
          // constant gravity acceleration mode
          fly_time = (ros::Time::now() - dyn_start_time_vec[n]).toSec();
          dynobj_dir[n] = dynobj_dir[n] + gravity_vec * fly_time;
          dynobj_poss[n] = dynobj_poss[n] + fly_time * dynobj_dir[n];
          break;
          }

        case 2:
          {
          // random walk mode
          fly_time = (ros::Time::now() - dyn_start_time_vec[n]).toSec();
          dyntemp_dir_polar(0) = rand() / double(RAND_MAX) * 3.1415926;
          dyntemp_dir_polar(1) = (rand() / double(RAND_MAX) - 0.5) * 3.1415926;
          dynobj_dir[n](0) = dyn_velocity * sin(dyntemp_dir_polar(1));
          dynobj_dir[n](1) = dyn_velocity * cos(dyntemp_dir_polar(1)) * sin(dyntemp_dir_polar(0));
          dynobj_dir[n](2) = dyn_velocity * cos(dyntemp_dir_polar(1)) * cos(dyntemp_dir_polar(0));
          dynobj_poss[n] = dynobj_poss[n] + fly_time * dynobj_dir[n];
          break;
          }
        
        default:
          break;
      }

          // if dynamic obstacle is out of bounding box, regenerte one
          if (dynobj_poss[n](0) < map_min(0) || dynobj_poss[n](0) > map_max(0) || dynobj_poss[n](1) < map_min(1) || dynobj_poss[n](1) > map_max(1) || dynobj_poss[n](2) < map_min(2) || dynobj_poss[n](2) > map_max(2))
          {
            dynobj_poss[n](0) = rand() / double(RAND_MAX) * (map_max(0) - map_min(0)) + map_min(0);
            dynobj_poss[n](1) = rand() / double(RAND_MAX) * (map_max(1) - map_min(1)) + map_min(1);
            dynobj_poss[n](2) = rand() / double(RAND_MAX) * (map_max(2) - map_min(2)) + map_min(2);
            Eigen::Vector3d dyntemp_dir_polar;
            dyntemp_dir_polar(0) = rand() / double(RAND_MAX) * 3.1415926;
            dyntemp_dir_polar(1) = (rand() / double(RAND_MAX) - 0.5) * 3.1415926;
            dynobj_dir[n](0) = dyn_velocity * sin(dyntemp_dir_polar(1));
            dynobj_dir[n](1) = dyn_velocity * cos(dyntemp_dir_polar(1)) * sin(dyntemp_dir_polar(0));
            dynobj_dir[n](2) = dyn_velocity * cos(dyntemp_dir_polar(1)) * cos(dyntemp_dir_polar(0));
            
          }
      dyn_start_time_vec[n] = ros::Time::now();
      // generate point cloud of dynamic obstacle
      pcl::PointCloud<PointType> dynobj_cloud;
      vector<PointType> temp_dynobj_points;
      generate_ptclouds_by_pos(dynobj_poss[n], dynobj_types[n], dynobj_cloud, temp_dynobj_points);
      dynobj_points_vis = dynobj_points_vis + dynobj_cloud;
      dynobj_points.insert(dynobj_points.end(), temp_dynobj_points.begin(), temp_dynobj_points.end());
      for (int i = 0; i < temp_dynobj_points.size(); i++)
      {
        dynobj_pointsindex.push_back(dynpt_count + origin_mapptcount);
        dynpt_count++;
      }
    }

    // // first type dynamic obstacles, different sizes sphere
    // //! you can add mode here
    // switch (dyn_mode)
    // {
    //   case 0:
    //     // constant speed mode 
    //     // fly_time = (ros::Time::now() - dyn_start_time).toSec();
    //     for (int n = 0; n < dynobject_num; n++)
    //     {
    //       fly_time = (ros::Time::now() - dyn_start_time_vec[n]).toSec();
    //       dynobj_poss[n] = dynobj_poss[n] + fly_time * dynobj_dir[n];
    //       // if dynamic obstacle is out of bounding box, regenerte one
    //       if (dynobj_poss[n](0) < map_min(0) || dynobj_poss[n](0) > map_max(0) || dynobj_poss[n](1) < map_min(1) || dynobj_poss[n](1) > map_max(1) || dynobj_poss[n](2) < map_min(2) || dynobj_poss[n](2) > map_max(2))
    //       {
    //         dynobj_poss[n](0) = rand() / double(RAND_MAX) * (map_max(0) - map_min(0)) + map_min(0);
    //         dynobj_poss[n](1) = rand() / double(RAND_MAX) * (map_max(1) - map_min(1)) + map_min(1);
    //         dynobj_poss[n](2) = rand() / double(RAND_MAX) * (map_max(2) - map_min(2)) + map_min(2);
    //         Eigen::Vector3d dyntemp_dir_polar;
    //         dyntemp_dir_polar(0) = rand() / double(RAND_MAX) * 3.1415926;
    //         dyntemp_dir_polar(1) = (rand() / double(RAND_MAX) - 0.5) * 3.1415926;
    //         dynobj_dir[n](0) = dyn_velocity * sin(dyntemp_dir_polar(1));
    //         dynobj_dir[n](1) = dyn_velocity * cos(dyntemp_dir_polar(1)) * sin(dyntemp_dir_polar(0));
    //         dynobj_dir[n](2) = dyn_velocity * cos(dyntemp_dir_polar(1)) * cos(dyntemp_dir_polar(0));
    //         dyn_start_time_vec[n] = ros::Time::now();
    //       }
    //     }
    //     break;
    //   case 1:
    //     // with gravity
    //     // fly_time = (ros::Time::now() - dyn_start_time_vec[n]).toSec();
    //     // ROS_INFO("In dyn mode 1");
    //     for (int n = 0; n < dynobject_num; n++)
    //     {
    //       fly_time = (ros::Time::now() - dyn_start_time_vec[n]).toSec();
    //       Eigen::Vector3d gravity_vec;
    //       gravity_vec<<0,0,-9.81;
    //       // cout << "dyn_obj_pos" << dynobj_poss[n].transpose() << ", gravity value = "<< 0.5 * gravity_vec.transpose() * fly_time * fly_time<<endl;
    //       dynobj_dir[n] = dynobj_dir[n] + gravity_vec * fly_time;
    //       dynobj_poss[n] = dynobj_poss[n] + fly_time * dynobj_dir[n];// + 0.5 * gravity_vec * fly_time * fly_time;
    //       // if dynamic obstacle is out of bounding box, regenerte one
    //       if (dynobj_poss[n](0) < map_min(0) || dynobj_poss[n](0) > map_max(0) || dynobj_poss[n](1) < map_min(1) || dynobj_poss[n](1) > map_max(1) || dynobj_poss[n](2) < map_min(2) || dynobj_poss[n](2) > map_max(2))
    //       {
    //         dynobj_poss[n](0) = rand() / double(RAND_MAX) * (map_max(0) - map_min(0)) + map_min(0);
    //         dynobj_poss[n](1) = rand() / double(RAND_MAX) * (map_max(1) - map_min(1)) + map_min(1);
    //         dynobj_poss[n](2) = rand() / double(RAND_MAX) * (map_max(2) - map_min(2)) + map_min(2);
    //         Eigen::Vector3d dyntemp_dir_polar;
    //         dyntemp_dir_polar(0) = rand() / double(RAND_MAX) * 3.1415926;
    //         dyntemp_dir_polar(1) = (rand() / double(RAND_MAX) - 0.5) * 3.1415926;
    //         dynobj_dir[n](0) = dyn_velocity * sin(dyntemp_dir_polar(1));
    //         dynobj_dir[n](1) = dyn_velocity * cos(dyntemp_dir_polar(1)) * sin(dyntemp_dir_polar(0));
    //         dynobj_dir[n](2) = dyn_velocity * cos(dyntemp_dir_polar(1)) * cos(dyntemp_dir_polar(0));
    //         dyn_start_time_vec[n] = ros::Time::now();
    //       }
    //     }
    //     break;
    //   case 2:
    //     // random walk
    //     for (int n = 0; n < dynobject_num; n++)
    //     {
    //       fly_time = (ros::Time::now() - dyn_start_time_vec[n]).toSec();
    //         Eigen::Vector3d dyntemp_dir_polar;
    //         dyntemp_dir_polar(0) = rand() / double(RAND_MAX) * 3.1415926;
    //         dyntemp_dir_polar(1) = (rand() / double(RAND_MAX) - 0.5) * 3.1415926;
    //         dynobj_dir[n](0) = dyn_velocity * sin(dyntemp_dir_polar(1));
    //         dynobj_dir[n](1) = dyn_velocity * cos(dyntemp_dir_polar(1)) * sin(dyntemp_dir_polar(0));
    //         dynobj_dir[n](2) = dyn_velocity * cos(dyntemp_dir_polar(1)) * cos(dyntemp_dir_polar(0));
    //       dynobj_poss[n] = dynobj_poss[n] + fly_time * dynobj_dir[n];// + 0.5 * gravity_vec * fly_time * fly_time;
    //       // if dynamic obstacle is out of bounding box, regenerte one
    //       if (dynobj_poss[n](0) < map_min(0) || dynobj_poss[n](0) > map_max(0) || dynobj_poss[n](1) < map_min(1) || dynobj_poss[n](1) > map_max(1) || dynobj_poss[n](2) < map_min(2) || dynobj_poss[n](2) > map_max(2))
    //       {
    //         dynobj_poss[n](0) = rand() / double(RAND_MAX) * (map_max(0) - map_min(0)) + map_min(0);
    //         dynobj_poss[n](1) = rand() / double(RAND_MAX) * (map_max(1) - map_min(1)) + map_min(1);
    //         dynobj_poss[n](2) = rand() / double(RAND_MAX) * (map_max(2) - map_min(2)) + map_min(2);
    //         // Eigen::Vector3d dyntemp_dir_polar;
    //         dyntemp_dir_polar(0) = rand() / double(RAND_MAX) * 3.1415926;
    //         dyntemp_dir_polar(1) = (rand() / double(RAND_MAX) - 0.5) * 3.1415926;
    //         dynobj_dir[n](0) = dyn_velocity * sin(dyntemp_dir_polar(1));
    //         dynobj_dir[n](1) = dyn_velocity * cos(dyntemp_dir_polar(1)) * sin(dyntemp_dir_polar(0));
    //         dynobj_dir[n](2) = dyn_velocity * cos(dyntemp_dir_polar(1)) * cos(dyntemp_dir_polar(0));
    //         dyn_start_time_vec[n] = ros::Time::now();
    //       }
    //     }
    //     break;
    // }

    // if(dyn_obs_diff_size_on == 1)
    // {
    //   for (int n = 0; n < dynobject_num; n++)
    //   {
    //     // double dynobject_rand_size = rand() / double(RAND_MAX) * (dynobject_size - downsample_res) + downsample_res;
    //     for (double i = dynobj_poss[n](0) - 0.5 * dyn_obs_size_vec[n]; i < dynobj_poss[n](0) + 0.5 * dyn_obs_size_vec[n]; i = i + downsample_res)
    //     {
    //       for (double j = dynobj_poss[n](1) - 0.5 * dyn_obs_size_vec[n]; j < dynobj_poss[n](1) + 0.5 * dyn_obs_size_vec[n]; j = j + downsample_res)
    //       {
    //         for (double k = dynobj_poss[n](2) - 0.5 * dyn_obs_size_vec[n]; k < dynobj_poss[n](2) + 0.5 * dyn_obs_size_vec[n]; k = k + downsample_res)
    //         {
    //           if (sqrt((i - dynobj_poss[n](0)) * (i - dynobj_poss[n](0)) + (j - dynobj_poss[n](1)) * (j - dynobj_poss[n](1)) + (k - dynobj_poss[n](2)) * (k - dynobj_poss[n](2))) > 0.5 * dyn_obs_size_vec[n])
    //           {
    //             continue;
    //           }
    //           temp_point.x = i;
    //           temp_point.y = j;
    //           temp_point.z = k;
    //           dynobj_points_vis.push_back(temp_point);
    //           dynobj_points.push_back(temp_point);

    //           dynobj_pointsindex.push_back(dynpt_count + origin_mapptcount);
    //           dynpt_count++;
    //         }
    //       }
    //     }
    //   }      
    // }else{
    //   for (int n = 0; n < dynobject_num; n++)
    //   {
    //     for (double i = dynobj_poss[n](0) - 0.5 * dynobject_size; i < dynobj_poss[n](0) + 0.5 * dynobject_size; i = i + downsample_res)
    //     {
    //       for (double j = dynobj_poss[n](1) - 0.5 * dynobject_size; j < dynobj_poss[n](1) + 0.5 * dynobject_size; j = j + downsample_res)
    //       {
    //         for (double k = dynobj_poss[n](2) - 0.5 * dynobject_size; k < dynobj_poss[n](2) + 0.5 * dynobject_size; k = k + downsample_res)
    //         {
    //           if (sqrt((i - dynobj_poss[n](0)) * (i - dynobj_poss[n](0)) + (j - dynobj_poss[n](1)) * (j - dynobj_poss[n](1)) + (k - dynobj_poss[n](2)) * (k - dynobj_poss[n](2))) > 0.5 * dynobject_size)
    //           {
    //             continue;
    //           }
    //           temp_point.x = i;
    //           temp_point.y = j;
    //           temp_point.z = k;
    //           dynobj_points_vis.push_back(temp_point);
    //           dynobj_points.push_back(temp_point);

    //           dynobj_pointsindex.push_back(dynpt_count + origin_mapptcount);
    //           dynpt_count++;
    //         }
    //       }
    //     }
    //   }      
    // }


    dynobj_points_vis.width = dynobj_points_vis.points.size();
    dynobj_points_vis.height = 1;
    dynobj_points_vis.is_dense = true;

    pcl::toROSMsg(dynobj_points_vis, dynobj_points_pcd);
    dynobj_points_pcd.header = odom_.header;
    dynobj_points_pcd.header.frame_id = "world";
    pub_dyncloud.publish(dynobj_points_pcd);

    kdtree_dyn.setInputCloud(dynobj_points_vis.makeShared());
    has_dyn_map = true;

    dyn_start_time = ros::Time::now();
  }
}

// decentralize simualtion, get other uav odometry and generate point clouds
void multiOdometryCallbck(const nav_msgs::OdometryConstPtr &msg, int drone_id)
{
  Eigen::Vector3d uav_pos;
  uav_pos(0) = msg->pose.pose.position.x;
  uav_pos(1) = msg->pose.pose.position.y;
  uav_pos(2) = msg->pose.pose.position.z;
  other_uav_pos[drone_id] = uav_pos;
  other_uav_rcv_time[drone_id] = msg->header.stamp.toSec();

  otheruav_points[drone_id].clear();
  otheruav_pointsindex[drone_id].clear();
  otheruav_points_vis.clear();

  PointType temp_point;
  int uavpt_count = 0;

  if(use_uav_extra_model == 0)
  {
    // generate uav point cloud
    double x_min = uav_pos(0) - 0.5 * uav_size[0];
    double y_min = uav_pos(1) - 0.5 * uav_size[1];
    double z_min = uav_pos(2) - 0.5 * uav_size[2];

    for (int i = 0; i < drone_drawpoints_num[0]; i++)
    {
      for (int j = 0; j < drone_drawpoints_num[1]; j++)
      {
        for (int k = 0; k < drone_drawpoints_num[2]; k++)
        {
          temp_point.x = i * downsample_res + x_min;
          temp_point.y = j * downsample_res + y_min;
          temp_point.z = k * downsample_res + z_min;
          temp_point.intensity = ((float)(MAX_INTENSITY - MIN_INTENSITY)) * ((drone_id + 1.0) / (float(drone_num))) + MIN_INTENSITY; // set the intensity of the point
          otheruav_points[drone_id].push_back(temp_point);
          otheruav_pointsindex[drone_id].push_back(uavpt_count + origin_mapptcount + 100000 + drone_id * uav_points_num);
          otheruav_points_vis.push_back(temp_point);
          uavpt_count++;
        }
      }
    }    
  }else{
    // read pcd model from file and attach it to UAVs odom
    Eigen::Quaterniond pose;
    pose.x() = msg->pose.pose.orientation.x;
    pose.y() = msg->pose.pose.orientation.y;
    pose.z() = msg->pose.pose.orientation.z;
    pose.w() = msg->pose.pose.orientation.w;
    Eigen::Matrix3d uav_rot = pose.toRotationMatrix();
    for(int i=0;i<uav_extra_model.points.size();i++)
    {
          Eigen::Vector3d model_point;
          model_point(0) = uav_extra_model.points[i].x;
          model_point(1) = uav_extra_model.points[i].y;
          model_point(2) = uav_extra_model.points[i].z;
          model_point = uav_rot * model_point + uav_pos;
          temp_point.x = model_point(0);
          temp_point.y = model_point(1);
          temp_point.z = model_point(2);
          temp_point.intensity = ((float)(MAX_INTENSITY - MIN_INTENSITY)) * ((drone_id + 1.0) / (float(drone_num))) + MIN_INTENSITY; // set the intensity of the point
          otheruav_points[drone_id].push_back(temp_point);
          otheruav_pointsindex[drone_id].push_back(uavpt_count + origin_mapptcount + 100000 + drone_id * uav_points_num);
          otheruav_points_vis.push_back(temp_point);
          uavpt_count++;
    }
  }

  // publish uav point cloud
  sensor_msgs::PointCloud2 otheruav_points_vis_pcd;
  pcl::toROSMsg(otheruav_points_vis, otheruav_points_vis_pcd);
  otheruav_points_vis_pcd.header.frame_id = "world";
  otheruav_points_vis_pcd.header.stamp = ros::Time::now();
  pub_uavcloud.publish(otheruav_points_vis_pcd);
}

void rcvOdometryCallbck(const nav_msgs::Odometry &odom)
{
  /*if(!has_global_map)
    return;*/
  has_odom = true;
  odom_ = odom;

  Matrix4d body2world = Matrix4d::Identity();

  Eigen::Vector3d request_position;
  Eigen::Quaterniond pose;
  pose.x() = odom.pose.pose.orientation.x;
  pose.y() = odom.pose.pose.orientation.y;
  pose.z() = odom.pose.pose.orientation.z;
  pose.w() = odom.pose.pose.orientation.w;
  body2world.block<3, 3>(0, 0) = pose.toRotationMatrix();
  body2world(0, 3) = odom.pose.pose.position.x;
  body2world(1, 3) = odom.pose.pose.position.y;
  body2world(2, 3) = odom.pose.pose.position.z;

  // convert to cam pose
  sensor2world = body2world * sensor2body;

  ros::Time start_time = ros::Time::now();

  // collision check
  if (has_global_map && collisioncheck_enable)
  {

    // collision check running time statistics
    ros::Time start_time = ros::Time::now();

    PointType searchPoint;
    searchPoint.x = odom.pose.pose.position.x;
    searchPoint.y = odom.pose.pose.position.y;
    searchPoint.z = odom.pose.pose.position.z;
    if (_kdtreeLocalMap.radiusSearch(searchPoint, collision_range, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    {
      ROS_ERROR("ENVIRONMENT COLLISION DETECTED!!!");
    }
    if (dynobj_enable && has_dyn_map)
    {
      if (kdtree_dyn.radiusSearch(searchPoint, collision_range, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
      {
        ROS_ERROR("DYNAMIC OBJECTS COLLISION DETECTED!!!");
      }
    }

    ros::Time end_time = ros::Time::now();
    double collision_check_time = (end_time - start_time).toSec();
    collision_check_time_sum += collision_check_time;
    collision_check_time_count++;
    if (collision_check_time_count % 100 == 0)
    {
      ROS_WARN("Collision check time: %f", collision_check_time_sum / collision_check_time_count);
    }

  }

  ros::Time end_time = ros::Time::now();
  double total_time = (end_time - start_time).toSec();
  collision_checktime_file << total_time << endl;
}

void rcvGlobalPointCloudCallBack(const sensor_msgs::PointCloud2 &pointcloud_map)
{
  if (has_global_map)
    return;

  ROS_WARN("Global Pointcloud received..");

  pcl::PointCloud<PointType> cloud_input;
  pcl::fromROSMsg(pointcloud_map, cloud_input);

  _voxel_sampler.setLeafSize(downsample_res, downsample_res, downsample_res);
  _voxel_sampler.setInputCloud(cloud_input.makeShared());
  _voxel_sampler.filter(cloud_all_map);

  origin_mapptcount = cloud_all_map.points.size();

  _kdtreeLocalMap.setInputCloud(cloud_all_map.makeShared());

  normalEstimation.setInputCloud(cloud_all_map.makeShared());
  //对于每一个点都用半径为3cm的近邻搜索方式
  normalEstimation.setRadiusSearch(3.0 * downsample_res);
  // Kd_tree是一种数据结构便于管理点云以及搜索点云，法线估计对象会使用这种结构来找到哦啊最近邻点
  pcl::search::KdTree<PointType>::Ptr kdtree(new pcl::search::KdTree<PointType>);
  normalEstimation.setSearchMethod(kdtree);
  //计算法线
  normalEstimation.compute(*all_normals);

  ROS_WARN("Normal compute finished.., mapsize = %d", origin_mapptcount);

  // trans the map into hash map
  PointType pt_in, center;
  long int ind_x, ind_y, ind_z;

  // get max xyz
  PointType global_mapmin;
  PointType global_mapmax;
  pcl::getMinMax3D(cloud_all_map, global_mapmin, global_mapmax);
  map_min(0) = global_mapmin.x;
  map_min(1) = global_mapmin.y;
  map_min(2) = global_mapmin.z;
  map_max(0) = global_mapmax.x;
  map_max(1) = global_mapmax.y;
  map_max(2) = global_mapmax.z;

  cube_numx = floor((global_mapmax.x - global_mapmin.x) / hash_cubesize) + 20;
  cube_numy = floor((global_mapmax.y - global_mapmin.y) / hash_cubesize) + 20;
  cube_numz = floor((global_mapmax.z - global_mapmin.z) / hash_cubesize) + 200;

  env_box.vertex_min[0] = round(global_mapmin.x / hash_cubesize) * hash_cubesize - 10 * hash_cubesize;
  env_box.vertex_min[1] = round(global_mapmin.y / hash_cubesize) * hash_cubesize - 10 * hash_cubesize;
  env_box.vertex_min[2] = round(global_mapmin.z / hash_cubesize) * hash_cubesize - 100 * hash_cubesize;
  env_box.vertex_max[0] = round(global_mapmax.x / hash_cubesize) * hash_cubesize + 10 * hash_cubesize;
  env_box.vertex_max[1] = round(global_mapmax.y / hash_cubesize) * hash_cubesize + 10 * hash_cubesize;
  env_box.vertex_max[2] = round(global_mapmax.z / hash_cubesize) * hash_cubesize + 100 * hash_cubesize;
  fov_checker.Set_Env(env_box);
  fov_checker.Set_BoxLength(hash_cubesize);

  for (int i = 0; i < int(cloud_all_map.points.size()); i++)
  {
    pt_in = cloud_all_map.points[i];
    ind_x = (round((pt_in.x - env_box.vertex_min[0] + EPSS) / hash_cubesize));
    ind_y = (round((pt_in.y - env_box.vertex_min[1] + EPSS) / hash_cubesize));
    ind_z = (round((pt_in.z - env_box.vertex_min[2] + EPSS) / hash_cubesize));

    long int box_index = ind_x + ind_y * cube_numx + ind_z * cube_numx * cube_numy;
    point_hashmap[box_index].push_back(pt_in);
    pointindex_hashmap[box_index].push_back(i);
  }

  if (dynobj_enable == 1)
  {
    // dynamic objects initial pos generate
    srand((unsigned)time(NULL));
    for (int i = 0; i < dynobject_num; i++)
    {
      Eigen::Vector3d dyntemp_pos;
      dyntemp_pos(0) = rand() / double(RAND_MAX) * (global_mapmax.x - global_mapmin.x) + global_mapmin.x;
      dyntemp_pos(1) = rand() / double(RAND_MAX) * (global_mapmax.y - global_mapmin.y) + global_mapmin.y;
      dyntemp_pos(2) = rand() / double(RAND_MAX) * (global_mapmax.z - global_mapmin.z) + global_mapmin.z;
      dynobj_poss.push_back(dyntemp_pos);

      // random genrate dynamic object type and motion mode
      int dynobj_type = rand() % 3;
      dynobj_types.push_back(dynobj_type);
      // int dynobj_motion_mode = rand() % 2;
      dynobj_move_modes.push_back(dynobj_type);

      Eigen::Vector3d dyntemp_dir, dyntemp_dir_polar;
      dyntemp_dir_polar(0) = rand() / double(RAND_MAX) * 3.1415926;
      dyntemp_dir_polar(1) = (rand() / double(RAND_MAX) - 0.5) * 3.1415926;
      dyntemp_dir[2] = dyn_velocity * sin(dyntemp_dir_polar(1));
      dyntemp_dir[1] = dyn_velocity * cos(dyntemp_dir_polar(1)) * sin(dyntemp_dir_polar(0));
      dyntemp_dir[0] = dyn_velocity * cos(dyntemp_dir_polar(1)) * cos(dyntemp_dir_polar(0));
      dynobj_dir.push_back(dyntemp_dir);

      if(dyn_obs_diff_size_on)
      {
        double dynobject_rand_size = rand() / double(RAND_MAX) * (dynobject_size - downsample_res) + downsample_res;
        dyn_obs_size_vec.push_back(dynobject_rand_size);
      }

      dyn_start_time = ros::Time::now();
      dyn_start_time_vec.push_back(dyn_start_time);      
    }


  }

  has_global_map = true;
}

inline void euc2polar(Eigen::Vector3d &euc_pt, float length, polar3D *polar_pt)
{
  polar_pt->theta = round((atan2(euc_pt[1], euc_pt[0])) / M_PI * 180.0 / polar_resolution);
  polar_pt->fi = round((atan2(euc_pt[2], euc_pt.head<2>().norm()) / M_PI * 180.0 / polar_resolution));
  polar_pt->r = length;
}

inline void polar2euc(polar3D *polar_pt, Eigen::Vector3d &euc_pt)
{
  // trans from polar coordinate to euclidean coordinate
  // theta_angle
  euc_pt[2] = polar_pt->r * sin(polar_pt->fi * polar_resolution / 180.0 * M_PI);
  euc_pt[1] = polar_pt->r * cos(polar_pt->fi * polar_resolution / 180.0 * M_PI) * sin(polar_pt->theta * polar_resolution / 180.0 * M_PI);
  euc_pt[0] = polar_pt->r * cos(polar_pt->fi * polar_resolution / 180.0 * M_PI) * cos(polar_pt->theta * polar_resolution / 180.0 * M_PI);
}

int sense_count = 0;
double duration1 = 0.0;
double duration2 = 0.0;
double duration3 = 0.0;
double duration4 = 0.0;
double duration5 = 0.0;
double duration6 = 0.0;
double duration7 = 0.0;
double duration8 = 0.0;
double duration_interline = 0.0;
int comp_time_count = 0;

void renderSensedPoints(const ros::TimerEvent &event)
{

  ros::Time t1 = ros::Time::now();

  if (!has_global_map || !has_odom)
    return;

  Eigen::Quaterniond q;
  q.x() = odom_.pose.pose.orientation.x;
  q.y() = odom_.pose.pose.orientation.y;
  q.z() = odom_.pose.pose.orientation.z;
  q.w() = odom_.pose.pose.orientation.w;

  // trans other uav point in
  if (otheruav_points.size() > 0)
  {
    otheruav_points_inrender = otheruav_points;
    otheruav_pointsindex_inrender = otheruav_pointsindex;
  }

  const Eigen::Vector3d pos(odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z);
  // pos << odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z;

  // const Eigen::Matrix3d rot(q.toRotationMatrix());

  Eigen::Matrix3d rot_body2lidar = Matrix3d::Identity();

  // rotate lidar 
  // Eigen::Vector3d eulerAngle_body2lidar(0,0.5236,0);
  // Eigen::AngleAxisd rollAngle(AngleAxisd(eulerAngle_body2lidar(0),Vector3d::UnitX()));
  // Eigen::AngleAxisd pitchAngle(AngleAxisd(eulerAngle_body2lidar(1),Vector3d::UnitY()));
  // Eigen::AngleAxisd yawAngle(AngleAxisd(eulerAngle_body2lidar(2),Vector3d::UnitZ()));
  // rot_body2lidar=yawAngle*pitchAngle*rollAngle;
  // rotate lidar end

  const Eigen::Matrix3d rot(q.toRotationMatrix()*rot_body2lidar);

  // rot = q.toRotationMatrix();
  const Eigen::Vector3d yaw_x(1, 0, 0);
  // yaw_x << 1,0,0;

  Eigen::Vector3d rotmyaw = rot * yaw_x;
  const Eigen::Vector3d yaw_vec(rotmyaw(0), rotmyaw(1), 0);
  // yaw_vec(2) = 0;

  local_map.points.clear();
  local_map_filled.points.clear();

  Eigen::Vector3d searchpoint_pos;
  Eigen::Vector3d sensorrange_vec;
  sensorrange_vec << 0.5 * sensing_horizon, 0, 0;
  searchpoint_pos = pos + rot * sensorrange_vec;

  PointType searchPoint;
  searchPoint.x = searchpoint_pos(0);
  searchPoint.y = searchpoint_pos(1);
  searchPoint.z = searchpoint_pos(2);

  pointIdxRadiusSearch.clear();
  pointRadiusSquaredDistance.clear();

  // polar coordinate matrix
  // add margin
  double margin_ = 8.0;
  const double vertical_fov_margined_ = vertical_fov + margin_;
  const int polar_height = ceil(vertical_fov_margined_ / polar_resolution);
  double temp_yaw;
  int temp_width;
  if (is_360lidar == 1)
  {
    temp_yaw = 360.0;
    temp_width = ceil(360.0 / polar_resolution);
  }
  else
  {
    temp_yaw = yaw_fov + margin_;
    temp_width = ceil(temp_yaw / polar_resolution);
  }

  const double yaw_fov_margined_ = temp_yaw;
  const int polar_width = temp_width;

  Eigen::MatrixXf polar_matrix = Eigen::MatrixXf::Zero(polar_width, polar_height);
  polar_matrix.setConstant(2 * sensing_horizon);
  Eigen::MatrixXi polarindex_matrix = Eigen::MatrixXi::Zero(polar_width, polar_height);
  Eigen::MatrixXi polarindextype_matrix = Eigen::MatrixXi::Zero(polar_width, polar_height);
  // 0 means no interline, 1 means depth interline, 2 means normal interline, 3 means neighber depth interline, 4 means dynamic or other uav points
  Eigen::MatrixXi polarpointintensity_matrix = Eigen::MatrixXi::Zero(polar_width, polar_height);
  polarpointintensity_matrix.setConstant(MIN_INTENSITY);
  Eigen::MatrixXi allowblur_matrix = Eigen::MatrixXi::Zero(polar_width, polar_height);
  int original_pointcount = 0;
  int pointcount = 0;
  int changepointcount = 0;

  const double tan_ver_ = tan(M_PI * (vertical_fov_margined_ / 2.0) / 180.0);
  const double yaw_ = (M_PI * (yaw_fov_margined_ / 2.0) / 180.0);
  const double ver_threshold = cos(M_PI * (vertical_fov_margined_ / 2.0) / 180.0);
  const double cover_dis = 0.55 * 1.7321 * downsample_res; // 0.707
  // compute effective range
  const double effect_range = cover_dis / sin(0.5 * polar_resolution * M_PI / 180.0);
  // ROS_INFO("POLAR R = %f, EFFECT RANGE = %f",polar_pt.r,effect_range);

  double max_fov_angle;
  if (yaw_fov_margined_ > vertical_fov_margined_)
  {
    max_fov_angle = yaw_fov_margined_;
  }
  else
  {
    max_fov_angle = vertical_fov_margined_;
  }

  // save the points need to interline
  vector<int> culling_vectheta;
  vector<int> culling_vecfi;
  vector<int> culling_kdindex;

  sense_count++;
  ros::Time t2 = ros::Time::now();
  duration1 = ((t2 - t1).toSec()) + duration1;

  // pattern
  Eigen::MatrixXf pattern_matrix = Eigen::MatrixXf::Zero(polar_width, polar_height);
  pattern_matrix.setConstant(1);

  // avia pattern
  if (use_avia_pattern == 1)
  {
    float w1 = 763.82589;     // 7294.0*2.0*3.1415926/60.0
    float w2 = -488.41293788; // -4664.0*2.0*3.1415926/60.0
    // int linestep = 2;
    // double point_duration = 0.000025;
    double point_duration = 0.000004167 * 6;

    float t_duration = 1.0 / sensing_rate;
    double t_start = (t2 - start_time).toSec();

    double scale_x = 0.48 * polar_width / 2.0;
    double scale_y = 0.43 * polar_height / 2.0;

    int linestep = ceil(livox_linestep / polar_resolution);

    for (double t_i = t_start; t_i < t_start + t_duration; t_i = t_i + point_duration)
    {
      int x = round(scale_x * (cos(w1 * t_i) + cos(w2 * t_i))) + round(0.5 * polar_width);
      int y = round(scale_y * (sin(w1 * t_i) + sin(w2 * t_i))) + round(0.5 * polar_height);

      if (x > (polar_width - 1))
      {
        x = (polar_width - 1);
      }
      else if (x < 0)
      {
        x = 0;
      }

      if (y > (polar_height - 1))
      {
        y = (polar_height - 1);
      }
      else if (y < 0)
      {
        y = 0;
      }

      pattern_matrix(x, y) = 2; // real pattern
      pattern_matrix(x, y + linestep) = 2;
      pattern_matrix(x, y + 2 * linestep) = 2;
      pattern_matrix(x, y + 3 * linestep) = 2;
      pattern_matrix(x, y - linestep) = 2;
      pattern_matrix(x, y - 2 * linestep) = 2;
    }
  }

  // vlp32 pattern
  if (use_vlp32_pattern == 1)
  {
    int step = round((40.0) / 32 / polar_resolution);
    int bottom_pattern = -ceil(25.0 / polar_resolution);
    for (int i = 0; i < 32; i++)
    {
      int y = bottom_pattern + i * step + round(0.5 * polar_height);

      for (int j = 0; j < polar_width; j++)
      {
        if (y > (polar_height - 1))
        {
          y = (polar_height - 1);
        }
        else if (y < 0)
        {
          y = 0;
        }
        pattern_matrix(j, y) = 2;
      }
    }
  }

  // os128 pattern
  if (use_os128_pattern == 1)
  {
    int step = round((45.0) / 128 / polar_resolution);
    int bottom_pattern = -ceil(22.5 / polar_resolution);
    for (int i = 0; i < 128; i++)
    {
      int y = bottom_pattern + i * step + round(0.5 * polar_height);

      for (int j = 0; j < polar_width; j++)
      {
        if (y > (polar_height - 1))
        {
          y = (polar_height - 1);
        }
        else if (y < 0)
        {
          y = 0;
        }
        pattern_matrix(j, y) = 2;
      }
    }
  }

  if (use_minicf_pattern == 1)
  {
    double point_duration = 1.0 / 200000.0;

    float t_duration = 1.0 / sensing_rate;
    double t_start = (t2 - start_time).toSec();

    double scale_x = 0.48 * polar_width / 2.0;
    double scale_y = 0.43 * polar_height / 2.0;
    double PI = 3.141519265357;

    for (double t_i = t_start; t_i < t_start + t_duration; t_i = t_i + point_duration)
    {
      int x = (int(-round(-62050.63 * t_i + 3.11 * cos(314159.2 * t_i) * sin(628.318 * 2 * t_i))) % 360) / polar_resolution;
      int y = round(25.5 * cos(20 * PI * t_i) + 4 * cos(2 * PI / 0.006 * t_i) * cos(10000 * PI * t_i) + 22.5) / polar_resolution + round(0.5 * polar_height);

      // ROS_INFO("X = %d, Y = %d",x,y);
      if (x > (polar_width - 1))
      {
        x = (polar_width - 1);
      }
      else if (x < 0)
      {
        x = 0;
      }

      if (y > (polar_height - 1))
      {
        y = (polar_height - 1);
      }
      else if (y < 0)
      {
        y = 0;
      }

      pattern_matrix(x, y) = 2; // real pattern
    }
  }

  ros::Time t3 = ros::Time::now();
  duration2 = ((t3 - t2).toSec()) + duration2;

  // hashmap with fov checker
  vector<BoxPointType> fov_boxes;
  // rotmyaw << 0,1,0;
  // ROS_INFO("POS = %f,%f,%f, rotmyaw = %f,%f,%f", pos(0), pos(1),pos(2), rotmyaw(0),rotmyaw(1),rotmyaw(2));
  vector<PointType> fov_points;
  vector<int> fov_pointsindex;

  if (is_360lidar == 1)
  {
    // box expand to search
    int expand_num = ceil(sensing_horizon / hash_cubesize);
    int cen_x = (round((pos(0) - env_box.vertex_min[0] + EPSS) / hash_cubesize));
    int cen_y = (round((pos(1) - env_box.vertex_min[1] + EPSS) / hash_cubesize));
    int cen_z = (round((pos(2) - env_box.vertex_min[2] + EPSS) / hash_cubesize));
    long int ind_x, ind_y, ind_z;
    for (int i = -expand_num; i <= expand_num; i++)
    {
      for (int j = -expand_num; j <= expand_num; j++)
      {
        for (int k = -expand_num; k <= expand_num; k++)
        {
          ind_x = cen_x + i;
          ind_y = cen_y + j;
          ind_z = cen_z + k;

          long int box_index = ind_x + ind_y * cube_numx + ind_z * cube_numx * cube_numy;
          fov_points.insert(fov_points.end(), point_hashmap[box_index].begin(), point_hashmap[box_index].end());
          fov_pointsindex.insert(fov_pointsindex.end(), pointindex_hashmap[box_index].begin(), pointindex_hashmap[box_index].end());
        }
      }
    }
    // ROS_INFO("POINT SIZE = %d, box size = %d", fov_points.size(), fov_boxes.size());
  }
  else
  {
    fov_checker.check_fov(pos, rotmyaw, M_PI * (max_fov_angle / 2.0 + 10) / 180.0, sensing_horizon, fov_boxes);
    Eigen::Vector3d box_center_temp;
    long int ind_x, ind_y, ind_z;
    for (int i = 0; i < fov_boxes.size(); i++)
    {
      box_center_temp(0) = (fov_boxes[i].vertex_max[0] - fov_boxes[i].vertex_min[0]) * 0.5 + fov_boxes[i].vertex_min[0];
      box_center_temp(1) = (fov_boxes[i].vertex_max[1] - fov_boxes[i].vertex_min[1]) * 0.5 + fov_boxes[i].vertex_min[1];
      box_center_temp(2) = (fov_boxes[i].vertex_max[2] - fov_boxes[i].vertex_min[2]) * 0.5 + fov_boxes[i].vertex_min[2];

      ind_x = (round((box_center_temp(0) - env_box.vertex_min[0] + EPSS) / hash_cubesize));
      ind_y = (round((box_center_temp(1) - env_box.vertex_min[1] + EPSS) / hash_cubesize));
      ind_z = (round((box_center_temp(2) - env_box.vertex_min[2] + EPSS) / hash_cubesize));

      PointType box_pt;
      box_pt.x = box_center_temp(0);
      box_pt.y = box_center_temp(1);
      box_pt.z = box_center_temp(2);

      long int box_index = ind_x + ind_y * cube_numx + ind_z * cube_numx * cube_numy;

      fov_points.insert(fov_points.end(), point_hashmap[box_index].begin(), point_hashmap[box_index].end());
      fov_pointsindex.insert(fov_pointsindex.end(), pointindex_hashmap[box_index].begin(), pointindex_hashmap[box_index].end());
    }
  }

  // add dyanmic objects
  // Step 1: add dynpoints int fov points
  if (dynobj_enable == 1)
  {
    fov_points.insert(fov_points.end(), dynobj_points.begin(), dynobj_points.end());
    fov_pointsindex.insert(fov_pointsindex.end(), dynobj_pointsindex.begin(), dynobj_pointsindex.end());
  }

  if (drone_num > 1)
  {
    for (int i = 0; i < drone_num; i++)
    {
      fov_points.insert(fov_points.end(), otheruav_points_inrender[i].begin(), otheruav_points_inrender[i].end());
      fov_pointsindex.insert(fov_pointsindex.end(), otheruav_pointsindex_inrender[i].begin(), otheruav_pointsindex_inrender[i].end());
    }
  }

  ros::Time t4 = ros::Time::now();
  duration3 = ((t4 - t3).toSec()) + duration3;

  double lookup_time = (t4-t1).toSec();
  myfile << lookup_time << " ";

  const size_t size_ = fov_points.size();

  omp_set_num_threads(32);

#ifndef DEBUG
#pragma omp parallel /*default(none)*/                                                             \
    shared(pattern_matrix, polar_matrix, cloud_all_map, pointIdxRadiusSearch,                  \
           min_raylength, downsample_res, polar_resolution, use_avia_pattern, _kdtreeLocalMap, \
           polarindex_matrix, local_map_filled, local_map,                                     \
           all_normals, vertical_fov, yaw_fov, sensing_horizon, fov_points, fov_pointsindex,   \
           polarindextype_matrix, original_pointcount, curvature_limit, allowblur_matrix,      \
           origin_mapptcount, drone_num, polarpointintensity_matrix, uav_points_num)
  {
#pragma omp for
#endif
    for (size_t i = 0; i < size_; ++i)
    {
      // auto pt = cloud_all_map.points[pointIdxRadiusSearch[i]];
      auto pt = fov_points[i];
      Eigen::Vector3d pt3;
      pt3[0] = pt.x;
      pt3[1] = pt.y;
      pt3[2] = pt.z;
      auto dir = pt3 - pos;

      if (dir.norm() > sensing_horizon)
      {
        continue;
      }

      original_pointcount++;

      // use polar to filter the right pointclouds
      polar3D polar_pt;
      Eigen::Vector3d dir_vec;
      // dir_vec = pt3 - pos;

      // trans coordinate to lidar coordinate
      dir_vec = rot.transpose() * dir;

      euc2polar(dir_vec, dir_vec.norm(), &polar_pt);
      // ROS_INFO("dir_vec = %f,%f,%f, polar = %d,%d",dir(0),dir(1),dir(2), polar_pt.theta,polar_pt.fi);
      int cen_theta_index = polar_pt.theta + round(0.5 * polar_width);
      int cen_fi_index = polar_pt.fi + round(0.5 * polar_height);

      int half_cover_angle = ceil(
          (asin(cover_dis / dir_vec.norm()) / (M_PI * polar_resolution / 180.0)));
      // ROS_INFO("half cover angle = %d",half_cover_angle);
      // int half_cover_angle = 1;

      if (polar_pt.r > effect_range)
      {

        if (unlikely((cen_theta_index > (polar_width - 1)) || (cen_theta_index < 0) || (cen_fi_index > (polar_height - 1)) || (cen_fi_index < 0)))
        {
          continue;
        }

        if (polar_matrix(cen_theta_index, cen_fi_index) > polar_pt.r)
        {

          polar_matrix(cen_theta_index, cen_fi_index) = polar_pt.r;
          polarindextype_matrix(cen_theta_index, cen_fi_index) = 0;
          allowblur_matrix(cen_theta_index, cen_fi_index) = 0;
          // polarindex_matrix(cen_theta_index,cen_fi_index) = pointIdxRadiusSearch[i];

          if ((fov_pointsindex[i] >= origin_mapptcount + 100000) && drone_num > 1)
          {
            int point_temp_index = fov_pointsindex[i] - origin_mapptcount - 100000;
            int droneid_temp = (point_temp_index / (uav_points_num));
            polarpointintensity_matrix(cen_theta_index, cen_fi_index) = fov_points[i].intensity;
          }
          else
          {
            polarpointintensity_matrix(cen_theta_index, cen_fi_index) = MIN_INTENSITY;
          }
        }
      }
      else
      {

        int theta_start = cen_theta_index - half_cover_angle;
        int theta_end = cen_theta_index + half_cover_angle;
        int fi_start = cen_fi_index - half_cover_angle;
        int fi_end = cen_fi_index + half_cover_angle;

        for (int theta_index_o = theta_start;
             theta_index_o <= theta_end; theta_index_o++)
        {
          for (int fi_index_o = fi_start;
               fi_index_o <= fi_end; fi_index_o = fi_index_o + 1)
          {

            if (unlikely((theta_index_o > (polar_width - 1)) || (theta_index_o < 0) || (fi_index_o > (polar_height - 1)) || (fi_index_o < 0)))
            {
              continue;
            }
            if (polar_matrix(theta_index_o, fi_index_o) > polar_pt.r + sensing_horizon)
            {
              polar_matrix(theta_index_o, fi_index_o) = polar_pt.r + sensing_horizon;
              polarindex_matrix(theta_index_o, fi_index_o) = fov_pointsindex[i];
              polarindextype_matrix(theta_index_o, fi_index_o) = 1;
              allowblur_matrix(theta_index_o, fi_index_o) = 0;
              if ((fov_pointsindex[i] >= origin_mapptcount + 100000) && drone_num > 1)
              {
                polarpointintensity_matrix(theta_index_o, fi_index_o) = fov_points[i].intensity;
              }
              else
              {
                polarpointintensity_matrix(theta_index_o, fi_index_o) = MIN_INTENSITY;
              }
            }
          }
        }

      } // end of for loop
    }

#ifndef DEBUG
  }
#endif
  ros::Time t5 = ros::Time::now();
  duration4 = (t5 - t4).toSec() + duration4;

  // ROS_INFO("After first filter");
  ros::Time t6 = ros::Time::now();

  if (plane_interline == 1)
  {

#pragma omp parallel /*default(none)*/ \
    shared(polarindex_matrix, culling_kdindex)
    {
      std::vector<int> vec_private;

#pragma omp for nowait collapse(2)
      // compute plane interline
      for (int i = 0; i < polar_width; i++)
      {
        for (int j = 0; j < polar_height; j++)
        {
          if (polarindex_matrix(i, j) != 0)
          {
            // check if the index in the vector
            if (std::find(vec_private.begin(), vec_private.end(), polarindex_matrix(i, j)) !=
                vec_private.end())
            {
              continue;
            }
            else
            {
              vec_private.push_back(polarindex_matrix(i, j));
            }
          }
        }
      }
#pragma omp critical
      culling_kdindex.insert(culling_kdindex.end(), vec_private.begin(), vec_private.end());
    }
    // ROS_INFO("CULLING COUNT = %d",culling_kdindex.size());

    t6 = ros::Time::now();
    duration5 = (t6 - t5).toSec() + duration5;
    duration_interline = 0;

    // omp_set_num_threads(32);
    std::vector<int>::iterator iter;
    int culling_size = culling_kdindex.size();

    double duration_interline = 0.0;
    double duration_direct = 0.0;

#ifndef DEBUG
#pragma omp parallel /*default(none)*/                                                                \
    shared(culling_kdindex,                                                                       \
           polar_matrix, cloud_all_map,                                                           \
           downsample_res, polar_resolution,                                                      \
           all_normals, curvature_limit, fov_points, origin_mapptcount, dynobj_points,            \
           polarindextype_matrix, sensing_horizon, dynobj_enable, culling_size, allowblur_matrix, \
           otheruav_points_inrender, drone_num, uav_points_num, polarpointintensity_matrix)
    {
#pragma omp for
#endif
      // for(iter = culling_kdindex.begin();iter!=culling_kdindex.end();++iter)
      for (int i = 0; i < culling_size; i++)
      {
        ros::Time t_in1 = ros::Time::now();

        // multi uav add
        int droneid_temp, point_temp_index;

        int point_index = culling_kdindex[i];

        // check if it is dyn points
        PointType pt;
        if (likely(point_index < origin_mapptcount))
        {
          pt = cloud_all_map.points[point_index];
        }
        else
        {
          if (dynobj_enable == 1)
          {
            pt = dynobj_points[point_index - origin_mapptcount];
          }
          else if (drone_num > 1)
          {
            point_temp_index = point_index - origin_mapptcount - 100000;
            droneid_temp = (point_temp_index / (uav_points_num));

            if ((point_temp_index - droneid_temp * uav_points_num) > (otheruav_points_inrender[droneid_temp].size() - 1))
              continue;
            pt = otheruav_points_inrender[droneid_temp][point_temp_index - droneid_temp * uav_points_num];
          }
          else
          {
            continue;
          }
        }

        Eigen::Vector3d pt3;
        pt3[0] = pt.x;
        pt3[1] = pt.y;
        pt3[2] = pt.z;
        auto dir = pt3 - pos;

        polar3D polar_pt;
        Eigen::Vector3d dir_vec;
        // dir_vec = pt3 - pos;

        // trans coordinate to lidar coordinate
        dir_vec = rot.transpose() * dir;
        double pt_dis = dir_vec.norm();
        euc2polar(dir_vec, pt_dis, &polar_pt);
        // ROS_INFO("dir_vec = %f,%f,%f, polar = %d,%d",dir(0),dir(1),dir(2), polar_pt.theta,polar_pt.fi);
        int cen_theta_index = polar_pt.theta + round(0.5 * polar_width);
        int cen_fi_index = polar_pt.fi + round(0.5 * polar_height);

        int half_cover_angle = ceil(
            (asin(cover_dis / pt_dis) / (M_PI * polar_resolution / 180.0)));

        ros::Time t_in2 = ros::Time::now();
        // ROS_INFO("Init using %f s",(t_in2-t_in1).toSec());

        //! check if it is dyn points
        if (likely(point_index < origin_mapptcount))
        {

          Eigen::Vector3d plane_normal;
          plane_normal(0) = all_normals->points[point_index].normal_x;
          plane_normal(1) = all_normals->points[point_index].normal_y;
          plane_normal(2) = all_normals->points[point_index].normal_z;
          float curvature;
          curvature = all_normals->points[point_index].curvature;

          if (isnan(plane_normal(0)) || isnan(plane_normal(1)) || isnan(plane_normal(2))) //|| curvature > 10*(1.0/(0.5*downsample_res))
          {
            continue;
          }

          int theta_start = cen_theta_index - half_cover_angle;
          int theta_end = cen_theta_index + half_cover_angle;
          int fi_start = cen_fi_index - half_cover_angle;
          int fi_end = cen_fi_index + half_cover_angle;

          int theta_index_o = theta_start;
          int fi_index_o = fi_start;

          for (int theta_index_o = theta_start;
               theta_index_o <= theta_end; theta_index_o++)
          {
            for (int fi_index_o = fi_start;
                 fi_index_o <= fi_end; fi_index_o = fi_index_o + 1)
            {
              // ros::Time t8 = ros::Time::now();
              // ros::Time t_in3 = ros::Time::now();

              polar3D cur_polarpt;
              Eigen::Vector3d ray, inter_point, inter_point_world;
              double vpt, line_t;

              if (unlikely((theta_index_o > (polar_width - 1)) || (theta_index_o < 0) || (fi_index_o > (polar_height - 1)) || (fi_index_o < 0)))
              {
              }
              else
              {
                // compute plane interline
                cur_polarpt.theta = theta_index_o - round(0.5 * polar_width);
                cur_polarpt.fi = fi_index_o - round(0.5 * polar_height);
                cur_polarpt.r = 1.0;
                polar2euc(&cur_polarpt, ray);
                ray = rot * ray;
                ray.normalize();
                vpt = ray.dot(plane_normal);

                line_t = dir.dot(plane_normal) / vpt;
                inter_point_world = pos + line_t * ray;

                if ((inter_point_world - pt3).norm() > 1 * 0.8660254 * downsample_res) // sqrt 3   0.5*1.7321  2*0.8660254*downsample_res
                {
                  // ROS_INFO("OUT OF LIMIT");
                }
                else
                {
                  inter_point = rot.transpose() * (inter_point_world - pos);
                  if (polar_matrix(theta_index_o, fi_index_o) > inter_point.norm())
                  {
                    polar_matrix(theta_index_o, fi_index_o) = inter_point.norm();
                    polarindextype_matrix(theta_index_o, fi_index_o) = 2;

                    // limit curvature, avoid a good plane to be interlined
                    if (curvature > 0.05)
                    {
                      // polarindextype_matrix(theta_index_o,fi_index_o) = 3;
                      allowblur_matrix(theta_index_o, fi_index_o) = 1;
                    }
                    else
                    {
                      // polarindextype_matrix(theta_index_o,fi_index_o) = 2;
                      allowblur_matrix(theta_index_o, fi_index_o) = 1;
                    }
                  }
                }
              }
              //  ros::Time t_in4 = ros::Time::now();
              //   duration_interline =duration_interline + (t_in4-t_in3).toSec();//
            }
          }
        }
        else
        {

          ros::Time t_in5 = ros::Time::now();

          int theta_start = cen_theta_index - half_cover_angle;
          int theta_end = cen_theta_index + half_cover_angle;
          int fi_start = cen_fi_index - half_cover_angle;
          int fi_end = cen_fi_index + half_cover_angle;

          for (int theta_index_o = theta_start;
               theta_index_o <= theta_end; theta_index_o++)
          {
            for (int fi_index_o = fi_start;
                 fi_index_o <= fi_end; fi_index_o++)
            {
              // ros::Time t8 = ros::Time::now();
              if (unlikely((theta_index_o > (polar_width - 1)) || (theta_index_o < 0) || (fi_index_o > (polar_height - 1)) || (fi_index_o < 0)))
              {
                continue;
              }
              if (polar_matrix(theta_index_o, fi_index_o) > polar_pt.r)
              {

                polar_matrix(theta_index_o, fi_index_o) = polar_pt.r;
                polarindextype_matrix(theta_index_o, fi_index_o) = 4;
                allowblur_matrix(theta_index_o, fi_index_o) = 0;
              }
            }
          }
          //  ros::Time t_in6 = ros::Time::now();
          //   duration_direct =duration_direct + (t_in6-t_in5).toSec();
          //  ROS_INFO("End one");
        }
      }
#ifndef DEBUG
    }
#endif
  }

  // ROS_INFO("After interline");

  ros::Time t7 = ros::Time::now();
  duration6 = (t7 - t6).toSec() + duration6;
  // ROS_INFO("Duration interline = %f, duration direct = %f",duration_interline,duration_direct);

  for (int i = 0; i < polar_width; i++)
  {
    for (int j = 0; j < polar_height; j++)
    {
      if (polar_matrix(i, j) < 1.999 * sensing_horizon && polar_matrix(i, j) > sensing_horizon)
      {
        polar_matrix(i, j) = polar_matrix(i, j) - sensing_horizon;
        polarindextype_matrix(i, j) = 1;
        allowblur_matrix(i, j) = 1;
      }
    }
  }

  int free_flag = 0;

#ifndef DEBUG
#pragma omp parallel /*default(none)*/                                                                 \
    shared(use_avia_pattern, use_vlp32_pattern, use_minicf_pattern, is_360lidar,                   \
           polar_matrix, pattern_matrix, min_raylength, vertical_fov, local_map_filled, free_flag, \
           sensing_horizon, rotmyaw, polarindextype_matrix, polarpointintensity_matrix)
  {
#endif
    pcl::PointCloud<PointType> temp_cloud;
#ifndef DEBUG
#pragma omp for nowait
#endif
    // trans polar matrix to point clouds
    for (int i = 0; i < polar_width; i++)
    {
      for (int j = 0; j < polar_height; j++)
      {
        if (use_avia_pattern == 1 || use_vlp32_pattern || use_minicf_pattern)
        {
          if (pattern_matrix(i, j) == 0 || pattern_matrix(i, j) == 1)
          {
            continue;
          }
        }
        if (polar_matrix(i, j) < sensing_horizon)
        {
          Eigen::Vector3d addeuc_pt;
          polar3D polarindex_pt;
          polarindex_pt.theta = i - round(0.5 * polar_width);
          polarindex_pt.fi = j - round(0.5 * polar_height);
          polarindex_pt.r = polar_matrix(i, j);
          polar2euc(&polarindex_pt, addeuc_pt);
          addeuc_pt = rot * addeuc_pt + pos;
          PointType add_pcl_pt;
          add_pcl_pt.x = addeuc_pt(0);
          add_pcl_pt.y = addeuc_pt(1);
          add_pcl_pt.z = addeuc_pt(2);
          // add_pcl_pt.intensity = polarindextype_matrix(i, j) * 20;

          add_pcl_pt.intensity = polarpointintensity_matrix(i, j);

          // filter the margin
          Eigen::Vector3d pt3;
          pt3[0] = add_pcl_pt.x;
          pt3[1] = add_pcl_pt.y;
          pt3[2] = add_pcl_pt.z;
          auto dir = pt3 - pos;

          if (dir.norm() < min_raylength)
            continue;

          if (is_360lidar == 1)
          {
          }
          else
          {
            if (acos(dir.dot(rotmyaw) / (dir.norm() * rotmyaw.norm()) > M_PI * (vertical_fov / 2.0) / 180.0))
            {
              continue;
            }
          }
          // push back the filtered points
          // local_map_filled.push_back(add_pcl_pt);
          temp_cloud.push_back(add_pcl_pt);
        }
        else if (polar_matrix(i, j) < 1.999 * sensing_horizon)
        {
          // polar_matrix(i,j) = polar_matrix(i,j) - sensing_horizon;
          Eigen::Vector3d addeuc_pt;
          polar3D polarindex_pt;
          polarindex_pt.theta = i - round(0.5 * polar_width);
          polarindex_pt.fi = j - round(0.5 * polar_height);
          polarindex_pt.r = polar_matrix(i, j);
          polar2euc(&polarindex_pt, addeuc_pt);
          addeuc_pt = rot * addeuc_pt + pos; //.transpose()*
          PointType add_pcl_pt;
          add_pcl_pt.x = addeuc_pt(0);
          add_pcl_pt.y = addeuc_pt(1);
          add_pcl_pt.z = addeuc_pt(2);
          add_pcl_pt.intensity = polarindextype_matrix(i, j) * 20;

          // filter the margin
          Eigen::Vector3d pt3;
          pt3[0] = add_pcl_pt.x;
          pt3[1] = add_pcl_pt.y;
          pt3[2] = add_pcl_pt.z;
          auto dir = pt3 - pos;

          if (dir.norm() < min_raylength)
            continue;

          if (is_360lidar == 1)
          {
          }
          else
          {
            if (acos(dir.dot(rotmyaw) / (dir.norm() * rotmyaw.norm()) > M_PI * (vertical_fov / 2.0) / 180.0))
            {
              continue;
            }
          }
          // push back the filtered points
          // local_map_filled.push_back(add_pcl_pt);
          temp_cloud.push_back(add_pcl_pt);
        }
        else if (polar_matrix(i, j) >= 1.999 * sensing_horizon)
        {
          Eigen::Vector3d addeuc_pt;
          polar3D polarindex_pt;
          polarindex_pt.theta = i - round(0.5 * polar_width);
          polarindex_pt.fi = j - round(0.5 * polar_height);
          polarindex_pt.r = 2 * sensing_horizon;
          polar2euc(&polarindex_pt, addeuc_pt);
          addeuc_pt = rot * addeuc_pt + pos;
          PointType add_pcl_pt;
          add_pcl_pt.x = addeuc_pt(0);
          add_pcl_pt.y = addeuc_pt(1);
          add_pcl_pt.z = addeuc_pt(2);
          // add_pcl_pt.intensity = polarindextype_matrix(i, j) * 20;
          add_pcl_pt.intensity = MIN_INTENSITY;
          // filter the margin
          Eigen::Vector3d pt3;
          pt3[0] = add_pcl_pt.x;
          pt3[1] = add_pcl_pt.y;
          pt3[2] = add_pcl_pt.z;
          auto dir = pt3 - pos;

          if (dir.norm() < min_raylength)
            continue;

          if (is_360lidar == 1)
          {
          }
          else
          {
            if (acos(dir.dot(rotmyaw) / (dir.norm() * rotmyaw.norm()) > M_PI * (vertical_fov / 2.0) / 180.0))
            {
              continue;
            }
          }

          free_flag = 1;
          // local_map_filled.push_back(add_pcl_pt);
          // temp_cloud.push_back(add_pcl_pt);
        }
      }
    }
#ifndef DEBUG
#pragma omp critical
#endif
    local_map_filled.insert(local_map_filled.end(), temp_cloud.begin(), temp_cloud.end());

#ifndef DEBUG
  }
#endif

  ros::Time t8 = ros::Time::now();
  duration7 = (t8 - t7).toSec() + duration7;

  if (free_flag == 1)
  {
    // ROS_WARN("Give free far !!!!!!!!!!!!!!!!!!!!!!!");
  }

  // ROS_INFO("GET OUT OF LOOP, pointcount = %d, origin pointcount = %d, change point = %d",local_map_filled.points.size(),original_pointcount,changepointcount);

  local_map.width = local_map.points.size();
  local_map.height = 1;
  local_map.is_dense = true;

  local_map_filled.width = local_map_filled.points.size();
  local_map_filled.height = 1;
  local_map_filled.is_dense = true;

  pcl::toROSMsg(local_map_filled, local_map_pcd);
  local_map_pcd.header = odom_.header;
  local_map_pcd.header.frame_id = "world";
  pub_cloud.publish(local_map_pcd);

  // transform
  std::string sensor_frame_id_ = "/sensor";
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transform;
  ros::Time time_stamp_ = odom_.header.stamp;
  transform.header.stamp = time_stamp_;
  transform.header.frame_id = "world";
  transform.child_frame_id = sensor_frame_id_;

  transform.transform.translation.x = pos.x();
  transform.transform.translation.y = pos.y();
  transform.transform.translation.z = pos.z();
  transform.transform.rotation.x = q.x();
  transform.transform.rotation.y = q.y();
  transform.transform.rotation.z = q.z();
  transform.transform.rotation.w = q.w();

  br.sendTransform(transform);

  Eigen::Matrix4d sensor2world;
  sensor2world << rot(0, 0), rot(0, 1), rot(0, 2), pos.x(),
      rot(1, 0), rot(1, 1), rot(1, 2), pos.y(),
      rot(2, 0), rot(2, 1), rot(2, 2), pos.z(),
      0, 0, 0, 1;
  Eigen::Matrix4d world2sensor;
  world2sensor = sensor2world.inverse();

  // pointcloud
  point_in_sensor.points.clear();
  pcl::transformPointCloud(local_map_filled, point_in_sensor, world2sensor);
  point_in_sensor.width = point_in_sensor.points.size();
  point_in_sensor.height = 1;
  point_in_sensor.is_dense = true;
  pcl::toROSMsg(point_in_sensor, sensor_map_pcd);
  sensor_map_pcd.header.frame_id = sensor_frame_id_;
  sensor_map_pcd.header.stamp = time_stamp_;
  pub_intercloud.publish(sensor_map_pcd);

  // depth image generation and output
  cv::Mat img(polar_height, polar_width, CV_32FC1);
  for (int i = 0; i < polar_width; i++)
  {
    for (int j = 0; j < polar_height; j++)
    {
      double depth = polar_matrix(i, j);
      if (depth == 0)
      {
        continue;
      }
      double color_id = depth / sensing_horizon;
      if (color_id > 1.0)
      {
        color_id = 1.0;
      }
      img.at<float>(polar_height - 1 - j, polar_width - 1 - i) = color_id;
    }
  }

  cv_bridge::CvImage out_msg;
  out_msg.header.stamp = time_stamp_;
  out_msg.header.frame_id = "/sensor";
  out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  out_msg.image = img.clone();
  depth_img_pub_.publish(out_msg.toImageMsg());

  ros::Time t_total = ros::Time::now();
  duration8 = (t_total - t8).toSec() + duration8;
  // ROS_INFO("Average statics: %f, %f ,%f ,%f, %f, %f ,%f ,%f,  total_time = %f",duration1/sense_count,duration2/sense_count,duration3/sense_count,duration4/sense_count,duration5/sense_count,duration6/sense_count,duration7/sense_count,duration8/sense_count,(t_total-t1).toSec());

  double interpolation_time = (t_total - t4).toSec();
  myfile << interpolation_time << endl;

  double comp_time_temp = (t_total - t1).toSec();
  comp_time_vec.push_back(comp_time_temp);
  if (comp_time_count > 20)
  {
    comp_time_vec.pop_front();
    geometry_msgs::PoseStamped totaltime_pub;
    totaltime_pub.pose.position.x = accumulate(comp_time_vec.begin(), comp_time_vec.end(), 0.0) / comp_time_vec.size();
    comp_time_pub.publish(totaltime_pub);
    ROS_INFO("Temp compute time = %lf, average compute time = %lf", comp_time_temp, totaltime_pub.pose.position.x);
  }
  else
  {
    comp_time_count++;
  }
}

void pubSensorPose(const ros::TimerEvent &e)
{
  Eigen::Quaterniond q;
  q = sensor2world.block<3, 3>(0, 0);

  geometry_msgs::PoseStamped sensor_pose;
  sensor_pose.header = odom_.header;
  sensor_pose.header.frame_id = "/map";
  sensor_pose.pose.position.x = sensor2world(0, 3);
  sensor_pose.pose.position.y = sensor2world(1, 3);
  sensor_pose.pose.position.z = sensor2world(2, 3);
  sensor_pose.pose.orientation.w = q.w();
  sensor_pose.pose.orientation.x = q.x();
  sensor_pose.pose.orientation.y = q.y();
  sensor_pose.pose.orientation.z = q.z();
  pub_pose.publish(sensor_pose);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pcl_render");
  ros::NodeHandle nh("~");

  nh.param("quadrotor_name", quad_name, std::string("quadrotor"));
  nh.getParam("is_360lidar", is_360lidar);
  nh.getParam("sensing_horizon", sensing_horizon);
  nh.getParam("sensing_rate", sensing_rate);
  nh.getParam("estimation_rate", estimation_rate);
  nh.getParam("polar_resolution", polar_resolution);
  nh.getParam("yaw_fov", yaw_fov);
  nh.getParam("vertical_fov", vertical_fov);
  nh.getParam("min_raylength", min_raylength);
  nh.getParam("downsample_res", downsample_res);
  nh.getParam("livox_linestep", livox_linestep);
  nh.getParam("use_avia_pattern", use_avia_pattern);
  nh.getParam("curvature_limit", curvature_limit);
  nh.getParam("hash_cubesize", hash_cubesize);
  nh.getParam("use_vlp32_pattern", use_vlp32_pattern);
  nh.getParam("use_minicf_pattern", use_minicf_pattern);

  // dyn parameters
  nh.getParam("dynobj_enable", dynobj_enable);
  nh.getParam("dynobject_size", dynobject_size);
  nh.getParam("dynobject_num", dynobject_num);
  nh.getParam("dyn_mode", dyn_mode);
  nh.getParam("dyn_velocity", dyn_velocity);

  nh.getParam("use_uav_extra_model", use_uav_extra_model);

  nh.getParam("collisioncheck_enable", collisioncheck_enable);
  nh.getParam("collision_range", collision_range);

  nh.getParam("output_pcd", output_pcd);

  // subscribe other uav pos
  nh.param("uav_num", drone_num, 1);
  nh.param("drone_id", drone_id, 0);
  other_uav_pos.resize(drone_num);
  other_uav_rcv_time.resize(drone_num);
  otheruav_points.resize(drone_num);
  otheruav_pointsindex.resize(drone_num);
  ros::Subscriber *subs = new ros::Subscriber[drone_num];
  for (int i = 0; i < drone_num; i++)
  {
    if (i == drone_id)
    {
      continue;
    }
    string topic = "/quad_";
    topic += to_string(i);
    topic += "/lidar_slam/odom";
    cout << topic << endl;
    subs[i] = nh.subscribe<nav_msgs::Odometry>(topic, 1000, boost::bind(&multiOdometryCallbck, _1, i));
  }

  pcl::VoxelGrid<PointType> sor;
  int pcd_read_status;
  if(use_uav_extra_model)
  {
    string uav_model_path;
    uav_model_path = ros::package::getPath("odom_visualization");//= "/home/mars/catkin_ws2/src/Exploration_sim/octomap_mapping/octomap_server"
    uav_model_path.append("/meshes/yunque001.pcd");
    std::cout << "\nFound pkg_path = " << uav_model_path << std::endl;
    // myfile.open(pkg_path.c_str(), std::ios_base::out);//, std::ios_base::out

    pcd_read_status = pcl::io::loadPCDFile<PointType>(uav_model_path, uav_extra_model);
    if (pcd_read_status == -1)
    {
        cout << "can't read uav extra model file." << endl;
        return 0;
    }
    //downsample uav model
    sor.setInputCloud(uav_extra_model.makeShared());
    sor.setLeafSize(downsample_res, downsample_res, downsample_res);
    sor.filter(uav_extra_model);    
    uav_points_num = uav_extra_model.size();
  }else{
    uav_size[0] = 0.4;
    uav_size[1] = 0.4;
    uav_size[2] = 0.3;
    drone_drawpoints_num[0] = (uav_size[0] / downsample_res);
    drone_drawpoints_num[1] = (uav_size[1] / downsample_res);
    drone_drawpoints_num[2] = (uav_size[2] / downsample_res);
    uav_points_num = drone_drawpoints_num[0] * drone_drawpoints_num[1] * drone_drawpoints_num[2];
    ROS_INFO("drone_drawpoints_num = %d,%d,%d, Uav points num = %d", drone_drawpoints_num[0], drone_drawpoints_num[1], drone_drawpoints_num[2], uav_points_num);

  }

  // UAV_odom_sub = nh.subscribe("/other_quad/lidar_slam/odom", 1000, UAVOdomCallback);


  // subscribe point cloud
  global_map_sub = nh.subscribe("global_map", 1, rcvGlobalPointCloudCallBack);
  odom_sub = nh.subscribe("odometry", 50, rcvOdometryCallbck);

  // publisher depth image and color image
  pub_dyncloud = nh.advertise<sensor_msgs::PointCloud2>("dyn_cloud", 10);
  pub_intercloud = nh.advertise<sensor_msgs::PointCloud2>("sensor_cloud", 10);
  pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("cloud", 10);
  pub_pose = nh.advertise<geometry_msgs::PoseStamped>("sensor_pose", 10);
  pub_uavcloud = nh.advertise<sensor_msgs::PointCloud2>("uav_cloud", 10); //扫描机身的点云
  depth_img_pub_ = nh.advertise<sensor_msgs::Image>("depth_img", 10);
  comp_time_pub = nh.advertise<geometry_msgs::PoseStamped>("simulator_compute_time", 10);
  double sensing_duration = 1.0 / sensing_rate;
  double estimate_duration = 1.0 / estimation_rate;

  start_time = ros::Time::now();

  local_sensing_timer = nh.createTimer(ros::Duration(sensing_duration), renderSensedPoints);
  pose_timer = nh.createTimer(ros::Duration(estimate_duration), pubSensorPose);
  dynobj_timer = nh.createTimer(ros::Duration(sensing_duration), dynobjGenerate);

  // pkg_path = ros::package::getPath("octomap_server");
  // pkg_path.append("/data/" + quad_name + "_explore_persentage.txt");
  // std::cout << "\nFound pkg_path = " << pkg_path << std::endl;
  // myfile.open(pkg_path.c_str(), std::ios_base::out); //, std::ios_base::out

  // open file to record time consumuption
  pkg_path = ros::package::getPath("local_sensing_node");  
  pkg_path.append("/data/" + quad_name + "_time_consumption.txt");
  std::cout << "\nFound pkg_path = " << pkg_path << std::endl;
  myfile.open(pkg_path.c_str(), std::ios_base::out); 

  // open file to record collision check time consumption
  pkg_path = ros::package::getPath("local_sensing_node");
  pkg_path.append("/data/" + quad_name + "_collision_check_time_consumption.txt");
  std::cout << "\nFound pkg_path = " << pkg_path << std::endl;
  collision_checktime_file.open(pkg_path.c_str(), std::ios_base::out);

  inv_resolution = 1.0 / resolution;
  gl_xl = -x_size / 2.0;
  gl_yl = -y_size / 2.0;
  gl_zl = 0.0;
  GLX_SIZE = (int)(x_size * inv_resolution);
  GLY_SIZE = (int)(y_size * inv_resolution);
  GLZ_SIZE = (int)(z_size * inv_resolution);

  sensor2body << 0.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

  ros::Rate rate(100);
  bool status = ros::ok();
  while (status)
  {
    ros::spinOnce();
    status = ros::ok();
    rate.sleep();
  }

  // if (output_pcd == 1)
  // {
  //   // write pcd
  //   pkg_path = ros::package::getPath("octomap_server");
  //       std::string pcd_name(quad_name + "cloud_explored");
  //   if (pcl::io::savePCDFileASCII(pkg_path + "/data/" + pcd_name + ".pcd", local_map) >= 0)
  //   {
  //     std::cerr << "Saved  " << pcd_name << ".pcd" << std::endl;
  //   }
  // }

  return 0;
}
