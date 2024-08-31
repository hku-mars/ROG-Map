#include "opengl_sim.hpp"
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <deque>
#include <numeric>
#include <ros/package.h>
#include <string>
#include <tr1/unordered_map>

using namespace Eigen;
using namespace std;

#define MAX_INTENSITY 1
#define MIN_INTENSITY 0.1

string file_name, pkg_path;
std::ofstream myfile, collision_checktime_file;
deque<double> comp_time_vec;

std::string quad_name;
int drone_num = 0;
int drone_id = 0;

opengl_pointcloud_render render;

ros::Publisher pub_cloud, pub_pose,pub_intercloud,pub_dyncloud, pub_uavcloud, depth_img_pub_,comp_time_pub,pub_collisioncloud;
sensor_msgs::PointCloud2 local_map_pcl;
sensor_msgs::PointCloud2 local_depth_pcl;
ros::Subscriber odom_sub;
ros::Subscriber global_map_sub, local_map_sub;
ros::Timer local_sensing_timer, pose_timer, dynobj_timer;

ros::Time t_init;

bool has_odom(false);

nav_msgs::Odometry odom_;
Eigen::Matrix4f sensor2body, sensor2world;

int output_pcd;
int collisioncheck_enable;
int is_360lidar;
bool use_inf_pt;
int use_avia_pattern,use_vlp32_pattern,use_minicf_pattern,use_os128_pattern,use_gaussian_filter;
int livox_linestep;
double sensing_horizon, sensing_rate, estimation_rate, polar_resolution, yaw_fov, vertical_fov, min_raylength, downsample_res, curvature_limit,hash_cubesize,collision_range;
double x_size, y_size, z_size;
double gl_xl, gl_yl, gl_zl;
double resolution, inv_resolution;
int GLX_SIZE, GLY_SIZE, GLZ_SIZE;

int plane_interline = 1;

// multi uav variables
// int drone_id = 0;
vector<Eigen::Vector3d> other_uav_pos;
vector<double> other_uav_rcv_time;
vector<vector<PointType>> otheruav_points, otheruav_points_inrender;
vector<vector<int>> otheruav_pointsindex, otheruav_pointsindex_inrender;
pcl::PointCloud<PointType> otheruav_points_vis, dyn_points_vis;
double uav_size[3];
int uav_points_num;
// int drone_num = 0;
int drone_drawpoints_num[3];

int use_uav_extra_model = 1;
pcl::PointCloud<PointType> uav_extra_model;

//dynamic objects variables
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
Eigen::Vector3f map_min,map_max;
int dyn_obs_diff_size_on = 1;
vector<double> dyn_obs_size_vec;
vector<ros::Time> dyn_start_time_vec;
PointType global_mapmin;
PointType global_mapmax;

pcl::PointCloud<PointType> dynamic_input_points;

pcl::PointCloud<PointType> cloud_all_map, local_map_filled, point_in_sensor;
pcl::PointCloud<PointType>::Ptr local_map (new pcl::PointCloud<PointType>);
// pcl::VoxelGrid<PointType> _voxel_sampler;
sensor_msgs::PointCloud2 local_map_pcd,sensor_map_pcd;

// variables for collision checks
pcl::search::KdTree<PointType> _kdtreeLocalMap, kdtree_dyn;
vector<int> pointIdxRadiusSearch;
vector<float> pointRadiusSquaredDistance;
double collision_check_time_sum = 0;
int collision_check_time_count = 0;
// hash map of uav models for collision checks
std::tr1::unordered_map<int, bool> uavpoint_hashmap;
PointType uav_modelmin;
PointType uav_modelmax;
int uavhash_xsize, uavhash_ysize, uavhash_zsize;
pcl::PointCloud<PointType> collision_points;

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
    if(use_uav_extra_model)
    {
      obs_cloud = uav_extra_model;
      for (int i = 0; i < obs_cloud.size(); i++)
      {
        obs_cloud.points[i].intensity = MIN_INTENSITY + (MAX_INTENSITY - MIN_INTENSITY) * 0.0 / 2.0;
      }      
    }else{
      obs_cloud = generate_box_cloud(dynobject_size);
    }
    break;

  case 1:
    obs_cloud = generate_sphere_cloud(dynobject_size);
    break;

  case 2:
    obs_cloud = generate_box_cloud(dynobject_size);
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
  if (dynobj_enable == 1)
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
      // for (int i = 0; i < temp_dynobj_points.size(); i++)
      // {
      //   dynobj_pointsindex.push_back(dynpt_count + origin_mapptcount);
      //   dynpt_count++;
      // }
    }

    ROS_INFO("dynobj points size = %d", dynobj_points_vis.points.size());

    dynobj_points_vis.width = dynobj_points_vis.points.size();
    dynobj_points_vis.height = 1;
    dynobj_points_vis.is_dense = true;

    // sensor_msgs::PointCloud2 dynobj_points_pcd;
    // pcl::toROSMsg(dynobj_points_vis, dynobj_points_pcd);
    // dynobj_points_pcd.header = odom_.header;
    // dynobj_points_pcd.header.frame_id = "world";
    // pub_dyncloud.publish(dynobj_points_pcd);

    kdtree_dyn.setInputCloud(dynobj_points_vis.makeShared());
    // has_dyn_map = true;

    // dyn_start_time = ros::Time::now();
  }
}

void rcvOdometryCallbck(const nav_msgs::Odometry& odom)
{
  /*if(!has_global_map)
    return;*/
  has_odom = true;
  odom_ = odom;

  Matrix4f body2world = Matrix4f::Identity();

  Eigen::Vector3f request_position;
  Eigen::Quaternionf pose;
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

  // collision check function
  // running time statistics
  ros::Time t1 = ros::Time::now();

  if(collisioncheck_enable)
  {
    collision_points.points.clear();
    PointType searchPoint;
    searchPoint.x = odom.pose.pose.position.x;
    searchPoint.y = odom.pose.pose.position.y;
    searchPoint.z = odom.pose.pose.position.z;
    if (_kdtreeLocalMap.radiusSearch(searchPoint, collision_range, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    {
      if(use_uav_extra_model)
      {
        // check if the neighbor points are in the uav model
        bool collision = false;
        for(int i = 0; i < pointIdxRadiusSearch.size(); i++)
        {
          int index = pointIdxRadiusSearch[i];
          PointType p = cloud_all_map.points[index];

          // transform the point to the uav model coordinate
          Eigen::Vector3f p_eigen(p.x, p.y, p.z);
          p_eigen = body2world.block<3, 3>(0, 0).transpose() * (p_eigen - body2world.block<3, 1>(0, 3));

          int hash_x = (p_eigen(0) - uav_modelmin.x) / downsample_res;
          int hash_y = (p_eigen(1) - uav_modelmin.y) / downsample_res;
          int hash_z = (p_eigen(2) - uav_modelmin.z) / downsample_res;
          int hash_index = hash_x + hash_y * uavhash_xsize + hash_z * uavhash_xsize * uavhash_ysize;
          if(uavpoint_hashmap.find(hash_index) != uavpoint_hashmap.end())
          {
            collision = true;
            ROS_ERROR("ENVIRONMENT COLLISION DETECTED!!!");
            // break;
            collision_points.points.push_back(p);
          }
        }
      }else{
        ROS_ERROR("ENVIRONMENT COLLISION DETECTED!!!");
      }
    }
    if(dynobj_enable || drone_num > 1)
    {
      if(kdtree_dyn.radiusSearch(searchPoint, collision_range, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
      {
        if(use_uav_extra_model)
        {
          // check if the neighbor points are in the uav model
          bool collision = false;
          for(int i = 0; i < pointIdxRadiusSearch.size(); i++)
          {
            int index = pointIdxRadiusSearch[i];
            PointType p = cloud_all_map.points[index];
            int hash_x = (p.x - uav_modelmin.x) / downsample_res;
            int hash_y = (p.y - uav_modelmin.y) / downsample_res;
            int hash_z = (p.z - uav_modelmin.z) / downsample_res;
            int hash_index = hash_x + hash_y * uavhash_xsize + hash_z * uavhash_xsize * uavhash_ysize;
            if(uavpoint_hashmap.find(hash_index) != uavpoint_hashmap.end())
            {
              collision = true;
              ROS_ERROR("DYNAMIC OBSTACLES COLLISION DETECTED!!!");
              // break;
              collision_points.points.push_back(p);
            }
          }
        }else{
          ROS_ERROR("DYNAMIC OBSTACLES COLLISION DETECTED!!!");
        }
      }      
    }
  }

  // publish collision points
  sensor_msgs::PointCloud2 collision_points_vis_pcd;
  pcl::toROSMsg(collision_points, collision_points_vis_pcd);
  collision_points_vis_pcd.header.frame_id = "world";
  collision_points_vis_pcd.header.stamp = ros::Time::now();
  pub_collisioncloud.publish(collision_points_vis_pcd);

  ros::Time t2 = ros::Time::now();
  collision_check_time_sum += (t2 - t1).toSec();
  collision_checktime_file << (t2 - t1).toSec() << endl;
  collision_check_time_count++;
  if(collision_check_time_count == 100)
  {
    ROS_INFO("collision check time: %f", collision_check_time_sum / collision_check_time_count);
    collision_check_time_sum = 0;
    collision_check_time_count = 0;
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
          // otheruav_pointsindex[drone_id].push_back(uavpt_count + origin_mapptcount + 100000 + drone_id * uav_points_num);
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
          otheruav_points_vis.push_back(temp_point);
          uavpt_count++;
    }
  }

  // publish uav point cloud
  // sensor_msgs::PointCloud2 otheruav_points_vis_pcd;
  // pcl::toROSMsg(otheruav_points_vis, otheruav_points_vis_pcd);
  // otheruav_points_vis_pcd.header.frame_id = "world";
  // otheruav_points_vis_pcd.header.stamp = ros::Time::now();
  // pub_uavcloud.publish(otheruav_points_vis_pcd);
}


int comp_time_count = 0;
void renderSensedPoints(const ros::TimerEvent& event)
{
  ros::Time t1 = ros::Time::now();

  if (!has_odom)
    return;

  Eigen::Quaternionf q;
  q.x() = odom_.pose.pose.orientation.x;
  q.y() = odom_.pose.pose.orientation.y;
  q.z() = odom_.pose.pose.orientation.z;
  q.w() = odom_.pose.pose.orientation.w;
  const Eigen::Matrix3f rot(q.toRotationMatrix());
  const Eigen::Vector3f pos(odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z);
  ros::Time time_stamp_ = odom_.header.stamp;

    ros::Time t_pattern = ros::Time::now();
    double time_frominit = (t_pattern - t_init).toSec();

    local_map->points.clear();
    dynamic_input_points.points.clear();

    for(int i = 0;i < drone_num;i++)
    {
      if(i == drone_id)
      {continue;}
      for(int j = 0;j < otheruav_points[i].size();j++)
      {
        dynamic_input_points.points.push_back(otheruav_points[i][j]);
      }
      // dynamic_input_points = dynamic_input_points + otheruav_points[i];
    }
    dynamic_input_points = dynamic_input_points + dynobj_points_vis; 

    if(dynamic_input_points.points.size() > 0)
    {
        kdtree_dyn.setInputCloud(dynamic_input_points.makeShared());      
    }

    // trans and publish the dynamic point cloud
    sensor_msgs::PointCloud2 dynamic_input_points_pcd;
    pcl::toROSMsg(dynamic_input_points, dynamic_input_points_pcd);
    dynamic_input_points_pcd.header.frame_id = "world";
    dynamic_input_points_pcd.header.stamp = ros::Time::now();
    pub_dyncloud.publish(dynamic_input_points_pcd);

    // dynamic_input_points = otheruav_points_vis + dyn_points_vis; 
    // ROS_INFO("dynamic_input_points size = %d", dynamic_input_points.size());
    render.input_dyn_clouds(dynamic_input_points);

    render.render_pointcloud(local_map,pos,q,time_frominit);

    ros::Time t_afterrender = ros::Time::now();    
    double comp_time_temp = (t_afterrender - t1).toSec();
    myfile << comp_time_temp << endl;
    comp_time_vec.push_back(comp_time_temp);
    if(comp_time_count > 20)
    {
      comp_time_vec.pop_front();
      geometry_msgs::PoseStamped totaltime_pub;
      totaltime_pub.pose.position.x = accumulate(comp_time_vec.begin(),comp_time_vec.end(),0.0)/comp_time_vec.size();
      comp_time_pub.publish(totaltime_pub);
      ROS_INFO("Temp compute time = %lf, average compute time = %lf", comp_time_temp,totaltime_pub.pose.position.x);
    }else{
      comp_time_count++;
    }

    local_map->width = local_map->points.size();
    local_map->height = 1;
    local_map->is_dense = true;

    // std::cout<< "local map size: " << local_map->points.size() << std::endl;

    pcl::toROSMsg(*local_map, local_map_pcd);
    local_map_pcd.header = odom_.header;
    local_map_pcd.header.stamp = time_stamp_;
    local_map_pcd.header.frame_id = "world";
    pub_cloud.publish(local_map_pcd);

    Eigen::Matrix4f sensor2world;
      sensor2world <<
            rot(0,0), rot(0,1), rot(0,2), pos.x(),
            rot(1,0),rot(1,1), rot(1,2), pos.y(),
            rot(2,0), rot(2,1),rot(2,2), pos.z(),
            0,0,0,1;
    Eigen::Matrix4f world2sensor;
    world2sensor = sensor2world.inverse();

    //body frame pointcloud
    point_in_sensor.points.clear();
    // pcl::copyPointCloud(local_map_filled, point_in_sensor);
    pcl::transformPointCloud(*local_map, point_in_sensor, world2sensor);
    point_in_sensor.width = point_in_sensor.points.size();
    point_in_sensor.height = 1;
    point_in_sensor.is_dense = true;

    std::string sensor_frame_id_ = "/sensor";
    pcl::toROSMsg(point_in_sensor, sensor_map_pcd);
    sensor_map_pcd.header.frame_id = sensor_frame_id_;
    sensor_map_pcd.header.stamp = time_stamp_;
    pub_intercloud.publish(sensor_map_pcd);

    
}

int main(int argc, char** argv)
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
  nh.getParam("livox_linestep",livox_linestep);
  nh.getParam("use_avia_pattern",use_avia_pattern);
  nh.getParam("curvature_limit",curvature_limit);
  nh.getParam("hash_cubesize", hash_cubesize);
  nh.getParam("use_vlp32_pattern",use_vlp32_pattern);
  nh.getParam("use_minicf_pattern",use_minicf_pattern);
  nh.getParam("use_os128_pattern",use_os128_pattern);
  nh.getParam("use_inf_pt",use_inf_pt);

  nh.getParam("use_gaussian_filter",use_gaussian_filter);

  //dyn parameters
  nh.getParam("dynobj_enable", dynobj_enable);
  nh.getParam("dynobject_size", dynobject_size);
  nh.getParam("dynobject_num", dynobject_num);
  nh.getParam("dyn_mode", dyn_mode);
  nh.getParam("dyn_velocity", dyn_velocity);

  nh.getParam("use_uav_extra_model", use_uav_extra_model);

  nh.getParam("collisioncheck_enable", collisioncheck_enable);
  nh.getParam("collision_range", collision_range);

  nh.getParam("output_pcd", output_pcd);

  nh.getParam("map/x_size", x_size);
  nh.getParam("map/y_size", y_size);
  nh.getParam("map/z_size", z_size);

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
  }


  // // UAV_odom_sub = nh.subscribe("/other_quad/lidar_slam/odom", 1000, UAVOdomCallback);

  uav_size[0] = 0.4;
  uav_size[1] = 0.4;
  uav_size[2] = 0.3;
  drone_drawpoints_num[0] = (uav_size[0] / downsample_res);
  drone_drawpoints_num[1] = (uav_size[1] / downsample_res);
  drone_drawpoints_num[2] = (uav_size[2] / downsample_res);
  uav_points_num = drone_drawpoints_num[0] * drone_drawpoints_num[1] * drone_drawpoints_num[2];
  ROS_INFO("drone_drawpoints_num = %d,%d,%d, Uav points num = %d", drone_drawpoints_num[0], drone_drawpoints_num[1], drone_drawpoints_num[2], uav_points_num);

  file_name = argv[1];
  // render.setParameters(400,400,250,250,downsample_res,0.1,sensing_horizon,sensing_rate,use_avia_pattern);
  int image_width;
  int image_height;
  if(is_360lidar)
  {
    image_width = ceil(360.0/polar_resolution);
  }else{
    image_width = ceil(yaw_fov/polar_resolution);
  }
  image_height = ceil(vertical_fov/polar_resolution);
  render.setParameters(image_width,image_height,250,250,downsample_res,polar_resolution,
    yaw_fov,vertical_fov,0.1,sensing_horizon,sensing_rate,use_avia_pattern,use_os128_pattern,
    use_minicf_pattern,use_inf_pt);

  render.read_pointcloud_fromfile(file_name);
  //home/dji/kong_ws/src/Exploration_sim/uav_simulator/map_generator/resource/Knowles_merge_01cutoff.pcd
// /home/dji/meshmap/Knowles_local_sor_001.pcd

  if(collisioncheck_enable)
  {
    // read pcd map and input kdtree map for collision check
      pcd_read_status = pcl::io::loadPCDFile<PointType>(file_name, cloud_all_map);
      if (pcd_read_status == -1)
      {
          cout << "can't read global" << endl;
          return 0;
      }

    ROS_INFO("Start to input kdtree. Enable high resolution collistion check may take one minute to initialize");

    // downsample map
    // pcl::VoxelGrid<PointType> sor;
    sor.setInputCloud(cloud_all_map.makeShared());
    sor.setLeafSize(downsample_res, downsample_res, downsample_res);
    sor.filter(cloud_all_map);

    pcl::getMinMax3D(cloud_all_map, global_mapmin, global_mapmax);

    //intput kdtree
    _kdtreeLocalMap.setInputCloud(cloud_all_map.makeShared());
    // cloud_all_map.points.clear();

    ROS_INFO("Global map read and input kdtree done.");    

    if(use_uav_extra_model)
    {
      // get max and min xyz of uav model
      pcl::getMinMax3D(uav_extra_model, uav_modelmin, uav_modelmax);
      uavhash_xsize = ceil((uav_modelmax.x - uav_modelmin.x) / downsample_res);
      uavhash_ysize = ceil((uav_modelmax.y - uav_modelmin.y) / downsample_res);
      uavhash_zsize = ceil((uav_modelmax.z - uav_modelmin.z) / downsample_res);

      // input the uav model point clouds into voxel grid hash map
      for(int i = 0; i < uav_extra_model.points.size(); i++)
      {
        PointType p = uav_extra_model.points[i];
        int x = floor((p.x - uav_modelmin.x) / downsample_res);
        int y = floor((p.y - uav_modelmin.y) / downsample_res);
        int z = floor((p.z - uav_modelmin.z) / downsample_res);
        long int hash = x + y * uavhash_xsize + z * uavhash_xsize * uavhash_ysize;
        uavpoint_hashmap[hash] = true;
      }

      ROS_INFO("UAV model read and input hash map done. UAV model size = %d,%d,%d", uavhash_xsize, uavhash_ysize, uavhash_zsize);
    }
  }

  if(dynobj_enable == 1)
  {
    if(!collisioncheck_enable)
    {
      pcd_read_status = pcl::io::loadPCDFile<PointType>(file_name, cloud_all_map);
      if (pcd_read_status == -1)
      {
          cout << "can't read global" << endl;
          return 0;
      }
      pcl::getMinMax3D(cloud_all_map, global_mapmin, global_mapmax);
    }

    map_min(0) = global_mapmin.x;
    map_min(1) = global_mapmin.y;
    map_min(2) = global_mapmin.z;
    map_max(0) = global_mapmax.x;
    map_max(1) = global_mapmax.y;
    map_max(2) = global_mapmax.z;

    // ROS_INFO("Global map min x = %f, max x = %f" , global_mapmin.x , global_mapmax.x);

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

      // dyn_start_time = ros::Time::now();
      dyn_start_time_vec.push_back(ros::Time::now());      
    }
  }

  // subscribe point cloud
//   global_map_sub = nh.subscribe("global_map", 1, rcvGlobalPointCloudCallBack);
  odom_sub = nh.subscribe("odometry", 50, rcvOdometryCallbck);

  // publisher depth image and color image
//   pub_dyncloud = nh.advertise<sensor_msgs::PointCloud2>("/pcl_render_node/dyn_cloud", 10);
  // pub_intercloud = nh.advertise<sensor_msgs::PointCloud2>("/pcl_render_node/explored_cloud", 10);
  pub_dyncloud = nh.advertise<sensor_msgs::PointCloud2>("dyn_cloud", 10);
  pub_intercloud = nh.advertise<sensor_msgs::PointCloud2>("sensor_cloud", 10);  
  pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("cloud", 10);
  pub_pose = nh.advertise<geometry_msgs::PoseStamped>("sensor_pose", 10);
  pub_uavcloud = nh.advertise<sensor_msgs::PointCloud2>("uav_cloud", 10);
  depth_img_pub_ = nh.advertise<sensor_msgs::Image>("depth_img", 10);
  comp_time_pub = nh.advertise<geometry_msgs::PoseStamped>("simulator_compute_time", 10);
  pub_collisioncloud = nh.advertise<sensor_msgs::PointCloud2>("collision_cloud", 10);
  double sensing_duration = 1.0 / sensing_rate;
  double estimate_duration = 1.0 / estimation_rate;

  t_init = ros::Time::now();

  local_sensing_timer = nh.createTimer(ros::Duration(sensing_duration), renderSensedPoints);
//   pose_timer = nh.createTimer(ros::Duration(estimate_duration), pubSensorPose);
  dynobj_timer = nh.createTimer(ros::Duration(sensing_duration), dynobjGenerate);

//     pkg_path = ros::package::getPath("octomap_server");
//     pkg_path.append("/data/explore_persentage.txt");
//     std::cout << "\nFound pkg_path = " << pkg_path << std::endl;
//     myfile.open(pkg_path.c_str(), std::ios_base::out);//, std::ios_base::out

  pkg_path = ros::package::getPath("local_sensing_node");  
  pkg_path.append("/data/" + quad_name + "_GPU_time_consumption.txt");
  std::cout << "\nFound pkg_path = " << pkg_path << std::endl;
  myfile.open(pkg_path.c_str(), std::ios_base::out); 

  // open file to record collision check time consumption
  pkg_path = ros::package::getPath("local_sensing_node");
  pkg_path.append("/data/" + quad_name + "_GPU_collision_check_time_consumption.txt");
  std::cout << "\nFound pkg_path = " << pkg_path << std::endl;
  collision_checktime_file.open(pkg_path.c_str(), std::ios_base::out);


//   sensor2body << 0.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

  ros::Rate rate(100);
  bool status = ros::ok();
  // rate.sleep();

  // // use asyncspinner
  // ros::AsyncSpinner spinner(3);
  // spinner.start();

  // // use multithread spinner
  // ros::MultiThreadedSpinner spinner(4);
  // spinner.spin();

  while (status)
  {
    ros::spinOnce();
    status = ros::ok();
    rate.sleep();
  }

  // ros::spin();

//   if(output_pcd==1)
//   {
//     //write pcd
//     pkg_path = ros::package::getPath("octomap_server");
//     std::string pcd_name("cloud_explored");
//     if(pcl::io::savePCDFileASCII (pkg_path+"/data/"+pcd_name+".pcd", local_map)>=0)
//     {std::cerr << "Saved  " << pcd_name<<".pcd"<< std::endl;}    
//   }

  return 0;

}
