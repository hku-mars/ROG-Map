#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/common/common.h>

// #include <pcl/point_types.h>
#include <pcl/io/io.h>
// #include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/search/impl/kdtree.hpp>
#include <ros/package.h>
#include <vector>

typedef pcl::PointXYZ PointT;

using namespace std;
string file_name;

int add_boundary = 0;
int is_bridge = 0;
double downsample_res;
double map_offset_x, map_offset_y, map_offset_z;

int minus_twopointcloud(pcl::PointCloud<pcl::PointXYZ> &cloud_input, pcl::PointCloud<pcl::PointXYZ> &cloud_input2,
                        pcl::PointCloud<pcl::PointXYZ> &cloud_output) {
    //cloud_input minus cloud_input2
    pcl::search::KdTree<pcl::PointXYZ> minus_kdtree;
    minus_kdtree.setInputCloud(cloud_input2.makeShared());

    vector<int> pointIdxRadiusSearch;
    vector<float> pointRadiusSquaredDistance;
    for (int i = 0; i < cloud_input.points.size(); i++) {
        if (minus_kdtree.radiusSearch(cloud_input.points[i], 0.6, pointIdxRadiusSearch, pointRadiusSquaredDistance) >
            0) {
            continue;
        }
        cloud_output.points.push_back(cloud_input.points[i]);
    }

    ROS_INFO("AFTER MINUS, points count = %d", cloud_output.points.size());

    return cloud_output.points.size();

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "map_recorder");
    ros::NodeHandle node("~");

    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
    node.getParam("add_boundary", add_boundary);
    node.getParam("is_bridge", is_bridge);
    node.getParam("downsample_res", downsample_res);

    node.getParam("map_offset_x", map_offset_x);
    node.getParam("map_offset_y", map_offset_y);
    node.getParam("map_offset_z", map_offset_z);

    ros::Publisher cloud_pub = node.advertise<sensor_msgs::PointCloud2>("/map_generator/global_cloud", 10, true);
    file_name = argv[1];

    ros::Duration(1.0).sleep();

    /* load cloud from pcd */
    pcl::PointCloud<pcl::PointXYZ> cloud_temp, cloud;
    int status = pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, cloud_temp);
    cloud = cloud_temp;
//  cloud.clear();
//  for (int i = 0; i < cloud_temp.size(); i+=5) {
//      Eigen::Vector3d pt(cloud_temp.points[i].x, cloud_temp.points[i].y, cloud_temp.points[i].z);
//      cloud.points.push_back(cloud_temp.points[i]);
//  }

    if (status == -1) {
        cout << "can't read file." << endl;
        return -1;
    }

    // string file_name2 = "/home/jackykong/motionplanning/FUEL_ws/src/FUEL/meshmap/hku_demo_color.ply";
    // pcl::PLYReader reader;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ply;
    // reader.read<pcl::PointXYZ>(file_name2,*cloud_ply);
    // pcl::io::savePCDFile("/home/jackykong/motionplanning/FUEL_ws/src/FUEL/meshmap/hku_demo_pcd.pcd",*cloud_ply);

    ROS_INFO("SUCCESS LOAD PCD FILE");

    //filter
    pcl::VoxelGrid<pcl::PointXYZ> _voxel_sampler;
    // _voxel_sampler.setLeafSize(downsample_res, downsample_res, downsample_res);
    // _voxel_sampler.setInputCloud(cloud.makeShared());
    // _voxel_sampler.filter(cloud);

    // //remove outlier points, like humans
    // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    // sor.setInputCloud (cloud.makeShared());
    // sor.setMeanK (50);
    // sor.setStddevMulThresh (1.0);
    // sor.filter (cloud);

    if (is_bridge == 1) {
        // Process map
        for (int i = 0; i < cloud.points.size(); ++i) {
            auto pt = cloud.points[i];
            pcl::PointXYZ pr;
            pr.x = pt.x;
            pr.y = -pt.z;
            pr.z = pt.y;
            cloud.points[i] = pr;
        }
    }

    // // Process map
    // for (int i = 0; i < cloud.points.size(); ++i)
    // {
    //   auto pt = cloud.points[i];
    //   pcl::PointXYZ pr;
    //   pr.x = pt.x - 40;
    //   pr.y = pt.y - 100;
    //   pr.z = pt.z - 80;
    //   cloud.points[i] = pr;
    // }

    pcl::PointXYZ global_mapmin;
    pcl::PointXYZ global_mapmax;

    pcl::getMinMax3D(cloud, global_mapmin, global_mapmax);

    ROS_INFO("Map bound: x=%f,%f, y=%f,%f, z=%f,%f", global_mapmin.x, global_mapmax.x, global_mapmin.y, global_mapmax.y,
             global_mapmin.z, global_mapmax.z);

    if (add_boundary == 1) {
        // for (double x = -10; x <= 10; x += 0.05)
        //   for (double y = -10; y <= 10; y += 0.05)
        //   {
        //     cloud.push_back(pcl::PointXYZ(x, y, 0));
        //   }

        //add boundary
        pcl::PointCloud<pcl::PointXYZ> cloud_boundary;
        int add_length = 1;
        int max_x = (int) global_mapmax.x + add_length;
        int min_x = (int) global_mapmin.x - add_length;
        int max_y = (int) global_mapmax.y + add_length;
        int min_y = (int) global_mapmin.y - add_length;
        int max_z = (int) global_mapmax.z + add_length;
        int min_z = (int) global_mapmin.z - add_length;
        int box_x = (max_x - min_x) * 10;
        int box_y = (max_y - min_y) * 10;
        int box_z = (max_z - min_z) * 10;
        //draw six plane
        cloud_boundary.width =
                (box_x + 1) * (box_y + 1) * (box_z + 1) - (box_x - 1) * (box_y - 1) * (box_z - 1);//points number
        cloud_boundary.height = 1;
        cloud_boundary.points.resize(cloud_boundary.width * cloud_boundary.height);
        int point_count = 0;
        //draw 2 xy plane
        for (float i = min_x; i <= max_x; i = i + 0.1) {
            for (float j = min_y; j <= max_y; j = j + 0.1) {
                // cloud_boundary.points[point_count].x = i;
                // cloud_boundary.points[point_count].y = j;
                // cloud_boundary.points[point_count].z = min_z;
                // point_count++;
                // cloud_boundary.points[point_count].x = i;
                // cloud_boundary.points[point_count].y = j;
                // cloud_boundary.points[point_count].z = max_z;
                // point_count++;
                cloud.push_back(pcl::PointXYZ(i, j, min_z));
                cloud.push_back(pcl::PointXYZ(i, j, max_z));
            }
        }
        //draw 4 plane
        for (float k = min_z; k <= max_z; k = k + 0.1) {
            //draw x two lines
            for (float i = min_x; i <= max_x; i = i + 0.1) {
                cloud_boundary.points[point_count].x = i;
                cloud_boundary.points[point_count].y = min_y;
                cloud_boundary.points[point_count].z = k;
                point_count++;
                cloud_boundary.points[point_count].x = i;
                cloud_boundary.points[point_count].y = max_y;
                cloud_boundary.points[point_count].z = k;
                point_count++;
                cloud_boundary.points[point_count].x = i;
                cloud_boundary.points[point_count].y = min_y - 0.1;
                cloud_boundary.points[point_count].z = k;
                point_count++;
                cloud_boundary.points[point_count].x = i;
                cloud_boundary.points[point_count].y = max_y + 0.1;
                cloud_boundary.points[point_count].z = k;
                point_count++;
            }
            for (float j = min_y; j <= max_y; j = j + 0.1) {
                cloud_boundary.points[point_count].x = min_x;
                cloud_boundary.points[point_count].y = j;
                cloud_boundary.points[point_count].z = k;
                point_count++;
                cloud_boundary.points[point_count].x = max_x;
                cloud_boundary.points[point_count].y = j;
                cloud_boundary.points[point_count].z = k;
                point_count++;
                cloud_boundary.points[point_count].x = min_x - 0.1;
                cloud_boundary.points[point_count].y = j;
                cloud_boundary.points[point_count].z = k;
                point_count++;
                cloud_boundary.points[point_count].x = max_x + 0.1;
                cloud_boundary.points[point_count].y = j;
                cloud_boundary.points[point_count].z = k;
                point_count++;
            }
        }

        cloud = cloud_boundary + cloud;

        ROS_INFO("ADD BOUNDARY!!!");
    }

    // pcl::VoxelGrid<pcl::PointXYZ> _voxel_sampler;
    if (downsample_res > 0) {
        _voxel_sampler.setLeafSize(downsample_res, downsample_res, downsample_res);
        _voxel_sampler.setInputCloud(cloud.makeShared());
        _voxel_sampler.filter(cloud);
    }


    // Process map
    for (int i = 0; i < cloud.points.size(); ++i) {
        auto pt = cloud.points[i];
        pcl::PointXYZ pr;
        pr.x = pt.x + map_offset_x;
        pr.y = pt.y + map_offset_y;
        pr.z = pt.z + map_offset_z;
        cloud.points[i] = pr;
    }

    //trans to mesh
    // 对点云重采样
    // pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    // cloud_filtered = cloud.makeShared();
    // pcl::search::KdTree<PointT>::Ptr treeSampling (new pcl::search::KdTree<PointT>);// 创建用于最近邻搜索的KD-Tree
    // pcl::PointCloud<PointT> mls_point;    //输出MLS
    // pcl::MovingLeastSquares<PointT,PointT> mls; // 定义最小二乘实现的对象mls
    // mls.setComputeNormals(false);  //设置在最小二乘计算中是否需要存储计算的法线
    // mls.setInputCloud(cloud_filtered);         //设置待处理点云
    // mls.setPolynomialOrder(2);            // 拟合2阶多项式拟合
    // mls.setPolynomialFit(false);     // 设置为false可以 加速 smooth
    // mls.setSearchMethod(treeSampling);         // 设置KD-Tree作为搜索方法
    // mls.setSearchRadius(0.05);           // 单位m.设置用于拟合的K近邻半径
    // mls.process(mls_point);                 //输出

    //   // 输出重采样结果
    //   pcl::PointCloud<PointT>::Ptr cloud_smoothed(new pcl::PointCloud<PointT>);
    //   // cloud_smoothed = mls_point.makeShared();
    //   cloud_smoothed = cloud.makeShared();
    //   std::cout<<"cloud_smoothed: "<<cloud_smoothed->size() <<std::endl;
    //   //save cloud_smoothed
    //   // pcl::io::savePCDFileASCII("/home/xiaohu/learn_SLAM/zuoye15/作业15-点云平滑及法线估计及显示/data/cloud_smoothed.pcd",*cloud_smoothed);

    //   pcl::VoxelGrid<PointT> downSampled;  //创建滤波对象
    //   downSampled.setInputCloud (cloud_smoothed);            //设置需要过滤的点云给滤波对象
    //   downSampled.setLeafSize (0.01f, 0.01f, 0.01f);  //设置滤波时创建的体素体积为1cm的立方体
    //   downSampled.filter (*cloud_smoothed);           //执行滤波处理，存储输出

    //   // 法线估计
    //   pcl::NormalEstimation<PointT,pcl::Normal> normalEstimation;             //创建法线估计的对象
    //   normalEstimation.setInputCloud(cloud_smoothed);                         //输入点云
    //   pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);// 创建用于最近邻搜索的KD-Tree
    //   normalEstimation.setSearchMethod(tree);
    //   pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>); // 定义输出的点云法线
    //   // K近邻确定方法，使用k个最近点，或者确定一个以r为半径的圆内的点集来确定都可以，两者选1即可
    //   normalEstimation.setKSearch(20);// 使用当前点周围最近的10个点
    //   //normalEstimation.setRadiusSearch(0.03);            //对于每一个点都用半径为3cm的近邻搜索方式
    //   normalEstimation.compute(*normals);               //计算法线
    //   // 输出法线
    //   std::cout<<"normals: "<<normals->size()<<", "<<"normals fields: "<<pcl::getFieldsList(*normals)<<std::endl;
    //   // pcl::io::savePCDFileASCII("/home/xiaohu/learn_SLAM/zuoye15/作业15-点云平滑及法线估计及显示/data/normals.pcd",*normals);

    // 	// 将点云位姿、颜色、法线信息连接到一起
    //   pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    //   pcl::concatenateFields(*cloud_smoothed, *normals, *cloud_with_normals);
    //   // pcl::io::savePCDFileASCII("/home/xiaohu/learn_SLAM/zuoye15/作业15-点云平滑及法线估计及显示/data/cloud_with_normals.pcd",*cloud_with_normals);

    // // 贪心投影三角化
    //   //定义搜索树对象
    //   pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    //   tree2->setInputCloud(cloud_with_normals);

    //   // // 三角化
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;   // 定义三角化对象
    pcl::PolygonMesh triangles; //存储最终三角化的网络模型

    //   // 设置三角化参数
    //   gp3.setSearchRadius(0.1);  //设置搜索时的半径，也就是KNN的球半径
    //   gp3.setMu (2.5);  //设置样本点搜索其近邻点的最远距离为2.5倍（典型值2.5-3），这样使得算法自适应点云密度的变化
    //   gp3.setMaximumNearestNeighbors (100);    //设置样本点最多可搜索的邻域个数，典型值是50-100

    //   gp3.setMinimumAngle(M_PI/18); // 设置三角化后得到的三角形内角的最小的角度为10°
    //   gp3.setMaximumAngle(2*M_PI/3); // 设置三角化后得到的三角形内角的最大角度为120°

    //   gp3.setMaximumSurfaceAngle(M_PI/4); // 设置某点法线方向偏离样本点法线的最大角度45°，如果超过，连接时不考虑该点
    //   gp3.setNormalConsistency(false);  //设置该参数为true保证法线朝向一致，设置为false的话不会进行法线一致性检查

    //   gp3.setInputCloud (cloud_with_normals);     //设置输入点云为有向点云
    //   gp3.setSearchMethod (tree2);   //设置搜索方式
    //   gp3.reconstruct (triangles);  //重建提取三角化
    //   pcl::io::saveOBJFile("/home/jackykong/motionplanning/FUEL_ws/src/Exploration_sim/uav_simulator/map_generator/resource/result.obj", triangles);
    //   std::cout<<" Out put finished"<<std::endl;

    // string objPath = "/home/jackykong/motionplanning/FUEL_ws/src/Exploration_sim/uav_simulator/map_generator/resource/result.obj"; //当前目录下的obj文件
    // //读取
    // pcl::PolygonMesh mesh;
    // pcl::io::loadPolygonFileOBJ(objPath, triangles);

    // // 显示网格化结果
    // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    // viewer->setBackgroundColor(0, 0, 0);  //
    // viewer->addPolygonMesh(triangles, "wangge");  //

    // cout << "Publishing map..." << endl;

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.frame_id = "world";
    ROS_INFO("Map point size = %d", cloud.points.size());

    //write files
    // std::string pkg_path("/home/jackykong/motionplanning/FUEL_ws/src/Exploration_sim/uav_simulator/map_generator/resource");
    // std::string pcd_name("watertre03_cutoff");
    // if(pcl::io::savePCDFileASCII (pkg_path+pcd_name+".pcd", cloud)>=0)
    // {std::cerr << "Saved  " << pcd_name<<".pcd"<< std::endl;} 

    // ros::Publisher minus_cloud_pub = node.advertise<sensor_msgs::PointCloud2>("/minus_cloud", 10, true);
    // pcl::PointCloud<pcl::PointXYZ> cloud_minus, cloud_output;
    // status = pcl::io::loadPCDFile<pcl::PointXYZ>("/home/jackykong/motionplanning/FUEL_ws/src/Exploration_sim/octomap_mapping/octomap_server/data/FUEL_WATERTRE03NEW2.pcd", cloud_minus);
    // if (status == -1)
    // {
    //   cout << "can't read file." << endl;
    //   return -1;
    // }
    // minus_twopointcloud(cloud,cloud_minus,cloud_output);
    // sensor_msgs::PointCloud2 minus_msg;
    // pcl::toROSMsg(cloud_output, minus_msg);
    // minus_msg.header.frame_id = "world";

    int count = 0;
    while (ros::ok())//!viewer->wasStopped()
    {

        // viewer->spinOnce(100);
        // boost::this_thread::sleep(boost::posix_time::microseconds(100000));

        ros::Duration(1.0).sleep();
        cloud_pub.publish(msg);
        // minus_cloud_pub.publish(minus_msg);
        ++count;
        if (count > 10) {
            break;
        }
    }
    cout << "finish publish map." << endl;

    return 0;
}