#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>

// #include <pangolin/var/var.h>
// #include <pangolin/var/varextra.h>
// // #include <pangolin/gl/gl.h>
// // #include <pangolin/gl/gldraw.h>
// #include <pangolin/display/display.h>
// #include <pangolin/display/view.h>
// #include <pangolin/display/widgets.h>
// #include <pangolin/display/default_font.h>
// #include <pangolin/handler/handler.h>

// #include <GL/glew.h>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <unistd.h>
#include <shader_m.h>

// #include "tools/tools_color_printf.hpp"
// #include "tools/tools_data_io.hpp"
// #include "tools/tools_logger.hpp"
// #include "tools/tools_eigen.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <fstream>
#include <chrono>
#include <vector>
#include <string>
#include <numeric>

#include "FOV_Checker/FOV_Checker.h"
#include <tr1/unordered_map>

using namespace std::chrono;
using namespace std;
using namespace cv;
using PointType = pcl::PointXYZI;

void framebuffer_size_callback(GLFWwindow* window, int screen_width, int screen_height)
{
    // make sure the viewport matches the new window dimensions; note that width and 
    // height will be significantly larger than specified on retina displays.
    glViewport(0, 0, screen_width, screen_height);
}

class opengl_pointcloud_render
{
    public:
    opengl_pointcloud_render(){};
    void read_pointcloud_fromfile(std::string map_filename);
    ~opengl_pointcloud_render();

    void setParameters(int width, int height, float fx, float fy, float downsample_res, float polar_res_, float yaw_fov_,\
                 float vertical_fov_,float near,float far,int sensing_rate,
                 int use_avia_pattern, int use_os128_pattern, int use_minicf_pattern,
                 int _use_inf_point = false);

    void render_pointcloud(pcl::PointCloud<PointType>::Ptr output_pointcloud, Eigen::Vector3f camera_pos, Eigen::Quaternionf camera_q, double t_pattern_start);
    void input_dyn_clouds(pcl::PointCloud<pcl::PointXYZI> input_cloud);

    private:
    bool use_inf_pt{false};
    int UI_WIDTH;
    int width = 350;
    int height = 350;
    float vertical_fov = 77.4;
    float yaw_fov = 70.4;
    float near = 0.1; 
    float far  = 30.0; 
    float fu = 350;
    float fv = 350;
    float u0 = width*0.5;
    float v0 = width*0.5;
    glm::mat4 projection, view;
    unsigned int VBO, VAO, EBO;
    GLFWwindow* window;
    Shader ourShader;

    int use_avia_pattern = 0;
    int use_os128_pattern = 0;
    int use_minicf_pattern = 0;
    int sensing_rate = 10;
    Eigen::MatrixXf pattern_matrix;
    Eigen::MatrixXf density_matrix;
    cv::Mat density_mat;

    double hash_cubesize = 5.0;
    FOV_Checker fov_checker;
    BoxPointType env_box;
    std::tr1::unordered_map<int, std::vector<PointType>> point_hashmap;
    std::tr1::unordered_map<int, std::vector<int>> pointindex_hashmap;
    int cube_numx,cube_numy,cube_numz;

    float downsample_res = 0.01;
    float polar_res = 0.2;
    float cover_dis = 0.55 * 1.7321 * downsample_res;
    float polar_resolution_max = 1.0/fu*(cos(1)*cos(1))*180/M_PI;;
    float effect_range = cover_dis / sin(0.5 * polar_resolution_max * M_PI / 180.0);;
    Eigen::Matrix3f camera2world, body2world;
    Eigen::Vector3f camera;
    Eigen::Vector3f camera_pos_world;
    std::vector<std::vector<Eigen::Matrix<float, 3, 1> > > interline_ptcloud_vec;
    pcl::PointCloud<pcl::Normal>::Ptr pointcloud_normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);

    pcl::PolygonMesh             mesh, mesh_obj;
    pcl::PointCloud< PointType > cloud_color_mesh;
    pcl::PointCloud< PointType >::Ptr pcl_pc_rgb;
    pcl::PointCloud< PointType >::Ptr render_cloud = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
    std::vector<Eigen::Matrix<float, 3, 1> > depth_ptcloud_vec;
    std::vector<float>          g_pointcloud_vec;
    int                         g_total_point_size;
    int g_point_step = 3;
    std::vector<Eigen::Matrix<float, 3, 1> > g_eigen_pt_vec, dyn_clouds_vec;
    std::vector<Eigen::Matrix<float, 3, 1>> g_eigen_rgb_vec;
    std::vector<Eigen::Matrix<float, 3, 1> > g_eigen_tri_pt_vec;
    std::vector<Eigen::Matrix<unsigned char, 3, 1>> g_eigen_tri_rgb_vec;
    std::vector<unsigned int> points_index_infov, dyn_clouds_index;

    pcl::PointCloud< pcl::PointXYZI > dyn_clouds;

    // pangolin::OpenGlRenderState s_cam;
    // pangolin::OpenGlRenderState s_cam2;
    // pangolin::View d_cam (1.0);
    // pangolin::View d_cam2 (1.0);

    struct BGR
    {
        uchar b;
        uchar g;
        uchar r;
    };
    
    struct HSV
    {
        int h;
        double s;
        double v;
    };

    float LinearizeDepth(float depth,float near, float far);
    float regainrealdepth(float depth,float near, float far);
    void test_vector_insert();
    void get_Rendering_info();
    void printf_pointcloud2_infos(pcl::PCLPointCloud2 & v);
    void load_pcd_file(std::string file_name, pcl::PointCloud<PointType>& cloud_color_mesh);
    void load_normal_file(std::string file_name, pcl::PointCloud<pcl::Normal>& normal_ptcloud);
    inline void glDrawColouredCube_atrandompos(Eigen::Vector3f pos, double length);
    void HSV2BGR(HSV &hsv, BGR &bgr);
    void init_pointcloud_data();
    inline void glDrawColouredCube_alpha(GLfloat axis_min, GLfloat axis_max, float alpha);
    template<typename T>
    void get_real_near_far( cv::Mat  & depth_mat , double & min, double & max );
    void culling_depth_image( cv::Mat  & depth_mat);
    inline bool is_in_triangle(Eigen::Vector3f p, Eigen::Vector3f a, Eigen::Vector3f b, Eigen::Vector3f c, Eigen::Vector3f& weights);
    void interlinebytriangles(cv::Mat& depth_image);
    bool is_in_fov(Eigen::Vector3f pt, Eigen::Vector3f pos, Eigen::Vector3f fov_direction, double fov_angle);
    int count_fov_point_num(pcl::PointCloud< PointType > cloud_all_map, Eigen::Vector3f pos, Eigen::Vector3f fov_direction,\
                         double sensing_range, double half_fov_angle);
    void preprocess(pcl::PointCloud< PointType > cloud_all_map, pcl::PointCloud<pcl::Normal>::Ptr all_normals);
    void depth_interline_on_depthimage(cv::Mat& depth_image);
    void render_dynclouds_on_depthimage(cv::Mat& depth_image);
    void depth_to_pointcloud(cv::Mat& depth_image, pcl::PointCloud<PointType>::Ptr origin_cloud, \
                        std::vector<Eigen::Matrix<float, 3, 1> >& depth_ptcloud_vec,\
                        vector<vector<Eigen::Matrix<float, 3, 1> > >& interline_ptcloud_vec);
    inline cv::Mat colormap_depth_img(cv::Mat & depth_mat);
    void read_depth( int depth_buffer_precision , std::vector<Eigen::Matrix<float, 3, 1> >& depth_ptcloud_vec);
    void new_gaussian_interline(cv::Mat  & depth_mat);
    void faces_need_draw_from_depthimage(cv::Mat& depth_image, cv::Mat color_image);
    void read_depth2pointcloud( int depth_buffer_precision , std::vector<Eigen::Matrix<float, 3, 1> >& depth_ptcloud_vec);
    // void framebuffer_size_callback(GLFWwindow* window, int screen_width, int screen_height);

};

void opengl_pointcloud_render::setParameters(int width, int height, float fx, float fy, float downsample_res, float polar_res_, float yaw_fov_, float vertical_fov_,\
                                    float near,float far,int sensing_rate,int use_avia_pattern, int use_os128_pattern, int use_minicf_pattern,
                                    int _use_inf_point)
{
    this->use_inf_pt = _use_inf_point;
    this->width = width;
    this->height = height;
    this->fu = fx;
    this->fv = fy;
    this->u0 = width*0.5;
    this->v0 = height*0.5;
    this->near = near;
    this->far = far;
    this->yaw_fov = yaw_fov_;
    this->vertical_fov = vertical_fov_;
    this->downsample_res = downsample_res;
    this->polar_res = polar_res_;
    this->camera2world = Eigen::Matrix3f::Identity();
    this->camera = Eigen::Vector3f(0,0,0);
    this->camera_pos_world = Eigen::Vector3f(0,0,0);
    this->cover_dis = 0.55 * 1.7321 * downsample_res;
    this->polar_resolution_max = 1.0/fu*(cos(1)*cos(1))*180/M_PI;
    this->effect_range = cover_dis / sin(0.5 * polar_res * M_PI / 180.0);
    this->sensing_rate = sensing_rate;
    this->use_avia_pattern = use_avia_pattern;
    this->use_os128_pattern = use_os128_pattern;
    this->use_minicf_pattern = use_minicf_pattern;

    if(use_avia_pattern==1 || use_os128_pattern == 1 || use_minicf_pattern == 1)
    {
        // this->pattern_matrix = Eigen::MatrixXf::Zero(height,width);
        // this->pattern_matrix.resize(width,height);
        // this->pattern_matrix.setZero();
        this->pattern_matrix = Eigen::MatrixXf::Zero(width,height);
    }else{
        // this->pattern_matrix = Eigen::MatrixXf::One(height,width);
        // this->pattern_matrix.resize(width,height);
        // this->pattern_matrix.setConstant(1);
        this->pattern_matrix = Eigen::MatrixXf::Zero(width,height);
        this->pattern_matrix.setConstant(1);
    }
    
}

void opengl_pointcloud_render::input_dyn_clouds(pcl::PointCloud<pcl::PointXYZI> input_cloud)
{
    this->dyn_clouds = input_cloud;
    
    // try to use shader to render dynamic clouds but fails with the running time increase
    // // push back the dyn_clouds to dyn_clouds_vec
    // Eigen::Vector3f rgb_pt;
    // rgb_pt << 0,0,1;
    // dyn_clouds_vec.clear();
    // dyn_clouds_index.clear();
    // for(int i=0;i< dyn_clouds.size();i++)
    // {
    //     // trans pcl points to eigen points
    //     Eigen::Vector3f pt;
    //     pt << dyn_clouds.points[i].x, dyn_clouds.points[i].y, dyn_clouds.points[i].z;
    //     dyn_clouds_vec.push_back(pt);
    //     rgb_pt << dyn_clouds.points[i].intensity,0,0;
    //     dyn_clouds_vec.push_back(rgb_pt);
    //     dyn_clouds_index.push_back(i + cloud_color_mesh.size());
    // }
    // // cout << "dyn_clouds_vec size: " << dyn_clouds_vec.size() << endl;
    // // cout << "g_eigen_pt_vec befroe insert size: " << g_eigen_pt_vec.size() << endl;
    // // cout << "points_index_infov befroe insert size: " << points_index_infov.size() << endl;

    // // clear vector to static map size
    // g_eigen_pt_vec.erase(g_eigen_pt_vec.begin() + cloud_color_mesh.size()*2, g_eigen_pt_vec.end());
    // points_index_infov.erase(points_index_infov.begin() + cloud_color_mesh.size(), points_index_infov.end());
    
    // // cout << "g_eigen_pt_vec after erase size: " << g_eigen_pt_vec.size() << endl;
    // // cout << "points_index_infov fter erase size: " << points_index_infov.size() << endl;
    
    // // merge two vector
    // g_eigen_pt_vec.insert(g_eigen_pt_vec.end(), dyn_clouds_vec.begin(), dyn_clouds_vec.end());
    // points_index_infov.insert(points_index_infov.end(), dyn_clouds_index.begin(), dyn_clouds_index.end());

    // // cout << "g_eigen_pt_vec after insert size: " << g_eigen_pt_vec.size() << endl;
    // // cout << "points_index_infov after insert size: " << points_index_infov.size() << endl;
}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
// void opengl_pointcloud_render::framebuffer_size_callback(GLFWwindow* window, int screen_width, int screen_height)
// {
//     // make sure the viewport matches the new window dimensions; note that width and 
//     // height will be significantly larger than specified on retina displays.
//     glViewport(0, 0, screen_width, screen_height);
// }

void opengl_pointcloud_render::read_pointcloud_fromfile(std::string map_filename){

    // glfw: initialize and configure
    // ------------------------------
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    glfwWindowHint(GLFW_VISIBLE, GL_FALSE); // invisible window

#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    // glfw window creation
    // --------------------
    window = glfwCreateWindow(width, height, "Opengl_sim", NULL, NULL);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return ;
    }
    glfwMakeContextCurrent(window);
    // glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    // glfwSetCursorPosCallback(window, mouse_callback);
    // glfwSetScrollCallback(window, scroll_callback);

    // tell GLFW to capture our mouse
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // glad: load all OpenGL function pointers
    // ---------------------------------------
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return ;
    }

    // configure global opengl state
    // -----------------------------
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glEnable(GL_POINT_SMOOTH);

    // build and compile our shader zprogram
    // ------------------------------------
    //find current program path
    char* cwd  = NULL;
    cwd = get_current_dir_name();
    std::string current_path = cwd;
    //delete one folder from current path
    // current_path = current_path.substr(0, current_path.find_last_of("/"));
    // current_path += "/kong_ws/src/Exploration_sim/uav_simulator/local_sensing/include/";
    std::string root_dir = ROOT_DIR;
    current_path = root_dir + "include/";
    printf("Current Path: %s\n",current_path.c_str());
    std::string vertex_path = current_path + "360camera.vs";
    std::string fragment_path = current_path + "camera.fs";
    Shader inputshader(vertex_path.c_str(), fragment_path.c_str());
    ourShader = inputshader;

    std::cout << "shader path = " << vertex_path << std::endl;;

    // std::cout << "You Pushed a button, now open file: " << a_string.Get() << endl;
    load_pcd_file( map_filename ,cloud_color_mesh);

    //downsample pointcloud
    // pcl::VoxelGrid<PointType> voxel_grid;
    // voxel_grid.setInputCloud(cloud_color_mesh.makeShared());
    // voxel_grid.setLeafSize(downsample_res, downsample_res, downsample_res);
    // voxel_grid.filter(cloud_color_mesh);

    //Map process

    // Mainbuilding process
    string Mainbuilding_highres, Mainbuilding_lowres;
    Mainbuilding_highres = "Mainbuilding_opti_002cutoff_sor.pcd";
    Mainbuilding_lowres = "Mainbuilding_opti_01cutoff_sor.pcd";
    if((map_filename.substr(map_filename.length()-Mainbuilding_highres.length(),map_filename.length())).compare(Mainbuilding_highres) == 0 ||
     (map_filename.substr(map_filename.length()-Mainbuilding_lowres.length(),map_filename.length())).compare(Mainbuilding_lowres) == 0)
    {
    for (int i = 0; i < cloud_color_mesh.points.size(); ++i)
    {
        auto pt = cloud_color_mesh.points[i];
        PointType pr;
        pr.x = pt.x ;
        pr.y = pt.y ;
        pr.z = pt.z + 3;
        cloud_color_mesh.points[i] = pr;
    }        
    }

    //rsc process
    string rsc_highres, rsc_lowres;
    rsc_highres = "rsc_merge_002cutoff.pcd";
    rsc_lowres = "rsc_merge_01cutoff_sor.pcd";
    if((map_filename.substr(map_filename.length()-rsc_highres.length(),map_filename.length())).compare(rsc_highres) == 0 ||
     (map_filename.substr(map_filename.length()-rsc_lowres.length(),map_filename.length())).compare(rsc_lowres) == 0)
    {
    for (int i = 0; i < cloud_color_mesh.points.size(); ++i)
    {
      auto pt = cloud_color_mesh.points[i];
      PointType pr;
      pr.x = pt.x + 20;
      pr.y = pt.y - 7;
      pr.z = pt.z + 5;
      cloud_color_mesh.points[i] = pr;
    }    
    }

    //Yuen Lang 02 process
    string yuanlang02_highres, yuanlang02_lowres;
    yuanlang02_highres = "yuanlang2_optimize_005cutoff.pcd";
    yuanlang02_lowres = "yuanlang2_optimize_01cutoff.pcd";
    if((map_filename.substr(map_filename.length()-yuanlang02_highres.length(),map_filename.length())).compare(yuanlang02_highres) == 0 || 
    (map_filename.substr(map_filename.length()-yuanlang02_lowres.length(),map_filename.length())).compare(yuanlang02_lowres) == 0)
    {
    for (int i = 0; i < cloud_color_mesh.points.size(); ++i)
    {
      auto pt = cloud_color_mesh.points[i];
      PointType pr;
      pr.x = pt.x + 25;
      pr.y = pt.y;
      pr.z = pt.z - 3;
      cloud_color_mesh.points[i] = pr;
    }    
    }

    //Library G process
    string library_highres, library_lowres;
    library_highres = "LibraryLG_002cutoff_sor.pcd";
    library_lowres = "LibraryLG_01cutoff_sor.pcd";
    if((map_filename.substr(map_filename.length()-library_highres.length(),map_filename.length())).compare(library_highres) == 0 ||
     (map_filename.substr(map_filename.length()-library_lowres.length(),map_filename.length())).compare(library_lowres) == 0)
    {
        for (int i = 0; i < cloud_color_mesh.points.size(); ++i)
        {
            auto pt = cloud_color_mesh.points[i];
            PointType pr;
            pr.x = pt.x;
            pr.y = pt.y;
            pr.z = pt.z + 3;
            cloud_color_mesh.points[i] = pr;
        }    
    }

    init_pointcloud_data();
    // preprocess(cloud_color_mesh,all_normals);
    cout << "Load data finish " << endl;

    // unsigned int VBO, VAO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) *3* (g_eigen_pt_vec.size()+0), g_eigen_pt_vec.data(), GL_DYNAMIC_DRAW);


    // position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    // texture coord attribute
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

     // glGenBuffers(1, &EBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int)* (points_index_infov.size()+0), points_index_infov.data(),GL_DYNAMIC_DRAW);
   
    
    ourShader.use();
}

opengl_pointcloud_render::~opengl_pointcloud_render(){

    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    
    glfwTerminate();
}

void opengl_pointcloud_render::render_pointcloud(pcl::PointCloud<PointType>::Ptr output_pointcloud, Eigen::Vector3f camera_pos, \
                    Eigen::Quaternionf camera_q, double t_pattern_start)
{        

    // glBindVertexArray(VAO);
    // glBindBuffer(GL_ARRAY_BUFFER, VBO);
    // glBufferData(GL_ARRAY_BUFFER, sizeof(float) *6* g_eigen_pt_vec.size(), g_eigen_pt_vec.data(), GL_DYNAMIC_DRAW);

    // // position attribute
    // glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    // glEnableVertexAttribArray(0);
    // // texture coord attribute
    // glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    // glEnableVertexAttribArray(1);

    //  // glGenBuffers(1, &EBO);
    // glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    // glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int)* points_index_infov.size(), points_index_infov.data(),GL_DYNAMIC_DRAW);
   

        //avia pattern
        if(use_avia_pattern == 1)
        {
            pattern_matrix.setConstant(0);
            float w1 = 763.82589;//7294.0*2.0*3.1415926/60.0
            float w2 = -488.41293788;// -4664.0*2.0*3.1415926/60.0 
            // int linestep = 2;
            // double point_duration = 0.000025;
            double point_duration = 0.000004167 * 6;

            float t_duration = 1.0/sensing_rate;
            double t_start = t_pattern_start;

            double scale_x = 0.48*width/2.0;
            double scale_y = 0.43*height/2.0;

            int linestep = ceil(1.4/0.2);
            // std::cout << "linestep = " << linestep << std::endl;

            for (double t_i = t_start;t_i < t_start+t_duration;t_i = t_i+point_duration)
            {
            int x = round(scale_x*(cos(w1*t_i) + cos(w2*t_i))) + round(0.5 * width);
            int y = round(scale_y*(sin(w1*t_i) + sin(w2*t_i))) + round(0.5 * height);

                if(x > (width-1))
                {
                    x = (width-1);
                }else if(x < 0)
                {
                    x = 0;
                }

                if(y > (height-1))
                {
                    y = (height-1);
                }else if(y < 0)
                {
                    y = 0;
                }

            pattern_matrix(x,y) = 2;//real pattern
            pattern_matrix(x,y+linestep) = 2;
            pattern_matrix(x,y+2*linestep) = 2;
            pattern_matrix(x,y+3*linestep) = 2;
            pattern_matrix(x,y-linestep) = 2;
            pattern_matrix(x,y-2*linestep) = 2;
            }
        }else if(use_os128_pattern == 1)
        {
            pattern_matrix.setConstant(0);
            int step = round((45.0) / 128 / polar_res);
            int bottom_pattern = -ceil(22.5 / polar_res);
            for (int i = 0; i < 128; i++)
            {
            int y = bottom_pattern + i * step + round(0.5 * height);

            for (int j = 0; j < width; j++)
            {
                if (y > (height - 1))
                {
                y = (height - 1);
                }
                else if (y < 0)
                {
                y = 0;
                }
                pattern_matrix(j, y) = 2;
            }
            }
        }else if(use_minicf_pattern == 1)
        {
            pattern_matrix.setConstant(0);
            double point_duration = 1.0 / 200000.0;

            float t_duration = 1.0 / sensing_rate;
            double t_start = t_pattern_start;

            double scale_x = 0.48 * width / 2.0;
            double scale_y = 0.43 * height / 2.0;
            double PI = 3.141519265357;

            for (double t_i = t_start; t_i < t_start + t_duration; t_i = t_i + point_duration)
            {
            int x = (int(-round(-62050.63 * t_i + 3.11 * cos(314159.2 * t_i) * sin(628.318 * 2 * t_i))) % 360) / polar_res;
            int y = round(25.5 * cos(20 * PI * t_i) + 4 * cos(2 * PI / 0.006 * t_i) * cos(10000 * PI * t_i) + 22.5) / polar_res + round(0.5 * height);

            // ROS_INFO("X = %d, Y = %d",x,y);
            if (x > (width - 1))
            {
                x = (width - 1);
            }
            else if (x < 0)
            {
                x = 0;
            }

            if (y > (height - 1))
            {
                y = (height - 1);
            }
            else if (y < 0)
            {
                y = 0;
            }
            
            pattern_matrix(x, y) = 2; // real pattern
            }
        }

        //trans odom to matrix
        Eigen::Matrix3f body2world_matrix = camera_q.toRotationMatrix();
        body2world = body2world_matrix;
        glm::vec3 cameraPos = glm::vec3(camera_pos(0), camera_pos(1), camera_pos(2));
        camera = camera_pos;

        // std::cout << body2world_matrix << std::endl;

        Eigen::Vector3f body_x, body_z;
        body_x << 1, 0, 0;
        body_z << 0, 0, 1;
        body_x = body2world_matrix * body_x;
        body_z = body2world_matrix * body_z;
        glm::vec3 cameraFront    = glm::vec3(body_x(0), body_x(1), body_x(2));
        glm::vec3 cameraUp    = glm::vec3(body_z(0), body_z(1), body_z(2));

        // std::cout << cameraUp.x << ", " << cameraUp.y << ", " << cameraUp.z << std::endl;
        // std::cout << cameraFront.x << ", " << cameraFront.y << ", " << cameraFront.z << std::endl;

        // render
        // ------
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

        // activate shader
        ourShader.use();

        // pass projection matrix to shader (note that in this case it could change every frame)
        projection = glm::perspective((float)atan(((float)width)/(2*fv))*2, (float)width / (float)height, near, far);
        ourShader.setMat4("projection", projection);

        // camera/view transformation
        // view = glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp);//original linear project
        view = glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp);
        ourShader.setMat4("view", view);

        glm::vec2 sensing_range = glm::vec2(near,far);
        ourShader.setVec2("range", sensing_range);

        glm::vec2 fov = glm::vec2(yaw_fov,vertical_fov);
        ourShader.setVec2("fov",fov);

        glm::vec2 res = glm::vec2(downsample_res,polar_res);
        ourShader.setVec2("res",res);

        glm::mat3 rotation_mat;
        Eigen::Matrix3f world2body = body2world_matrix.transpose();//

        for(int i = 0; i < 3;i++)
        {
            for(int j = 0;j < 3;j++)
            {
                rotation_mat[i][j] = world2body(j,i);
            }
        }
        
        // glm::mat3 rotation_mat = glm::make_mat3x3((body2world_matrix.transpose()));
        ourShader.setMat3("rot",rotation_mat);

        // glm::vec3 curpos = glm::vec3()
        ourShader.setVec3("pos",cameraPos);

        //translate matrix to eigen
        Eigen::Matrix4f eigen_view = Eigen::Matrix4f::Identity();
        for(int i = 0; i < 4; i++)
        {
            for(int j = 0; j < 4; j++)
            {
                eigen_view(i,j) = view[j][i];
            }
        }
        Eigen::Matrix4f eigen_proj = Eigen::Matrix4f::Identity();
        for(int i = 0; i < 4; i++)
        {
            for(int j = 0; j < 4; j++)
            {
                eigen_proj(i,j) = projection[i][j];
            }
        }
        camera2world = eigen_view.block<3,3>(0,0).transpose();
        // std::cout << eigen_view << std::endl;
        // std::cout << eigen_proj << std::endl;

        system_clock::time_point t0 = system_clock::now();

        //FoV checker
        // points_index_infov.clear();
        // vector<BoxPointType> fov_boxes;
        // Eigen::Vector3d camera_pos_d, body_x_d;
        // camera_pos_d(0) = camera_pos(0);
        // camera_pos_d(1) = camera_pos(1);
        // camera_pos_d(2) = camera_pos(2);
        // body_x_d(0) = body_x(0);
        // body_x_d(1) = body_x(1);
        // body_x_d(2) = body_x(2);
        // double far_d = far;
        // fov_checker.check_fov(camera_pos_d, body_x_d, M_PI * (77.0 / 2.0 + 10) / 180.0, far_d, fov_boxes);
        // Eigen::Vector3d box_center_temp;
        // long int ind_x,ind_y,ind_z;
        // for(int i = 0;i<fov_boxes.size();i++)
        // { 
        //   box_center_temp(0) = (fov_boxes[i].vertex_max[0] - fov_boxes[i].vertex_min[0])*0.5 + fov_boxes[i].vertex_min[0];
        //   box_center_temp(1) = (fov_boxes[i].vertex_max[1] - fov_boxes[i].vertex_min[1])*0.5 + fov_boxes[i].vertex_min[1];
        //   box_center_temp(2) = (fov_boxes[i].vertex_max[2] - fov_boxes[i].vertex_min[2])*0.5 + fov_boxes[i].vertex_min[2];

        //   ind_x = (round((box_center_temp(0) - env_box.vertex_min[0] + EPSS)/hash_cubesize));
        //   ind_y = (round((box_center_temp(1) - env_box.vertex_min[1] + EPSS)/hash_cubesize));
        //   ind_z = (round((box_center_temp(2) - env_box.vertex_min[2] + EPSS)/hash_cubesize));

        //   pcl::PointXYZI box_pt;
        //   box_pt.x = box_center_temp(0);
        //   box_pt.y = box_center_temp(1);
        //   box_pt.z = box_center_temp(2);
        //   // local_map.push_back(box_pt);

        //   long int box_index = ind_x + ind_y*cube_numx + ind_z*cube_numx*cube_numy;
          
        //   points_index_infov.insert(points_index_infov.end(),pointindex_hashmap[box_index].begin(),pointindex_hashmap[box_index].end());
        // }

        // glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        // void *ebo_cp = glMapBuffer(GL_ELEMENT_ARRAY_BUFFER, GL_WRITE_ONLY);
        // memcpy(ebo_cp, points_index_infov.data(), sizeof(unsigned int)* points_index_infov.size());
        // glUnmapBuffer(GL_ELEMENT_ARRAY_BUFFER);

        // std::cout << "points index infov size = " << points_index_infov.size() <<std::endl;

        // render boxes
        glBindVertexArray(VAO);  

        system_clock::time_point t1 = system_clock::now();
        auto fov_check_dur = t1 - t0;
        duration<double> fovcheck_second(fov_check_dur);
        // std::cout << "FoV checker cost " << fovcheck_second.count() << " seconds" << std::endl;

        glPointSize(1);
        // glColor3f(1.0f,0.0f,0.0f);
        // glDrawArrays(GL_POINTS, 0, g_eigen_pt_vec.size()/2);
        // glDrawElements(GL_POINTS, g_eigen_pt_vec.size()/2, GL_UNSIGNED_INT, 0);
        glDrawElements(GL_POINTS, points_index_infov.size(), GL_UNSIGNED_INT, 0);

        glfwSwapBuffers(window);
        glfwPollEvents();

        // system_clock::time_point t3 = system_clock::now();
        // auto dur2 = t3 - t1;
        // duration<double> second2(dur2);
        // std::cout << "One draw frame cost " << second2.count() << " seconds" << std::endl;

        depth_ptcloud_vec.clear();
        read_depth(16,depth_ptcloud_vec);

        pcl::copyPointCloud(*render_cloud,*output_pointcloud);
        system_clock::time_point t2 = system_clock::now();
        auto dur = t2 - t1;
        duration<double> second(dur);
        // std::cout << "One frame show cost " << second.count() << " seconds" << std::endl;
}

//preprocess pointcloud data
void opengl_pointcloud_render::preprocess(pcl::PointCloud< PointType > cloud_all_map, pcl::PointCloud<pcl::Normal>::Ptr all_normals)
{
                  pcl::NormalEstimation<PointType,pcl::Normal> normalEstimation;
                  //preprocess pointcloud data, normal estimation
                  normalEstimation.setInputCloud(cloud_all_map.makeShared());
                  //对于每一个点都用半径为3cm的近邻搜索方式
                  normalEstimation.setRadiusSearch(3.0*downsample_res);
                  //Kd_tree是一种数据结构便于管理点云以及搜索点云，法线估计对象会使用这种结构来找到哦啊最近邻点
                  pcl::search::KdTree<PointType>::Ptr kdtree(new pcl::search::KdTree<PointType>);
                  normalEstimation.setSearchMethod(kdtree);
                  //计算法线
                  normalEstimation.compute(*all_normals);

                  int origin_mapptcount = cloud_all_map.points.size();
                  cout << "Normal compute finished.., mapsize = " << origin_mapptcount << endl;

                  string normal_filename = "/home/dji/meshmap/normal_files/Knowles_01_normals.pcd";
                  pcl::io::savePCDFileASCII(normal_filename, *all_normals);
}

void opengl_pointcloud_render::render_dynclouds_on_depthimage(cv::Mat& depth_image)
{

    //count running time
    system_clock::time_point t1 = system_clock::now();

    int width = depth_image.cols;
    int height = depth_image.rows;

    // cout << "depth image size = " << width << " x " << height << endl;
    // cout << "dyn_clouds size = " << dyn_clouds.points.size() << endl;

// #pragma omp parallel default (none) \
//                     shared (dyn_clouds, effect_range,\
//                     cover_dis, width, height,depth_image,\
//                     camera2world, camera, yaw_fov, vertical_fov,\
//                     downsample_res, near, far, polar_res)
// {
// #pragma omp for
    for(int i = 0; i< dyn_clouds.points.size();i++)
    {
        // cout << "in dyn cloud loop: " << i << endl;

        Eigen::Vector3f temp_point;
        temp_point(0) = dyn_clouds.points[i].x;
        temp_point(1) = dyn_clouds.points[i].y;
        temp_point(2) = dyn_clouds.points[i].z;

        // fov check
        Eigen::Vector3f temp_point_cam = body2world.transpose() * (temp_point - camera);

        // project to depth image and interline
        float depth = temp_point_cam.norm();
        if(depth > far || depth < near)
            continue;

        Eigen::Vector3f temp_point_cam_norm = temp_point_cam.normalized();
        int cen_theta_index = atan2(temp_point_cam_norm(1),temp_point_cam_norm(0)) / M_PI * 180.0 /polar_res + round(0.5*width);
        int cen_fi_index = atan2(temp_point_cam_norm(2),sqrt(temp_point_cam_norm(0)*temp_point_cam_norm(0)+temp_point_cam_norm(1)*temp_point_cam_norm(1))) / M_PI * 180.0 /polar_res + round(0.5*height);
        if(cen_theta_index < 0 || cen_theta_index >= width || cen_fi_index < 0 || cen_fi_index >= height)
            continue;

        // cout << "temp_point = " << temp_point.transpose() << endl;
        // cout << "temp_point_cam = " << temp_point_cam.transpose() << endl;

        if(depth > effect_range)
        {
            if(depth_image.at<float>(cen_fi_index,cen_theta_index) > depth)
                depth_image.at<float>(cen_fi_index,cen_theta_index) = depth;
        }else{
            int half_cover_angle = ceil(
                (asin(cover_dis / depth) / (M_PI * polar_res / 180.0)));

            // cout << "half_cover_angle = " << half_cover_angle << endl;
            // cout << "cen_theta_index = " << cen_theta_index << endl;
            // cout << "cen_fi_index = " << cen_fi_index << endl;

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
                    int fi_index_inver = height -1 - fi_index_o;
                    int theta_index_inver = width -1 - theta_index_o;
                    if ((theta_index_inver > (width - 1)) || (theta_index_inver < 0) || (fi_index_inver > (height - 1)) || (fi_index_inver < 0))
                    {
                        continue;
                    }

                    if(depth_image.at<float>(fi_index_inver,theta_index_inver) > depth)
                    {
                        depth_image.at<float>(fi_index_inver,theta_index_inver) = depth;
                        density_matrix(fi_index_inver,theta_index_inver) = dyn_clouds.points[i].intensity;
                    }
                }
            }
        }
    }
// }

    system_clock::time_point t2 = system_clock::now();
    auto dur = t2 - t1;
    duration<double> second(dur);
    // cout << "dynamic interline costs " << second.count() << " seconds" << endl;
        // std::cout << "One frame interline costs " << second.count() << " seconds, " << "first for loop costs "<< second2.count() << endl;
}

void opengl_pointcloud_render::depth_interline_on_depthimage(cv::Mat& depth_image)
{

    //count running time
    system_clock::time_point t1 = system_clock::now();

    int width = depth_image.cols;
    int height = depth_image.rows;

    vector<Eigen::Vector3f> shouldinterline_ptvec;
    vector<Eigen::Vector2i> shouldinterline_ptvec_index;

    for (int v = 0; v < height; v++)
    {
        for (int u = 0; u < width; u++)
        {
            float depth = depth_image.at<float>(v, u);
            if (depth > near && depth<far)
            {
                Eigen::Vector3f temp_point, temp_point_world;
                temp_point(0) = (u - u0) * depth / fu;
                temp_point(1) = -(v - v0) * depth / fv;
                temp_point(2) = -depth;

                if(temp_point.norm() < effect_range)
                {
                    shouldinterline_ptvec.push_back(temp_point);
                    shouldinterline_ptvec_index.push_back(Eigen::Vector2i(u,v));
                }

                // temp_point_world = camera2world.transpose() * (temp_point - camera);//.transpose()

                // origin_cloud->points[v * width + u].x = temp_point_world(0);
                // origin_cloud->points[v * width + u].y = temp_point_world(1);
                // origin_cloud->points[v * width + u].z = temp_point_world(2);
                // depth_ptcloud_vec[v * width + u] = temp_point_world;

                //depth interline points vector construct
                
            }else{
                // origin_cloud->points[v * width + u].x = 0;
                // origin_cloud->points[v * width + u].y = 0;
                // origin_cloud->points[v * width + u].z = 0;
                // depth_ptcloud_vec[v * width + u] = Eigen::Vector3f(0,0,0);

            }
        }
    }
    system_clock::time_point t_middle = system_clock::now();
    // cout << "shouldinterline_ptvec size = " << shouldinterline_ptvec.size() << endl;

// #ifdef USE_OPENMP
// #pragma 
    //depth interline points on depth image

#pragma omp parallel default (none) \
                     shared (shouldinterline_ptvec, shouldinterline_ptvec_index, fu, fv, effect_range,\
                     cover_dis, width, height,depth_image)
{
#pragma omp for
    for(int i = 0; i< shouldinterline_ptvec.size();i++)
    {
        Eigen::Vector3f temp_point;
        temp_point = shouldinterline_ptvec[i];

        int u = shouldinterline_ptvec_index[i](0);
        int v = shouldinterline_ptvec_index[i](1);
        float depth = -temp_point(2);

            float theta = atan2(temp_point(0),depth);
            float fi = atan2(temp_point(1),depth);
            // float cover_dis = 0.55 * 1.7321 * downsample_res;// 0.707
            float polar_resolution_x = 1.0/fu*(cos(theta)*cos(theta))*180/M_PI;
            float polar_resolution_y = 1.0/fv*(cos(fi)*cos(fi))*180/M_PI;
            // //find max polar resolution
            // float polar_resolution_max = polar_resolution_x > polar_resolution_y ? polar_resolution_x : polar_resolution_y;
            // //compute effective range
            // float effect_range = cover_dis / sin(0.5 * polar_resolution_max * M_PI / 180.0);

            // std::cout << "theta: " << theta << " fi: " << fi << " polar_resolution_x: " << polar_resolution_x << " polar_resolution_y: " \
            // << polar_resolution_y << " polar_resolution_max: " << polar_resolution_max << " effect_range: " << effect_range << std::endl;

            if (temp_point.norm() > effect_range)
            {
                continue;
            }else{
                int half_cover_angle_x = ceil(
                        (asin(cover_dis / temp_point.norm()) / (M_PI * polar_resolution_x / 180.0))); 
                int half_cover_angle_y = ceil(
                        (asin(cover_dis / temp_point.norm()) / (M_PI * polar_resolution_y / 180.0)));

                // int half_cover_angle_x = 1;
                // int half_cover_angle_y = 1;

                // // cout << "half_cover_angle_x: " << half_cover_angle_x << " half_cover_angle_y: " << half_cover_angle_y << endl;
                // // cout << "depth: " << depth << " temp_point.norm(): " << temp_point.norm() << endl;

                int theta_start = u - half_cover_angle_x;
                int theta_end = u + half_cover_angle_x;
                int fi_start = v - half_cover_angle_y;
                int fi_end = v + half_cover_angle_y;
              
                for (int theta_index_o = theta_start;
                    theta_index_o <= theta_end; theta_index_o++) {
                for (int fi_index_o = fi_start;
                    fi_index_o <= fi_end; fi_index_o = fi_index_o+1) {
                        if(theta_index_o < 0 || theta_index_o >= width || fi_index_o < 0 || fi_index_o >= height)
                            {continue;}
                        if(depth_image.at<float>(fi_index_o, theta_index_o) > depth)
                        {
                            // cout << "depth_image.at<float>(fi_index_o, theta_index_o) = " << depth_image.at<float>(fi_index_o, theta_index_o)
                            // << "depth = " <<depth << endl;
                            depth_image.at<float>(fi_index_o, theta_index_o) = depth;
                        }
                    }
                }
            }
    }
}

    system_clock::time_point t2 = system_clock::now();
        auto dur = t2 - t1;
        duration<double> second(dur);
        auto dur2 = t_middle - t1;
        duration<double> second2(dur2);
        // std::cout << "One frame interline costs " << second.count() << " seconds, " << "first for loop costs "<< second2.count() << endl;
}

void opengl_pointcloud_render::faces_need_draw_from_depthimage(cv::Mat& depth_image, cv::Mat color_image)
{
    //count running time
    system_clock::time_point t1 = system_clock::now();

    int width = depth_image.cols;
    int height = depth_image.rows;

    vector<Eigen::Vector3f> shouldinterline_ptvec;
    vector<Eigen::Vector2i> shouldinterline_ptvec_index;

    for (int v = 0; v < height; v++)
    {
        for (int u = 0; u < width; u++)
        {
            float depth = depth_image.at<float>(v, u);
            if (depth > near && depth<far)
            {
                Eigen::Vector3f temp_point;
                temp_point(0) = (u - u0) * depth / fu;
                temp_point(1) = -(v - v0) * depth / fv;
                temp_point(2) = -depth;

                if(temp_point.norm() < effect_range)
                {
                    shouldinterline_ptvec.push_back(temp_point);
                    shouldinterline_ptvec_index.push_back(Eigen::Vector2i(u,v));
                }
                
            }else{

            }
        }
    }

    // cout << "shouldinterline_ptvec size = " << shouldinterline_ptvec.size() << endl;

    g_eigen_tri_pt_vec.clear();
    g_eigen_tri_rgb_vec.clear();
    g_eigen_tri_pt_vec.resize(shouldinterline_ptvec.size()*6);
    g_eigen_tri_rgb_vec.resize(shouldinterline_ptvec.size()*6);
    //faces extracted from points on depth image
    for(int i = 0; i< shouldinterline_ptvec.size();i++)
    {
        Eigen::Vector3f temp_point, temp_point_world, temp_point_normal, dir_vec;
        temp_point_normal << (color_image.at<Vec3b>(shouldinterline_ptvec_index[i](1), shouldinterline_ptvec_index[i](0))[2]-128) / 255.0 \
                            , (color_image.at<Vec3b>(shouldinterline_ptvec_index[i](1), shouldinterline_ptvec_index[i](0))[1]-128) / 255.0 \
                            , (color_image.at<Vec3b>(shouldinterline_ptvec_index[i](1), shouldinterline_ptvec_index[i](0))[0]-128) / 255.0;
        temp_point_normal.normalize();
        temp_point = shouldinterline_ptvec[i];

        temp_point_world = camera2world * (temp_point) + camera;
        dir_vec = temp_point_world - camera;
        dir_vec.normalize();

        // std::cout << "temp_point_normal = " << temp_point_normal << endl;
        // std::cout << "temp_point_world = " << temp_point_world << std::endl;
        
        Eigen::Vector3f xdir_vec, ydir_vec, xydir_vec;

        //check angle of two vector is less than 30 degree
        if(acos(temp_point_normal.dot(dir_vec)) < M_PI * 0.0 / 180.0)
        {
            xdir_vec << 1,0,0;
            ydir_vec = xdir_vec.cross(temp_point_normal);
            ydir_vec.normalize();
            xydir_vec = xdir_vec+ydir_vec;
            xydir_vec.normalize();
        }else{
            //cross product to generate interline square
            xdir_vec = dir_vec.cross(temp_point_normal);
            xdir_vec.normalize();
            ydir_vec = xdir_vec.cross(temp_point_normal);
            ydir_vec.normalize();
            xydir_vec = xdir_vec+ydir_vec;
            xydir_vec.normalize();
        }
            // std::cout << "coverage_dis = " << cover_dis << std::endl;
            // cout << "xdir_vec = " << xdir_vec << endl;
            // cout << "ydir_vec = " << ydir_vec << endl;

            g_eigen_tri_pt_vec[ 6 * i ] = temp_point_world + xdir_vec * cover_dis* 0.5 + ydir_vec * cover_dis* 0.5; // may be a little big square
            g_eigen_tri_pt_vec[ 6 * i + 1 ] = temp_point_world + xdir_vec * cover_dis* 0.5 - ydir_vec * cover_dis* 0.5;
            g_eigen_tri_pt_vec[ 6 * i + 2 ] = temp_point_world - xdir_vec * cover_dis* 0.5 + ydir_vec * cover_dis* 0.5;
            g_eigen_tri_pt_vec[ 6 * i + 3 ] = temp_point_world - xdir_vec * cover_dis* 0.5 - ydir_vec * cover_dis* 0.5;
            g_eigen_tri_pt_vec[ 6 * i + 4 ] = temp_point_world - xdir_vec * cover_dis* 0.5 + ydir_vec * cover_dis* 0.5;
            g_eigen_tri_pt_vec[ 6 * i + 5 ] = temp_point_world + xdir_vec * cover_dis* 0.5 - ydir_vec * cover_dis* 0.5;

            // std::cout << "g_eigen_tri_pt_vec[ 3 * i ] = " << g_eigen_tri_pt_vec[ 6 * i ] << std::endl;

            //color vector
            Eigen::Matrix<unsigned char, 3, 1> color_vec;
            color_vec << 255, 0 , 0;
            g_eigen_tri_rgb_vec[ 6 * i ] = color_vec;
            g_eigen_tri_rgb_vec[ 6 * i + 1 ] = color_vec;
            g_eigen_tri_rgb_vec[ 6 * i + 2 ] = color_vec;
            g_eigen_tri_rgb_vec[ 6 * i + 3 ] = color_vec;
            g_eigen_tri_rgb_vec[ 6 * i + 4 ] = color_vec;
            g_eigen_tri_rgb_vec[ 6 * i + 5 ] = color_vec;


    }

    system_clock::time_point t2 = system_clock::now();
        auto dur = t2 - t1;
        duration<double> second(dur);
        // std::cout << "One frame interline costs " << second.count() << " seconds\n";
}

//from depth image to point cloud
void opengl_pointcloud_render::depth_to_pointcloud(cv::Mat& depth_image, pcl::PointCloud<PointType>::Ptr origin_cloud, \
                        std::vector<Eigen::Matrix<float, 3, 1> >& depth_ptcloud_vec,\
                        vector<vector<Eigen::Matrix<float, 3, 1> > >& interline_ptcloud_vec)
{
    int width = depth_image.cols;
    int height = depth_image.rows;
    origin_cloud->points.clear();
    depth_ptcloud_vec.clear();
    // origin_cloud->width = width;
    // origin_cloud->height = height;
    // origin_cloud->is_dense = false;
    // origin_cloud->points.resize(origin_cloud->width * origin_cloud->height);
    // depth_ptcloud_vec.resize(origin_cloud->width * origin_cloud->height);
    u0 = width*0.5;
    v0 = height*0.5;

    // depth_interline_on_depthimage(depth_image);

    cv::Mat pattern_image = cv::Mat::zeros(height, width, CV_32F);

    //trans linear project depth image to point cloud 
    // for (int v = 0; v < height; v++)
    // {
    //     for (int u = 0; u < width; u++)
    //     {
    //         pattern_image.at<float>(v,u) = 30;
    //         if(pattern_matrix(u,v)>0)
    //         {
    //         float depth = depth_image.at<float>(v, u);
    //         if (depth > near && depth<(far-0.1))
    //         {
    //             Eigen::Vector3f temp_point, temp_point_world;
    //             temp_point(0) = (u - u0) * depth / fu;
    //             temp_point(1) = -(v - v0) * depth / fv;
    //             temp_point(2) = -depth;

    //             //using R and T from opengl
    //             // temp_point_world = camera2world.transpose() * (temp_point - camera);//.transpose()

    //             //interface with ros
    //             temp_point_world = camera2world * (temp_point) + camera;

    //             origin_cloud->points[v * width + u].x = temp_point_world(0);
    //             origin_cloud->points[v * width + u].y = temp_point_world(1);
    //             origin_cloud->points[v * width + u].z = temp_point_world(2);
    //             depth_ptcloud_vec[v * width + u] = temp_point_world;

    //             //depth interline points vector construct

    //             pattern_image.at<float>(v,u) = depth;
                
    //         }else if(depth >= (far-0.1)){
    //             Eigen::Vector3f temp_point, temp_point_world;
    //             temp_point(0) = (u - u0) * depth / fu;
    //             temp_point(1) = -(v - v0) * depth / fv;
    //             temp_point(2) = -depth;
    //             //using R and T from opengl
    //             // temp_point_world = camera2world.transpose() * (temp_point - camera);//.transpose()

    //             //get normal vector
    //             temp_point.normalize();
    //             temp_point = 2*far*temp_point;

    //             //interface with ros
    //             temp_point_world = camera2world * (temp_point) + camera;

    //             origin_cloud->points[v * width + u].x = temp_point_world(0);
    //             origin_cloud->points[v * width + u].y = temp_point_world(1);
    //             origin_cloud->points[v * width + u].z = temp_point_world(2);
    //             depth_ptcloud_vec[v * width + u] = temp_point_world;

    //             // origin_cloud->points[v * width + u].x = 0;
    //             // origin_cloud->points[v * width + u].y = 0;
    //             // origin_cloud->points[v * width + u].z = 0;
    //             // depth_ptcloud_vec[v * width + u] = Eigen::Vector3f(0,0,0);

    //         }
    //         }
    //     }
    // }

    //trans polar depth image to point clouds 
    // float polar_resolution = 0.2/180.0*3.14159265358;
    float polar_res_rad = polar_res/180.0*3.14159265358;
    for (int v = 0; v < height; v++)
    {
        for (int u = 0; u < width; u++)
        {
            pattern_image.at<float>(v,u) = 30;
            if(pattern_matrix(u,(height-1-v))>0)
            {
            float depth = depth_image.at<float>(v, u);
                // if(v == v0 && u == (int)(u0))
                // {
                //     std::cout << "depth = " << depth << std::endl;
                // }
            if (depth > near && depth<(far-0.1))
            {
                Eigen::Vector3f temp_point, temp_point_world;
                // temp_point(0) = (u - u0) * depth / fu;
                // temp_point(1) = -(v - v0) * depth / fv;
                // temp_point(2) = -depth;
                // temp_point(0) = depth * cos(polar_resolution*(v - v0)) * sin(polar_resolution*(u-u0));
                // temp_point(1) = -depth * sin(polar_resolution*(v - v0));
                // temp_point(2) = -depth * cos(polar_resolution*(v - v0)) * cos(polar_resolution*(u-u0));
                // temp_point(0) = depth * cos(vertical_fov/2/180.0*3.14159265358*(v - v0)/(0.5*height)) * sin(polar_res*(u-u0));
                // temp_point(1) = -depth * sin(vertical_fov/2/180.0*3.14159265358*(v - v0)/(0.5*height));
                // temp_point(2) = -depth * cos(vertical_fov/2/180.0*3.14159265358*(v - v0)/(0.5*height)) * cos(polar_res*(u-u0));

                temp_point(0) = depth * cos(polar_res_rad*(v - v0)) * sin(polar_res_rad*(u-u0));
                temp_point(1) = -depth * sin(polar_res_rad*(v - v0));
                temp_point(2) = -depth * cos(polar_res_rad*(v - v0)) * cos(polar_res_rad*(u-u0));


                // if(v == v0 && u ==u0)
                // {
                //     std::cout << "depth = " << depth << " , temp point = " << temp_point.transpose() << std::endl;
                // }

                //using R and T from opengl
                // temp_point_world = camera2world.transpose() * (temp_point - camera);//.transpose()

                //interface with ros
                temp_point_world = camera2world * (temp_point) + camera;
                // temp_point_world = temp_point;

                // origin_cloud->points[v * width + u].x = temp_point_world(0);
                // origin_cloud->points[v * width + u].y = temp_point_world(1);
                // origin_cloud->points[v * width + u].z = temp_point_world(2);
                // origin_cloud->points[v * width + u].intensity = density_matrix(v,u);

                PointType temp_pclpt;
                temp_pclpt.x = temp_point_world(0);
                temp_pclpt.y = temp_point_world(1);
                temp_pclpt.z = temp_point_world(2);
                temp_pclpt.intensity = density_matrix(v,u);
                origin_cloud->points.push_back(temp_pclpt);
                depth_ptcloud_vec.push_back(temp_point_world);
                // depth_ptcloud_vec[v * width + u] = temp_point_world;

                //depth interline points vector construct

                pattern_image.at<float>(v,u) = depth;
                
            }else if(depth >= (far-0.1)){
                Eigen::Vector3f temp_point, temp_point_world;
                // temp_point(0) = (u - u0) * depth / fu;
                // temp_point(1) = -(v - v0) * depth / fv;
                // temp_point(2) = -depth;
                temp_point(0) = depth * cos(polar_res_rad*(v - v0)) * sin(polar_res_rad*(u-u0));
                temp_point(1) = -depth * sin(polar_res_rad*(v - v0));
                temp_point(2) = -depth * cos(polar_res_rad*(v - v0)) * cos(polar_res_rad*(u-u0));

                //using R and T from opengl
                // temp_point_world = camera2world.transpose() * (temp_point - camera);//.transpose()

                //get normal vector
                temp_point.normalize();
                temp_point = 2*far*temp_point;

                //interface with ros
                temp_point_world = camera2world * (temp_point) + camera;

                if(use_inf_pt) {
                    PointType temp_pclpt;
                    temp_pclpt.x = temp_point_world(0);
                    temp_pclpt.y = temp_point_world(1);
                    temp_pclpt.z = temp_point_world(2);
                    temp_pclpt.intensity = density_matrix(v,u);
                    origin_cloud->points.push_back(temp_pclpt);
                    depth_ptcloud_vec.push_back(temp_point_world);
                }

                // if do not want to show the points over sensing range, comment the following code
                // origin_cloud->points[v * width + u].x = temp_point_world(0);
                // origin_cloud->points[v * width + u].y = temp_point_world(1);
                // origin_cloud->points[v * width + u].z = temp_point_world(2);
                // origin_cloud->points[v * width + u].intensity = density_matrix(v,u);
                // depth_ptcloud_vec[v * width + u] = temp_point_world;

                // origin_cloud->points[v * width + u].x = 0;
                // origin_cloud->points[v * width + u].y = 0;
                // origin_cloud->points[v * width + u].z = 0;
                // depth_ptcloud_vec[v * width + u] = Eigen::Vector3f(0,0,0);

            }
            }
        }
    }

        // double  min;
        // double  max;
        // minMaxLoc(pattern_image,&min,&max,0,0);
        // // cout << "final_min = " << min << ", final_max = " << max << endl ;
        // pattern_image.convertTo( pattern_image, CV_8UC1, 255.0 / ( max - min ), -min );
        // cv::Mat show_pattern;
        // cv::applyColorMap( pattern_image, show_pattern, cv::COLORMAP_HOT );
        // cv::imshow("pattern",show_pattern);
        // cv::waitKey(1);
    
    // std::cout << "origin_cloud->points.size() = " << origin_cloud->points.size() << std::endl;
}

inline cv::Mat opengl_pointcloud_render::colormap_depth_img(cv::Mat & depth_mat)
{
    cv::Mat cm_img0;
    cv::Mat adjMap;
    double  min;
    double  max;
    // expand your range to 0..255. Similar to histEq();
    // cv::minMaxIdx(depth_mat, &min, &max);
    // get_real_near_far<unsigned short>(depth_mat, min, max);
    // cout << "Min = " << min << ", max = " << max ;
    // min = 0;
    // max = 1000;
    // max = min+10000;

    // depth_mat.setTo(63000,depth_mat>65534);


    //process opencv mat 
    for (int i = 0; i < depth_mat.rows; ++i) {
        for (int j = 0; j < depth_mat.cols; ++j) {
            float depth = depth_mat.at<float>(i, j);
            // depth_mat.at<float>(i, j) = LinearizeDepth(depth,near,far);
            // depth_mat.at<float>(i, j) = regainrealdepth(depth,near,far);
            depth_mat.at<float>(i, j) = 2*(depth-0.5)*far;//
        }
    }
    minMaxLoc(depth_mat,&min,&max,0,0);
    // cout << "final_min = " << min << ", final_max = " << max << endl ;
    depth_mat.convertTo( adjMap, CV_8UC1, 255.0 / ( max - min ), -min );

        // cv::imshow("Depth1", depth_mat);//colormap_depth_img(image_aft_flip)
        // // std::cout << image_mat << std::endl;
        // cv::waitKey(1);

    cm_img0 = adjMap;//adjMap
    // cv::applyColorMap( adjMap, cm_img0, cv::COLORMAP_JET );

    // cv::normalize(depth_mat, adjMap, 0, 255, cv::NORM_MINMAX);
    // adjMap.convertTo(cm_img0, CV_8UC3);
    // cv::applyColorMap(255-adjMap, cm_img0, cv::COLORMAP_JET);
    return cm_img0;
}

void opengl_pointcloud_render::read_depth( int depth_buffer_precision , std::vector<Eigen::Matrix<float, 3, 1> >& depth_ptcloud_vec)
{
    if ( depth_buffer_precision == 16 )
    {
        //add time count
        system_clock::time_point t0 = system_clock::now();

        int image_width = width;
        int image_height = height;
        GLfloat mypixels[ image_width * image_height ];//GLushort GLuint
        // glReadPixels( (int)(5*width*(0.2+0.8*0.5)) , (int)(5*width*(1-0.8*0.5/640.0*480.0)), (int)(5*width*0.8*0.5), (int)(5*width*0.8*0.5/640.0*480.0), GL_DEPTH_COMPONENT, GL_FLOAT , mypixels );//GL_UNSIGNED_SHORT GL_UNSIGNED_INT
        // (int)(5*width*(1-0.8*0.5*640.0*480.0))
        // glReadPixels( (int)(2*width) , (int)(1*height)-30, image_width, image_height, GL_DEPTH_COMPONENT, GL_FLOAT , mypixels );
        glReadPixels( 0, 0, width, height, GL_DEPTH_COMPONENT, GL_FLOAT, mypixels );

        system_clock::time_point t_afterreadpixels = system_clock::now();

        // read color elements
        // GLubyte mypixels2[ image_width * image_height * 3 ];
        // glReadPixels(  0 , 0, image_width, image_height, GL_RGB, GL_UNSIGNED_BYTE, mypixels2 );
        // //trans into cv::Mat
        // cv::Mat color_image_mat(image_height, image_width, CV_8UC3, mypixels2);
        // cv::flip(color_image_mat, color_image_mat , 0 );
        // cv::imshow("color_image_mat", (color_image_mat));//
        // cv::waitKey(1);
        // //the image read from opengl's Red and Blue channel is reversed

        // // only read red channel image and set as density reference, unit
        // GLubyte red_pixels[ image_width * image_height ];
        // glReadPixels(  0 , 0, image_width, image_height, GL_RED, GL_UNSIGNED_BYTE, red_pixels );
        // cv::Mat color_image_mat(image_height, image_width, CV_8UC1, red_pixels);
        // cv::flip(color_image_mat, color_image_mat , 0 );
        // cv::imshow("red_image_mat", (color_image_mat));//
        // cv::waitKey(1);

        // only read red channel image and set as density reference, float
        GLfloat red_pixels[ image_width * image_height ];
        glReadPixels(  0 , 0, image_width, image_height, GL_RED, GL_FLOAT, red_pixels );
        cv::Mat red_image_mat(image_height, image_width, CV_32FC1, red_pixels);
        cv::flip(red_image_mat, red_image_mat , 0 );
        // cv::imshow("red_image_mat", (red_image_mat));//
        // cv::waitKey(1);

        // copy red channel to density matrix
        // red_image_mat.copyTo(density_mat);
        // trans mat to density eigen matrix
        density_matrix.resize(image_height, image_width);
        for (int v = 0; v < image_height; v++)
        {
            for (int u = 0; u < image_width; u++)
            {
                density_matrix(v,u) = red_image_mat.at<float>(v,u);
                // image_height-1-
            }
        }

        // // check the max and min value of the matrix
        // double  red_min;
        // double  red_max;
        // minMaxLoc(color_image_mat,&red_min,&red_max,0,0);
        // cout << "red_min = " << red_min << ", red_max = " << red_max << endl ;

        // std::ofstream pixelfile;
        // pixelfile.open("/home/dji/OpenGL/test_pangolin/pixels.csv");

        // int textline = 0;
        // for(int i = 0;i<width*height;i++)
        // {
        //     if((i%width) ==0)
        //     {
        //         pixelfile<<"\n";
        //         textline++;
        //     }
        //     pixelfile << mypixels[i] << " ";
        // }
        // pixelfile.close();

        // int height_depth = (int)(640.0f / 480.0f * UI_WIDTH);
        // std::cout << height_depth << " " <<  UI_WIDTH << std::endl;
        // GLushort mypixels[ UI_WIDTH * height_depth ];
        // glReadPixels( 0, 0, UI_WIDTH, height_depth, GL_DEPTH_COMPONENT, GL_UNSIGNED_SHORT, mypixels );
        // cout << mypixels[ (int)(width * height *0.5 + width*0.5) ] << endl;

        cv::Mat image_mat(image_height, image_width, CV_32F, mypixels);

        // std::ofstream pixelfile;
        // pixelfile.open("/home/dji/OpenGL/test_pangolin/mat.csv");
        // pixelfile << image_mat <<std::endl;
        // pixelfile.close();

        // std::cout << image_mat << std::endl;
        // std::cout << image_mat.rows << " " <<  image_mat.cols << std::endl;
        cv::Mat image_aft_flip;

        //original 
        cv::flip(image_mat, image_aft_flip , 0 );
        //polar
        // image_aft_flip = image_mat;

        // colormap_depth_img(image_mat);
        // cv::imshow("Depth", );//
        colormap_depth_img(image_aft_flip);
        // cv::waitKey(1);

        // depth_interline_on_depthimage(image_aft_flip);
        render_dynclouds_on_depthimage(image_aft_flip);

        // faces_need_draw_from_depthimage(image_aft_flip,color_image_mat);
        // new_gaussian_interline(image_aft_flip);
        // interlinebytriangles(image_aft_flip);

        // cv::Mat thresh_mat;
        // cv::threshold(image_aft_flip, thresh_mat, near, 255.0, cv::THRESH_BINARY);
        // cv::imshow("thresh_mat", thresh_mat);
        // cv::waitKey(1);

        system_clock::time_point t_after_interline = system_clock::now();

        // insert dynamic point clouds into depth image
        // for(int i = 0;i < dyn_clouds.points.size();i++)
        // {

        // }

        depth_to_pointcloud(image_aft_flip, render_cloud, depth_ptcloud_vec,interline_ptcloud_vec);

        system_clock::time_point t_aftertransptclouds = system_clock::now();

        auto dur_readpixels = t_afterreadpixels - t0;
        duration<double> second_readpixels(dur_readpixels);
        auto dur_interline = t_after_interline - t_afterreadpixels;
        duration<double> second_interline(dur_interline);
        auto dur_trans2ptclouds = t_aftertransptclouds - t_after_interline;
        duration<double> second_trans2ptclouds(dur_trans2ptclouds);
        // std::cout << "Read pixles use " << second_readpixels.count() << "s, " << 
        // "interline use " << second_interline.count() << "s, " << 
        // "trans2ptclouds use " <<  second_trans2ptclouds.count() << "s" << std::endl;
        
        // double  min;
        // double  max;
        // minMaxLoc(image_aft_flip,&min,&max,0,0);
        // // cout << "final_min = " << min << ", final_max = " << max << endl ;
        // image_aft_flip.convertTo( image_aft_flip, CV_8UC1, 255.0 / ( max - min ), -min );
        // cv::applyColorMap( image_aft_flip, image_aft_flip, cv::COLORMAP_JET );
        // cv::imshow("interline Depth", (image_aft_flip));//
        // cv::waitKey(1);

    }
    else if ( depth_buffer_precision == 24 )
    {
        GLuint mypixels[ width * height ];                                                       // There is no 24 bit variable, so we'll have to settle for 32 bit
        glReadPixels( 0, 0, width, height, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT_24_8, mypixels ); // No upconversion.
        cv::Mat image_mat(height, width, CV_16U, mypixels);
      
        cv::imshow("Depth", colormap_depth_img(image_mat));
        cv::waitKey(10);
    }
    else if ( depth_buffer_precision == 32 )
    {
        GLuint mypixels[ width * height ];
        glReadPixels( 0, 0, width, height, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, mypixels );
        cv::Mat image_mat(height, width, CV_16U, mypixels);
        cv::imshow("Depth", colormap_depth_img(image_mat));
        cv::waitKey(10);
    }
    else if ( depth_buffer_precision == 48 )
    {
        GLfloat mypixels[ width * height ];
        glReadPixels( 0, 0, width, height, GL_DEPTH_COMPONENT, GL_FLOAT, mypixels );
        cout << mypixels[ (int)(width * height *0.5 + width*0.5) ] << endl;
        cv::Mat image_mat(height, width, CV_32F, mypixels);
        cv::Mat image_aft_flip;
        cv::flip(image_mat, image_aft_flip , 0 );
        cv::imshow("Depth", colormap_depth_img(image_aft_flip));
        cv::waitKey(1);
    }
}

void opengl_pointcloud_render::read_depth2pointcloud( int depth_buffer_precision , std::vector<Eigen::Matrix<float, 3, 1> >& depth_ptcloud_vec)
{
        int image_width = 2*width;
        int image_height = 2*height;
        GLfloat mypixels[ image_width * image_height ];//GLushort GLuint
        // glReadPixels( (int)(5*width*(0.2+0.8*0.5)) , (int)(5*width*(1-0.8*0.5/640.0*480.0)), (int)(5*width*0.8*0.5), (int)(5*width*0.8*0.5/640.0*480.0), GL_DEPTH_COMPONENT, GL_FLOAT , mypixels );//GL_UNSIGNED_SHORT GL_UNSIGNED_INT
        // (int)(5*width*(1-0.8*0.5*640.0*480.0))
        glReadPixels( (int)(2*width) , (int)(1*height)-30, image_width, image_height, GL_DEPTH_COMPONENT, GL_FLOAT , mypixels );
        // glReadPixels( 0, 0, width, height, GL_DEPTH_COMPONENT, GL_UNSIGNED_SHORT, mypixels );

        cv::Mat image_mat(image_height, image_width, CV_32F, mypixels);
        cv::Mat image_aft_flip;
        cv::flip(image_mat, image_aft_flip , 0 );
        // colormap_depth_img(image_mat);
        // cv::imshow("Depth", );//
        colormap_depth_img(image_aft_flip);
        cv::waitKey(1);

        // depth_interline_on_depthimage(image_aft_flip);
        // faces_need_draw_from_depthimage(image_aft_flip,color_image_mat);
        // new_gaussian_interline(image_aft_flip);
        // interlinebytriangles(image_aft_flip);
        depth_to_pointcloud(image_aft_flip, render_cloud, depth_ptcloud_vec,interline_ptcloud_vec);
        
        double  min;
        double  max;
        minMaxLoc(image_aft_flip,&min,&max,0,0);
        // cout << "final_min = " << min << ", final_max = " << max << endl ;
        image_aft_flip.convertTo( image_aft_flip, CV_8UC1, 255.0 / ( max - min ), -min );
        cv::applyColorMap( image_aft_flip, image_aft_flip, cv::COLORMAP_JET );
        cv::imshow("interline Depth", (image_aft_flip));//
        cv::waitKey(1);

}

float opengl_pointcloud_render::LinearizeDepth(float depth,float near, float far) 
{
    float z = depth * 2.0 - 1.0; // back to NDC 
    return (2.0 * near * far) / (far + near - z * (far - near));    
}

float opengl_pointcloud_render::regainrealdepth(float depth,float near, float far) 
{
    return 1.0/(1.0/near + depth* (1.0/far - 1.0/near));   
}

void opengl_pointcloud_render::test_vector_insert()
{
    std::vector<pcl::PointXYZ> vec_all;
    std::vector<pcl::PointXYZ> vec_temp;
    for(int i = 0;i<1000000;i++)
    {
        pcl::PointXYZ temp_point;
        temp_point.x = 1;
        temp_point.y = 1;
        temp_point.z = 1;
        vec_temp.push_back(temp_point);
    }

        system_clock::time_point t1 = system_clock::now();


    for(int i = 0;i<20;i++)
    {
        // pcl::PointXYZ temp_point;
        // temp_point.x = 1;
        // temp_point.y = 1;
        // temp_point.z = 1;
        // vec_temp.push_back(temp_point);
        // vec_all.resize(vec_all.size()+vec_temp.size());
        vec_all.insert(vec_all.end(),vec_temp.begin(),vec_temp.end());
        // for(std::vector<pcl::PointXYZ>::iterator iter = vec_temp.begin();\
        //     iter!=vec_temp.end();++iter)
        //     {
        //         vec_all.push_back(*iter);
        //     }
    }


        system_clock::time_point t2 = system_clock::now();
        auto dur = t2 - t1;
        duration<double> second(dur);
        std::cout << "One insert cost " << second.count() << " seconds\n";
}

int if_first_call = 1;

void opengl_pointcloud_render::get_Rendering_info()
{
    if(if_first_call == 0)
    {
        return ;
    }
    if_first_call = 0;
    printf("OpenGL version supported by this platform (%s): \n", glGetString(GL_VERSION));
    printf("GL_VERSION: %s\r\n", glGetString(GL_VERSION));
    printf("GL_VENDOR: %s\r\n", glGetString(GL_VENDOR));
    printf("GL_RENDER: %s\r\n", glGetString(GL_RENDER));
    // printf("GL_EXTENSIONS: %s \r\n",  glGetString(GL_EXTENSIONS));
    //  GLint kb;
    // glGetIntegerv(GL_NVX_gpu_memory_info, &kb);
    // printf("GL_NVX_gpu_memory_info: %f \r\n",  kb );
    // printf("GL_GPU_ADDRESS_NV: %s \r\n",  glGetString(GL_GPU_ADDRESS_NV));
}

void opengl_pointcloud_render::printf_pointcloud2_infos(pcl::PCLPointCloud2 & v)
{
    cout << "**** Printf pointcloud2 infos *****" << endl;
    cout << "Header: " << std::endl;
    cout<< v.header;
    cout << "Height: ";
    cout << "  " << v.height << std::endl;
    cout << "Width: ";
    cout << "  " << v.width << std::endl;
    cout << "Pointcloud fields: " << endl;
    //clang-format off
    for ( int i = 0; i < v.fields.size(); i++ )
    {
        cout << "=== Pointcloud field [" << i << "] ===\r\n" << v.fields[ i ];
    }
    //clang-format on
    cout << endl;
}

void opengl_pointcloud_render::load_pcd_file(std::string file_name, pcl::PointCloud<PointType>& cloud_color_mesh)
{
    // mesh = pcl::PolygonMesh();
    // cout << "Loading mesh file from : " << file_name << endl;
    // pcl::io::loadPLYFile( file_name.c_str(), mesh );
    // pcl::fromPCLPointCloud2( mesh.cloud, cloud_color_mesh );

    // cout << "Pointcloud have " << cloud_color_mesh.points.size() << " points." << endl;
    // cout << "Mesh have pointcloud: " << mesh.cloud.width << ", " << mesh.cloud.height << mesh.cloud.row_step << ", " << mesh.cloud.data.size() << ", " << mesh.cloud.data.size() / mesh.cloud.width
    //      << endl;
    // cout << "Size of pcl_points: " << sizeof( pcl::PointXYZRGB ) << ", " << sizeof( pcl::PointXYZRGBA ) << ", " << sizeof( pcl::PointXYZI ) << ", " << sizeof( pcl::PointXYZ ) << endl;
    // cout << "Mesh have polygon: " << mesh.polygons.size() << endl;
    // printf_pointcloud2_infos( mesh.cloud );
    // printf_polygon_infos(mesh);

    int status = pcl::io::loadPCDFile<PointType>(file_name, cloud_color_mesh);
    if (status == -1)
    {
        cout << "can't read file." << endl;
        return ;
    }
}

void opengl_pointcloud_render::load_normal_file(std::string file_name, pcl::PointCloud<pcl::Normal>& normal_ptcloud)
{
    int status = pcl::io::loadPCDFile<pcl::Normal>(file_name, normal_ptcloud);
    if (status == -1)
    {
        cout << "can't read file." << endl;
        return ;
    }
}


inline void opengl_pointcloud_render::glDrawColouredCube_atrandompos(Eigen::Vector3f pos, double length)
{
    const GLfloat x_min = pos(0) - 0.5*length;
    const GLfloat x_max = pos(0) + 0.5*length;
    const GLfloat y_min = pos(1) - 0.5*length;
    const GLfloat y_max = pos(1) + 0.5*length;
    const GLfloat z_min = pos(2) - 0.5*length;
    const GLfloat z_max = pos(2) + 0.5*length;

    //draw a cube
    // GLfloat verts[72];
    const GLfloat verts[] = {
        x_min,y_min,z_max,  x_max,y_min,z_max,  x_min,y_max,z_max,  x_max,y_max,z_max,  // FRONT
        x_min,y_min,z_min,  x_min,y_max,z_min,  x_max,y_min,z_min,  x_max,y_max,z_min,  // BACK
        x_min,y_min,z_max,  x_min,y_max,z_max,  x_min,y_min,z_min,  x_min,y_max,z_min,  // LEFT
        x_max,y_min,z_min,  x_max,y_max,z_min,  x_max,y_min,z_max,  x_max,y_max,z_max,  // RIGHT
        x_min,y_max,z_max,  x_max,y_max,z_max,  x_min,y_max,z_min,  x_max,y_max,z_min,  // TOP
        x_min,y_min,z_max,  x_min,y_min,z_min,  x_max,y_min,z_max,  x_max,y_min,z_min   // BOTTOM
    };

    // const GLfloat l = axis_min;
    // const GLfloat h = axis_max;
    
    // const GLfloat verts[] = {
    //     l,l,h,  h,l,h,  l,h,h,  h,h,h,  // FRONT
    //     l,l,l,  l,h,l,  h,l,l,  h,h,l,  // BACK
    //     l,l,h,  l,h,h,  l,l,l,  l,h,l,  // LEFT
    //     h,l,l,  h,h,l,  h,l,h,  h,h,h,  // RIGHT
    //     l,h,h,  h,h,h,  l,h,l,  h,h,l,  // TOP
    //     l,l,h,  l,l,l,  h,l,h,  h,l,l   // BOTTOM
    // };
    
    glVertexPointer(3, GL_FLOAT, 0, verts);
    glEnableClientState(GL_VERTEX_ARRAY);
    
    glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    glDrawArrays(GL_TRIANGLE_STRIP, 4, 4);
    
    glColor4f(0.0f, 1.0f, 0.0f, 1.0f);
    glDrawArrays(GL_TRIANGLE_STRIP, 8, 4);
    glDrawArrays(GL_TRIANGLE_STRIP, 12, 4);
    
    glColor4f(0.0f, 0.0f, 1.0f, 1.0f);
    glDrawArrays(GL_TRIANGLE_STRIP, 16, 4);
    glDrawArrays(GL_TRIANGLE_STRIP, 20, 4);
    
    glDisableClientState(GL_VERTEX_ARRAY);
}

struct BGR
{
    uchar b;
    uchar g;
    uchar r;
};
 
struct HSV
{
    int h;
    double s;
    double v;
};

void opengl_pointcloud_render::HSV2BGR(HSV &hsv, BGR &bgr)
{
    int h = hsv.h;
    double s = hsv. s;
    double v = hsv. v;
    double b = 0.0;
    double g = 0.0;
    double r = 0.0;
 
    int flag = (int)abs(h / 60.0);
    double f = h/60.0 - flag;
    double p = v * (1 - s);
    double q = v * (1 - f*s);
    double t = v * (1 - (1- f)*s);
 
    switch (flag)
    {
    case 0:
         b = p;
         g = t;
         r = v;
         break;
    case 1:
         b = p;
         g = v;
         r = q;
         break;
    case 2:
         b = t;
         g = v;
         r = p;
         break;
    case 3:
         b = v;
         g = q;
         r = p;
         break;
    case 4:
         b = v;
         g = p;
         r = t;
         break;
    case 5:
         b = q;
         g = p;
         r = v;
         break;
    default:
         break;
    }
 
    int blue = int(b * 255);
    bgr.b = (blue > 255) ? 255 : blue;
    bgr.b = (blue < 0) ? 0 : bgr.b;
 
    int green = int(g * 255);
    bgr.g = (green > 255) ? 255 : green;
    bgr.g = (green < 0) ? 0 : bgr.g;
 
    int red = int(r * 255);
    bgr.r = (red > 255) ? 255 : red;
    bgr.r = (red < 0) ? 0 : bgr.r;
}

void opengl_pointcloud_render::init_pointcloud_data()
{
    // preprocess(cloud_color_mesh,pointcloud_normals);
    // load_normal_file("/home/dji/meshmap/normal_files/Knowles_002_normals.pcd",*pointcloud_normals);

  //trans the map into hash map
  PointType pt_in,center;
  long int ind_x,ind_y,ind_z;

  //get max xyz
  PointType global_mapmin;
  PointType global_mapmax;
  pcl::getMinMax3D(cloud_color_mesh,global_mapmin,global_mapmax);
//   map_min(0) = global_mapmin.x;
//   map_min(1) = global_mapmin.y;
//   map_min(2) = global_mapmin.z;
//   map_max(0) = global_mapmax.x;
//   map_max(1) = global_mapmax.y;
//   map_max(2) = global_mapmax.z;

  cube_numx = floor((global_mapmax.x - global_mapmin.x)/hash_cubesize) + 20;
  cube_numy = floor((global_mapmax.y - global_mapmin.y)/hash_cubesize) + 20;
  cube_numz = floor((global_mapmax.z - global_mapmin.z)/hash_cubesize) + 200;

    env_box.vertex_min[0] = round(global_mapmin.x/hash_cubesize) * hash_cubesize - 10*hash_cubesize;
    env_box.vertex_min[1] = round(global_mapmin.y/hash_cubesize) * hash_cubesize - 10*hash_cubesize;
    env_box.vertex_min[2] = round(global_mapmin.z/hash_cubesize) * hash_cubesize - 100*hash_cubesize;
    env_box.vertex_max[0] = round(global_mapmax.x/hash_cubesize) * hash_cubesize + 10*hash_cubesize;
    env_box.vertex_max[1] = round(global_mapmax.y/hash_cubesize) * hash_cubesize + 10*hash_cubesize;
    env_box.vertex_max[2] = round(global_mapmax.z/hash_cubesize) * hash_cubesize + 100*hash_cubesize;
    // fov_checker.Set_Env(env_box);
    // fov_checker.Set_BoxLength(hash_cubesize);

    // scope_color(ANSI_COLOR_YELLOW_BOLD);
    cout << "Now initing the point cloud data: " << endl;
    g_eigen_tri_rgb_vec.clear();
    g_eigen_tri_pt_vec.clear();
    // g_pointcloud_vec.clear();
    g_eigen_pt_vec.clear();
    // g_eigen_rgb_vec.clear();
    g_total_point_size = cloud_color_mesh.size();
    // g_pointcloud_vec.resize(  cloud_color_mesh.size() * g_point_step );
    Eigen::Matrix<float, 3, 1>  eigen_pt;
    Eigen::Matrix<float, 3, 1 >  rgb_pt;
    g_eigen_pt_vec.resize( cloud_color_mesh.size()*2);
    // g_eigen_rgb_vec.reserve( cloud_color_mesh.size() );
    // points_index_infov.resize(cloud_color_mesh.size());
    for ( int i = 0; i < cloud_color_mesh.size(); i++ )
    {
        // g_pointcloud_vec[ i * g_point_step ] = cloud_color_mesh.points[ i ].x;
        // g_pointcloud_vec[ i * g_point_step + 1 ] = cloud_color_mesh.points[ i ].y;
        // g_pointcloud_vec[ i * g_point_step + 2 ] = cloud_color_mesh.points[ i ].z;
        
        eigen_pt << cloud_color_mesh.points[ i ].x, cloud_color_mesh.points[ i ].y, cloud_color_mesh.points[ i ].z;
        rgb_pt << 0,0,1;
        // rgb_pt << cloud_color_mesh.points[ i ].r, cloud_color_mesh.points[ i ].g, cloud_color_mesh.points[ i ].b;
        
        g_eigen_pt_vec[ 2*i ] = eigen_pt;
        g_eigen_pt_vec[ 2*i+1 ] = rgb_pt;
        // g_eigen_rgb_vec[ i ] = rgb_pt;

        //put pts into fov boxes
        pt_in = cloud_color_mesh.points[i];

        ind_x = (round((pt_in.x - env_box.vertex_min[0] + EPSS)/hash_cubesize));
        ind_y = (round((pt_in.y - env_box.vertex_min[1] + EPSS)/hash_cubesize));
        ind_z = (round((pt_in.z - env_box.vertex_min[2] + EPSS)/hash_cubesize));

        long int box_index = ind_x + ind_y*cube_numx + ind_z*cube_numx*cube_numy;
        // point_hashmap.insert(pair<int, >(box_index, pt_in));
        // point_hashmap[box_index].push_back(pt_in);
        // pointindex_hashmap[box_index].push_back(i);

        //EBO index vector
        // if(eigen_pt(1)>5.0)
        // {
            points_index_infov.push_back(i);
        // }
        
    }

    cout << "Number of points = " << g_eigen_pt_vec.size() / 20000.0 << "X 10000" << endl;
    cout << "Number of faces = " << g_eigen_tri_pt_vec.size() / 10000.0 << "X 10000" << endl;

    cloud_color_mesh.clear();
}


inline void opengl_pointcloud_render::glDrawColouredCube_alpha(GLfloat axis_min=-0.5f, GLfloat axis_max = +0.5f, float alpha = 1.0)
{
    // const GLfloat x_min = pos(0) - 0.5*length;
    // const GLfloat x_max = pos(0) + 0.5*length;
    // const GLfloat y_min = pos(1) - 0.5*length;
    // const GLfloat y_max = pos(1) + 0.5*length;
    // const GLfloat z_min = pos(2) - 0.5*length;
    // const GLfloat z_max = pos(2) + 0.5*length;

    // //draw a cube
    // // GLfloat verts[72];
    // const GLfloat verts[] = {
    //     x_min,y_min,z_max,  x_max,y_min,z_max,  x_min,y_max,z_max,  x_max,y_max,z_max,  // FRONT
    //     x_min,y_min,z_min,  x_min,y_max,z_min,  x_max,y_min,z_min,  x_max,y_max,z_min,  // BACK
    //     x_min,y_min,z_max,  x_min,y_max,z_max,  x_min,y_min,z_min,  x_min,y_max,z_min,  // LEFT
    //     x_max,y_min,z_min,  x_max,y_max,z_min,  x_max,y_min,z_max,  x_max,y_max,z_max,  // RIGHT
    //     x_min,y_max,z_max,  x_max,y_max,z_max,  x_min,y_max,z_min,  x_max,y_max,z_min,  // TOP
    //     x_min,y_min,z_max,  x_min,y_min,z_min,  x_max,y_min,z_max,  x_max,y_min,z_min   // BOTTOM
    // };

    const GLfloat l = axis_min;
    const GLfloat h = axis_max;
    
    const GLfloat verts[] = {
        l,l,h,  h,l,h,  l,h,h,  h,h,h,  // FRONT
        l,l,l,  l,h,l,  h,l,l,  h,h,l,  // BACK
        l,l,h,  l,h,h,  l,l,l,  l,h,l,  // LEFT
        h,l,l,  h,h,l,  h,l,h,  h,h,h,  // RIGHT
        l,h,h,  h,h,h,  l,h,l,  h,h,l,  // TOP
        l,l,h,  l,l,l,  h,l,h,  h,l,l   // BOTTOM
    };
    // Enable this function to blend alpha.
    glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
    glEnable( GL_BLEND );
    glVertexPointer(3, GL_FLOAT, 0, verts);
    glEnableClientState(GL_VERTEX_ARRAY);
    
    glColor4f(1.0f, 0.0f, 0.0f, alpha);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    glDrawArrays(GL_TRIANGLE_STRIP, 4, 4);
    
    glColor4f(0.0f, 1.0f, 0.0f, alpha);
    glDrawArrays(GL_TRIANGLE_STRIP, 8, 4);
    glDrawArrays(GL_TRIANGLE_STRIP, 12, 4);
    
    glColor4f(0.0f, 0.0f, 1.0f, alpha);
    glDrawArrays(GL_TRIANGLE_STRIP, 16, 4);
    glDrawArrays(GL_TRIANGLE_STRIP, 20, 4);
    
    glDisableClientState(GL_VERTEX_ARRAY);
}

template<typename T>
void opengl_pointcloud_render::get_real_near_far( cv::Mat  & depth_mat , double & min, double & max )
{
    int temp_width = depth_mat.cols;
    int temp_height = depth_mat.rows;
    int size = temp_width * temp_height;
    min = 3e8;
    max = -3e8;
    for ( int i = 0; i < size; i++ )
    {
        T val = depth_mat.at< T >( i / temp_width, i % temp_height );
        if ( val == 65535 )
        {
            depth_mat.at< T >( i / temp_width, i % temp_height ) = 0;
            continue;
        }    
        if ( min > val )
        {
            min = val;
        }
        if ( max < val )
        {
            max = val;
        }
    }

    for ( int i = 0; i < size; i++ )
    {
        T val = depth_mat.at< T >( i / temp_width, i % temp_height );
        if ( val == 65535 )
        {
            depth_mat.at< T >( i / temp_width, i % temp_height ) = 0;
            // continue;
        }    
    }
}

void opengl_pointcloud_render::culling_depth_image( cv::Mat  & depth_mat)
{

#pragma omp parallel default (none) \
                     shared (depth_mat)
{
#pragma omp for nowait collapse(2)
    for (int i = 0; i < depth_mat.rows; ++i) {
        for (int j = 0; j < depth_mat.cols; ++j) {
            uint16_t depth = depth_mat.at<uint16_t>(i, j);

            if(depth < 30000)
            {
                continue;
            }

            // std::cout<<"("<<i<<","<<j<<") depth = " << depth<<std::endl;
            // continue;

            int half_cover_angle = 50;
            int theta_start = i - half_cover_angle;
            int theta_end = i + half_cover_angle;
            int fi_start = j - half_cover_angle;
            int fi_end = j + half_cover_angle;
              for (int theta_index_o = theta_start;
               theta_index_o <= theta_end; theta_index_o++) {
              for (int fi_index_o = fi_start;
                   fi_index_o <= fi_end; fi_index_o = fi_index_o+1) {

                  if(((theta_index_o > (depth_mat.rows - 1))||(theta_index_o < 0)||(fi_index_o > (depth_mat.cols - 1))||(fi_index_o < 0)))
                  {continue;}
                  if (depth_mat.at<uint16_t>(theta_index_o, fi_index_o) > depth) {// + sensing_horizon
                        depth_mat.at<uint16_t>(theta_index_o, fi_index_o) = depth;// + sensing_horizon
                  }
                   }
               }
        }
    }
}
}

//Triangle interline method attempt but failed
//check if the point is in triangle
inline bool opengl_pointcloud_render::is_in_triangle(Eigen::Vector3f p, Eigen::Vector3f a, Eigen::Vector3f b, Eigen::Vector3f c, Eigen::Vector3f& weights)
{
    Eigen::Vector3f v0 = b - a;
    Eigen::Vector3f v1 = c - a;
    Eigen::Vector3f v2 = p - a;
    float dot00 = v0.dot(v0);
    float dot01 = v0.dot(v1);
    float dot02 = v0.dot(v2);
    float dot11 = v1.dot(v1);
    float dot12 = v1.dot(v2);
    float invDenom = 1 / (dot00 * dot11 - dot01 * dot01);
    float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
    float v = (dot00 * dot12 - dot01 * dot02) * invDenom;
    weights(0) = 1 - u - v;
    weights(1) = u;
    weights(2) = v;
    return ((u >= 0) && (v >= 0) && (u + v < 1));
}

//interpolation in triangles
void opengl_pointcloud_render::interlinebytriangles(cv::Mat& depth_image)
{
    pcl::search::KdTree<PointType> kdtree_2d;
    pcl::PointCloud<PointType>::Ptr cloud_2d(new pcl::PointCloud<PointType>);

    //input all points into kdtree
#pragma omp parallel default (none) \
                     shared (depth_image, cloud_2d, kdtree_2d, far)
      {
#pragma omp for nowait collapse(2)
        for(int i = 0; i < depth_image.rows; i++)
        {
            for(int j = 0; j < depth_image.cols; j++)
            {
                if(depth_image.at<float>(i,j) < far)
                {
                    //get the point
                    PointType pt;
                    pt.x = j;
                    pt.y = i;
                    pt.z = 0;
                    //add to pointcloud
                    #pragma omp critical
                    {
                        cloud_2d->points.push_back(pt);
                    }
                }else{

                }
            }
        }
      }
    kdtree_2d.setInputCloud(cloud_2d);

    //show to debug
    Mat debug_image = Mat::zeros(depth_image.rows, depth_image.cols, CV_8UC3);

    //check up all the pixels to interline
    int search_num = 10;
    std::vector<int> pointIdxNKNSearch(search_num);
    std::vector<float> pointNKNSquaredDistance(search_num);    
    std::vector<Eigen::Vector3f> nearest_points;
    for(int i = 0; i < depth_image.rows; i++)
    {
        for(int j = 0; j < depth_image.cols; j++)
        {
            //search the nearest neighbors, and interline
            if(depth_image.at<float>(i,j) >= far-0.5)
            {
                // debug_image.at<Vec3b>(i,j) = Vec3b(0,0,255);

                //get the point
                PointType pt;
                pt.x = j;
                pt.y = i;
                pt.z = 0;
                nearest_points.clear();
                //search the nearest neighbor
                if(kdtree_2d.nearestKSearch(pt, search_num, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
                {
                    debug_image.at<Vec3b>(i,j) = Vec3b(0,255,0);

                    //get the nearest neighbor points
                    for(int k = 0; k < search_num; k++)
                    {
                        Eigen::Vector3f pt_temp;
                        pt_temp(0) = cloud_2d->points[pointIdxNKNSearch[k]].x;
                        pt_temp(1) = cloud_2d->points[pointIdxNKNSearch[k]].y;
                        pt_temp(2) = depth_image.at<float>((int)pt_temp(1),(int)pt_temp(0));
                        nearest_points.push_back(pt_temp);
                    }
                    //sort the points by z
                    std::sort(nearest_points.begin(), nearest_points.end(), [](const Eigen::Vector3f& a, const Eigen::Vector3f& b)
                    {
                        return a(2) < b(2);
                        // return a.norm() < b.norm();
                    });
                    //print sort result
                    // for(int k = 0; k < nearest_points.size(); k++)
                    // {
                    //     cout << nearest_points[k](0) << " " << nearest_points[k](1) << " " << nearest_points[k](2) << endl;
                    // }

                    //get the three nearest point
                    Eigen::Vector3f pt_a,pt_b,pt_c;
                    pt_a = nearest_points[0];
                    pt_b = nearest_points[1];
                    pt_c = nearest_points[2];

                    //check if the point is in triangle
                    Eigen::Vector3f p,a,b,c,weights;
                    p = Eigen::Vector3f(j,i,0);
                    a = nearest_points[0];
                    a(2) = 0;
                    b = nearest_points[1];
                    b(2) = 0;
                    c = nearest_points[2];
                    c(2) = 0;
                    if(is_in_triangle(p,a,b,c,weights))
                    {
                        //interpolation
                        float z = weights(0) * pt_a(2) + weights(1) * pt_b(2) + weights(2) * pt_c(2);
                        depth_image.at<float>(i,j) = z;

                        debug_image.at<Vec3b>(i,j) = Vec3b(255,0,0);
                    }else{
                        pt_c = nearest_points[3];   
                        c = pt_c;
                        c(2) = 0;
                        if(is_in_triangle(p,a,b,c,weights))
                        {
                            //interpolation
                            float z = weights(0) * pt_a(2) + weights(1) * pt_b(2) + weights(2) * pt_c(2);
                            depth_image.at<float>(i,j) = z;

                            debug_image.at<Vec3b>(i,j) = Vec3b(255,0,0);
                        }
                    }

                    // //draw nearest points in debug image
                    // for(int k = 0; k < search_num; k++)
                    // {
                    //     debug_image.at<Vec3b>(cloud_2d->points[pointIdxNKNSearch[k]].y,cloud_2d->points[pointIdxNKNSearch[k]].x) = Vec3b(0,0,255);
                    // }


                }
            }
        }
    }
    //show the debug image
    imshow("debug", debug_image);
    waitKey(1);                    

}

bool opengl_pointcloud_render::is_in_fov(Eigen::Vector3f pt, Eigen::Vector3f pos, Eigen::Vector3f fov_direction, double fov_angle)
{
    //get the direction vector
    fov_direction.normalize();
    Eigen::Vector3f direction_vector;
    direction_vector = pt-pos;
    direction_vector.normalize();

    //compute if it is in fov
    double cos_angle = fov_direction.dot(direction_vector);
    double fov_angle_rad = fov_angle * M_PI / 180;
    if(abs(cos_angle) < abs(cos(fov_angle_rad)))
    {
        return false;
    }else{
        return true;
    }
}

int opengl_pointcloud_render::count_fov_point_num(pcl::PointCloud< PointType > cloud_all_map, Eigen::Vector3f pos, Eigen::Vector3f fov_direction,\
                         double sensing_range, double half_fov_angle)
{
    //pcl kdtree radius search
    pcl::KdTreeFLANN<PointType> kdtree;
    kdtree.setInputCloud(cloud_all_map.makeShared());

    vector<float> diss;
    vector<int> indexs;
    PointType search_point;
    search_point.x = pos(0);
    search_point.y = pos(1);
    search_point.z = pos(2);
    // if(kdtree.radiusSearch(search_point, sensing_range, indexs, diss)>0)
    // {
    //     //get the points in the fov
    //     vector<int> fov_indexs;
    //     for(int i = 0; i < indexs.size(); i++)
    //     {
    //         Eigen::Vector3f pt_temp;
    //         pt_temp(0) = cloud_all_map.points[indexs[i]].x;
    //         pt_temp(1) = cloud_all_map.points[indexs[i]].y;
    //         pt_temp(2) = cloud_all_map.points[indexs[i]].z;
    //         if(is_in_fov(pt_temp, pos, fov_direction, half_fov_angle))
    //         {
    //             fov_indexs.push_back(indexs[i]);
    //         }
    //     }
    //     return (int)fov_indexs.size();
    // }else{
    //     return 0;
    // }

    //count running time
    system_clock::time_point t1 = system_clock::now();

    //search nearest point
    vector<int> pointIdxNKNSearch;
    vector<float> pointNKNSquaredDistance;
    int search_num = kdtree.nearestKSearch(search_point, 1, pointIdxNKNSearch, pointNKNSquaredDistance);

    system_clock::time_point t2 = system_clock::now();
        auto dur = t2 - t1;
        duration<double> second(dur);
        std::cout << "One search in kdtree cost " << second.count() << " seconds\n";
}

void opengl_pointcloud_render::new_gaussian_interline(cv::Mat  & depth_mat)
{

    //count running time
    system_clock::time_point t1 = system_clock::now();

    int width = depth_mat.cols;
    int height = depth_mat.rows;

    //matrix vector
    vector<vector<float>> depth_mat_interline_vector;
    depth_mat_interline_vector.resize(height*width);

    vector<Eigen::Vector3f> shouldinterline_ptvec;
    vector<Eigen::Vector2i> shouldinterline_ptvec_index;

    for (int v = 0; v < height; v++)
    {
        for (int u = 0; u < width; u++)
        {
            float depth = depth_mat.at<float>(v, u);
            if (depth > near && depth<far)
            {
                Eigen::Vector3f temp_point, temp_point_world;
                temp_point(0) = (u - u0) * depth / fu;
                temp_point(1) = -(v - v0) * depth / fv;
                temp_point(2) = -depth;

                if(temp_point.norm() < effect_range)
                {
                    shouldinterline_ptvec.push_back(temp_point);
                    shouldinterline_ptvec_index.push_back(Eigen::Vector2i(u,v));
                    
                    float theta = atan2(temp_point(0),depth);
                    float fi = atan2(temp_point(1),depth);
                    // float cover_dis = 0.55 * 1.7321 * downsample_res;// 0.707
                    float polar_resolution_x = 1.0/fu*(cos(theta)*cos(theta))*180/M_PI;
                    float polar_resolution_y = 1.0/fv*(cos(fi)*cos(fi))*180/M_PI;

                    int half_cover_angle_x = 2*ceil(
                            (asin(cover_dis / temp_point.norm()) / (M_PI * polar_resolution_x / 180.0))); 
                    int half_cover_angle_y = 2*ceil(
                            (asin(cover_dis / temp_point.norm()) / (M_PI * polar_resolution_y / 180.0)));

                    int theta_start = u - half_cover_angle_x;
                    int theta_end = u + half_cover_angle_x;
                    int fi_start = v - half_cover_angle_y;
                    int fi_end = v + half_cover_angle_y;
                
                    for (int theta_index_o = theta_start;
                        theta_index_o <= theta_end; theta_index_o++) {
                    for (int fi_index_o = fi_start;
                        fi_index_o <= fi_end; fi_index_o = fi_index_o+1) {
                            if(theta_index_o < 0 || theta_index_o >= width || fi_index_o < 0 || fi_index_o >= height)
                                {continue;}
                            
                            float pixel_dist = sqrt(pow(theta_index_o - u, 2) + pow(fi_index_o - v, 2));
                            depth_mat_interline_vector[theta_index_o + fi_index_o*width].push_back(depth);
                        }
                    }

                }
                
            }else{

            }
        }
    }

    // cout << "shouldinterline_ptvec size = " << shouldinterline_ptvec.size() << endl;

    //depth interline points on depth image
    for (int v = 0; v < height; v++)
    {
        for (int u = 0; u < width; u++)
        {
            float depth = depth_mat.at<float>(v, u);
            if(depth_mat_interline_vector[u + v*width].size() > 0)//depth>(far-0.5) &&
            {
                float summ = 0;
                //delete so far points
                int minValue = *min_element(depth_mat_interline_vector[u + v*width].begin(), depth_mat_interline_vector[u + v*width].end());
                for(int i = 0;i<depth_mat_interline_vector[u + v*width].size();i++)
                {
                    if(depth_mat_interline_vector[u + v*width][i] > minValue+10*downsample_res)
                    {
                        depth_mat_interline_vector[u + v*width].erase(depth_mat_interline_vector[u + v*width].begin() + i);
                        i = i-1;
                        // continue;
                    }
                    // summ += depth_mat_interline_vector[u + v*width][i];
                }

                //get average depth
                // float summ = std::accumulate(depth_mat_interline_vector[u + v*width].begin(), depth_mat_interline_vector[u + v*width].end(), 0);
                
                //printf vector 
                for(int i = 0;i<depth_mat_interline_vector[u + v*width].size();i++)
                {
                    // printf("%f ",depth_mat_interline_vector[u + v*width][i]);
                    // std::cout << depth_mat_interline_vector[u + v*width][i] << " ";
                    summ += depth_mat_interline_vector[u + v*width][i];
                }
                // std::cout <<" sum = "<<summ <<  endl;                
                
                depth_mat.at<float>(v, u) = summ / depth_mat_interline_vector[u + v*width].size();
            }
        }
     
    }

    system_clock::time_point t2 = system_clock::now();
        auto dur = t2 - t1;
        duration<double> second(dur);
        // std::cout << "One frame interline costs " << second.count() << " seconds\n";

        cv::Mat show_img;
        double  min;
        double  max;
        minMaxLoc(depth_mat,&min,&max,0,0);
        depth_mat.convertTo( show_img, CV_8UC1, 255.0 / ( max - min ), -min );
        cv::applyColorMap( show_img, show_img, cv::COLORMAP_JET );
        cv::imshow("new Depth", (show_img));//
        cv::waitKey(1);   
}