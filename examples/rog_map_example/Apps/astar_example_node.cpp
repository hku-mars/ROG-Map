#include "rog_astar/rog_astar.hpp"


void rvizClickCallback(const geometry_msgs::PoseStampedConstPtr& msg);

rog_astar::AStar::Ptr rog_astar_ptr;
ros::Publisher mkr_pub;


void publishPointWithText(const rog_map::Vec3f& p,
                          const std::string& text,
                          const rog_map::Color c = rog_map::Color::Green());

int main(int argc, char** argv) {
    ros::init(argc, argv, "rm_node");
    ros::NodeHandle nh("~");

    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    /* 1. Creat a ROGMap ptr*/
    rog_map::ROGMap::Ptr rog_map_ptr = std::make_shared<rog_map::ROGMap>(nh);

    /* 2. Creat a path search module and input the map ptr*/
    rog_astar_ptr = std::make_shared<rog_astar::AStar>(nh, rog_map_ptr);

    /* 3. Creat some interactive nodes */
    ros::Subscriber rviz_click_sub = nh.subscribe("/goal", 1, &rvizClickCallback);
    mkr_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);

    /* Publisher and subcriber */
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::Duration(1.0).sleep();

    /* Run an example first*/
    rog_astar::Vec3f start, goal;
    rog_astar_ptr->getExampleStartGoal(start, goal);

    publishPointWithText(start, "start", rog_map::Color::Orange());
    publishPointWithText(goal, "goal", rog_map::Color::Green());

    const auto ret = rog_astar_ptr->runExample();
    if (ret) {
        ROS_INFO("Path found");
    }
    else {
        ROS_ERROR("Path not found");
    }

    ros::waitForShutdown();
    return 0;
}


void publishPointWithText(const rog_map::Vec3f& p, const std::string& text, const rog_map::Color c) {
    visualization_msgs::Marker point_marker;
    point_marker.header.frame_id = "world";
    point_marker.header.stamp = ros::Time::now();
    point_marker.ns = text + "_pos";
    point_marker.id = 0;
    point_marker.type = visualization_msgs::Marker::SPHERE;
    point_marker.action = visualization_msgs::Marker::ADD;
    point_marker.pose.position.x = p(0);
    point_marker.pose.position.y = p(1);
    point_marker.pose.position.z = p(2);
    point_marker.pose.orientation.w = 1.0;
    point_marker.scale.x = 0.2;
    point_marker.scale.y = 0.2;
    point_marker.scale.z = 0.2;
    point_marker.color = c;
    point_marker.color.a = 1.0;


    visualization_msgs::Marker text_marker;
    text_marker.header.frame_id = "world";
    text_marker.header.stamp = ros::Time::now();
    text_marker.ns = text;
    text_marker.id = 1;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.pose.position.x = p(0);
    text_marker.pose.position.y = p(1);
    text_marker.pose.position.z = p(2) + 0.3;
    text_marker.pose.orientation.w = 1.0;
    text_marker.scale.z = 0.5;
    text_marker.color = c;
    text_marker.color.a = 1.0;
    text_marker.text = text;

    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.push_back(point_marker);
    marker_array.markers.push_back(text_marker);
    mkr_pub.publish(marker_array);
}


void rvizClickCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
    ROS_INFO("Note, the click height is set to 1.0 to ease the user interaction.");
    ROS_INFO("x: %f, y: %f, z: %f", msg->pose.position.x, msg->pose.position.y, 1.0);
    static rog_map::Vec3f start_pos, goal_pos;
    static bool is_start = true;

    /// NOTE the click heigh is set to 1.0 to ease the user interaction in Rviz
    if (is_start) {
        start_pos = rog_map::Vec3f(msg->pose.position.x, msg->pose.position.y, 1.0);
        is_start = false;
        publishPointWithText(start_pos, "start", rog_map::Color::Orange());
    }
    else {
        goal_pos = rog_map::Vec3f(msg->pose.position.x, msg->pose.position.y, 1.0);

        /* Two points got, start one plan */
        publishPointWithText(goal_pos, "goal", rog_map::Color::Green());
        /* 1) set the searching settings */
        int flag = rog_astar::UNKNOWN_AS_FREE | rog_astar::ON_INF_MAP;
        const auto ret = rog_astar_ptr->pathSearch(start_pos, goal_pos, 0.1, flag);
        if (ret) {
            ROS_INFO("Path found");
        }
        else {
            ROS_ERROR("Path not found");
        }

        is_start = true;
    }
}
