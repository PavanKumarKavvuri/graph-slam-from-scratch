#include "graph-slam-from-scratch/graph_slam.hpp"


namespace graph_slam_ns{

    GraphSLAM::GraphSLAM(const rclcpp::NodeOptions & options): rclcpp::Node("graph_slam_node", options){
        RCLCPP_INFO(this->get_logger(), "Custom Graph SLAM Node Initialised!");

        GraphSLAM::initialise_pub_subs();

    }

    void GraphSLAM::initialise_pub_subs(){

        rclcpp::QoS qos_settings_ = rclcpp::QoS(1).reliable();

        m_lls_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/lls", qos_settings_,
            std::bind(&GraphSLAM::gtPoseCallback, this, std::placeholders::_1));

        m_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", qos_settings_,
            std::bind(&GraphSLAM::odomCallback, this, std::placeholders::_1));

    }

    void GraphSLAM::gtPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
        RCLCPP_INFO_STREAM(this->get_logger(), "Ground Truth Pose Received: [" << msg->pose.position.x << ", " << msg->pose.position.y << "]");
    }

    void GraphSLAM::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
        RCLCPP_INFO_STREAM(this->get_logger(), "Ground Truth Odometry Received: [" << msg->pose.pose.position.x << ", " << msg->pose.pose.position.y << "]");
    }


    GraphSLAM::~GraphSLAM(){
        RCLCPP_INFO(this->get_logger(), "Custom Graph SLAM Node Destroyed!");
    }

}