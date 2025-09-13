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
        // RCLCPP_INFO_STREAM(this->get_logger(), "Ground Truth Pose Received: [" << msg->pose.position.x << ", " << msg->pose.position.y << "]");
        // frameCount += 1;
    }

    void GraphSLAM::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
        
        geometry_msgs::msg::Point current_point = msg->pose.pose.position;
        geometry_msgs::msg::Quaternion current_quat = msg->pose.pose.orientation;

        if(is_firstFrame){
            RCLCPP_INFO(this->get_logger(), "First Frame Received");
            is_firstFrame = false;
            // nodeCounter += 1;

            auto theta = extractTheta(current_quat);

            Pose2D pose_2d{ .x = current_point.x,
                            .y = current_point.y,
                            .theta = theta,
                            .node_id = nodeCounter
                            };
            PoseArray.push_back(pose_2d);
        }
        else{
            auto dist = computeDistance(current_point, last_point_);
            distance_travelled += dist;
            
            if(distance_travelled > 10){
                RCLCPP_INFO(this->get_logger(), "Distance greater than 15mts. Dropping node now");
                
                nodeCounter += 1;
                auto theta = extractTheta(current_quat);

                Pose2D pose_2d{ .x = current_point.x,
                                .y = current_point.y,
                                .theta = theta,
                                .node_id = nodeCounter
                                };
                PoseArray.push_back(pose_2d);

                std::cout<< "Node id: " << nodeCounter << " " << pose_2d << std::endl;

                last_point_ = current_point;
                distance_travelled = 0.0;

                std::cout<< "Vector size: "<< PoseArray.size() << std::endl;
            }
        }
    }

    double GraphSLAM::computeDistance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2){
        return std::sqrt(
            std::pow(p2.x - p1.x, 2) +
            std::pow(p2.y - p1.y, 2)
        );
    }

    double GraphSLAM::extractTheta(const geometry_msgs::msg::Quaternion& q){
        Eigen::Quaterniond quaternion(q.w, q.x, q.y, q.z);
        Eigen::Vector3d euler_ang = quaternion.toRotationMatrix().eulerAngles(2, 1, 0); // Z, Y, X

        return euler_ang(0);
    }


    GraphSLAM::~GraphSLAM(){
        RCLCPP_INFO(this->get_logger(), "Custom Graph SLAM Node Destroyed!");
    }

}