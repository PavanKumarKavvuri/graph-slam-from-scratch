#pragma once

#include "iostream"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "vector"
#include "cmath"
#include "Eigen/Dense"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "graph-slam-from-scratch/edge_struct.hpp"
#include "graph-slam-from-scratch/node_struct.hpp"


namespace graph_slam_ns{

    class GraphSLAM : public rclcpp::Node{

        public:
            
            GraphSLAM(const rclcpp::NodeOptions & options);
            
            ~GraphSLAM();

        private:
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_sub;
            rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_lls_sub;
            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_node_marker_pub;
            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_edge_marker_pub;

            
            std::vector<graph_slam_ns::Node2D> NodesArray{};
            std::vector<graph_slam_ns::Edge2D> EdgesArray{};
            geometry_msgs::msg::Point last_point_{};
            graph_slam_ns::Node2D previous_node_;

            visualization_msgs::msg::Marker m_edge_marker;

            int nodeCounter{0};
            bool is_firstFrame = true;
            double euc_dist{};
            double distance_travelled{};

            void initialise_pub_subs();

            void gtPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
            void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

            double computeDistance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2);
            double extractTheta(const geometry_msgs::msg::Quaternion& q);

            void publishNodeMarker(const graph_slam_ns::Node2D& node);
            visualization_msgs::msg::Marker create_marker_obj();
            void createEdgesMarker_Obj();
            void publishEdgeMarker(const graph_slam_ns::Node2D& prev, const graph_slam_ns::Node2D& current, const std::string& edge_type);

    };

    inline std::ostream& operator << (std::ostream& os, const graph_slam_ns::Node2D& pose) {
        os << "(";
        os << pose.x << ", " << pose.y << ", " << pose.theta;
        os << ")";
        os << " Node id : " << pose.node_id;

        return os;
    }

            
}