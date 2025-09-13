#include "graph-slam-from-scratch/visualiser.hpp"

namespace graph_slam_ns{

    GraphVisualiser::GraphVisualiser(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub)
        : m_node_marker_pub(pub) {

        // GraphVisualiser::initialise_pub_subs();

    }
    GraphVisualiser::~GraphVisualiser()= default;

    // void GraphVisualiser::initialise_pub_subs(){

    //     rclcpp::QoS qos_settings_ = rclcpp::QoS(1).reliable();

    //     m_node_marker_pub = this->create_publisher<visualization_msgs::msg::Marker>(
    //         "visualization_marker", qos_settings_);

    // }

    auto GraphVisualiser::create_marker_obj() -> visualization_msgs::msg::Marker {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "odom";        // or "odom", "base_link"
        // marker.header.stamp = this->now();
        marker.ns = "graph_slam_ns";
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.orientation.w = 1.0;       

        // Scale (size of the marker in meters)
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;

        // Color (RGBA, must set alpha > 0)
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;

        marker.lifetime = rclcpp::Duration(0,0);  // 0 = forever
        
        return marker;
    }

    void GraphVisualiser::publishNodeMarker(const graph_slam_ns::Pose2D& node){
        auto marker_obj = GraphVisualiser::create_marker_obj();

        marker_obj.id = node.node_id; 

        marker_obj.pose.position.x = node.x;
        marker_obj.pose.position.y = node.y;
        marker_obj.pose.position.z = 0.0;

        m_node_marker_pub.publish(marker_obj);
    }


}