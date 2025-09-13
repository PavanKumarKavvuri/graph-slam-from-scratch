#pragma once

#include "graph-slam-from-scratch/graph_slam.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"

namespace graph_slam_ns{

    struct Pose2D; // Forward declaration

    class GraphVisualiser {

        public:
            GraphVisualiser(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub);
            ~GraphVisualiser();

            visualization_msgs::msg::Marker create_marker_obj();
            void publishNodeMarker(const graph_slam_ns::Pose2D& node);

            // void initialise_pub_subs();

        private:
            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_node_marker_pub;

    };
}