#include "graph-slam-from-scratch/graph_slam.hpp"
#include "graph-slam-from-scratch/helpers.hpp"


namespace graph_slam_ns{

    GraphSLAM::GraphSLAM(const rclcpp::NodeOptions & options): rclcpp::Node("graph_slam_node", options){
        RCLCPP_INFO(this->get_logger(), "Custom Graph SLAM Node Initialised!");

        GraphSLAM::initialise_pub_subs();

        GraphSLAM::createEdgesMarker_Obj();

    }

    void GraphSLAM::initialise_pub_subs(){

        rclcpp::QoS qos_settings_ = rclcpp::QoS(1).reliable();

        m_lls_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/lls", qos_settings_,
            std::bind(&GraphSLAM::gtPoseCallback, this, std::placeholders::_1));

        m_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", qos_settings_,
            std::bind(&GraphSLAM::odomCallback, this, std::placeholders::_1));

        m_node_marker_pub = this->create_publisher<visualization_msgs::msg::Marker>(
            "visualization_marker", qos_settings_);
        
        m_edge_marker_pub = this->create_publisher<visualization_msgs::msg::Marker>(
            "edges_marker", qos_settings_);

    }

    void GraphSLAM::gtPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
        // RCLCPP_INFO_STREAM(this->get_logger(), "Ground Truth Pose Received: [" << msg->pose.position.x << ", " << msg->pose.position.y << "]");
        // frameCount += 1;
    }

    void GraphSLAM::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
        
        geometry_msgs::msg::Point current_point = msg->pose.pose.position;
        geometry_msgs::msg::Quaternion current_quat = msg->pose.pose.orientation;
        auto current_cov = msg->pose.covariance;

        if(is_firstFrame){
            RCLCPP_INFO(this->get_logger(), "First Frame Received");
            is_firstFrame = false;
            // nodeCounter += 1;

            auto theta = extractTheta(current_quat);

            Node2D node_2d{ .x = current_point.x,
                            .y = current_point.y,
                            .theta = theta,
                            .node_id = nodeCounter
                            };
            NodesArray.push_back(node_2d);

            previous_node_ = node_2d;

            // Now publish Node2D
            GraphSLAM::publishNodeMarker(node_2d);
        }
        else{
            auto dist = computeDistance(current_point, last_point_);
            distance_travelled += dist;
            
            if(distance_travelled > 10){
                // RCLCPP_INFO(this->get_logger(), "Distance greater than 15mts. Dropping node now");
                
                nodeCounter += 1;
                auto theta = extractTheta(current_quat);

                Node2D node_2d{ .x = current_point.x,
                                .y = current_point.y,
                                .theta = theta,
                                .node_id = nodeCounter
                                };
                NodesArray.push_back(node_2d);

                // Now publish Node2D
                GraphSLAM::publishNodeMarker(node_2d);

                std::cout<< node_2d << std::endl;

                Eigen::Vector3d rel_meas = graph_slam_ns::relativePose2D(previous_node_, node_2d);
                Eigen::Matrix3d inf_mat = graph_slam_ns::computeInfoMatrix(current_cov);
                
                // const auto type = "odom";
                Edge2D edge(node_2d.node_id, previous_node_.node_id, rel_meas, inf_mat);
                EdgesArray.push_back(edge);

                const std::string type = "odom";

                // Now publish Edge2D
                GraphSLAM::publishEdgeMarker(previous_node_, node_2d, type);
                
                last_point_ = current_point;
                previous_node_ = node_2d;

                distance_travelled = 0.0;
                

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

    auto GraphSLAM::create_marker_obj() -> visualization_msgs::msg::Marker {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "odom";        // or "odom", "base_link"
        marker.header.stamp = rclcpp::Clock().now();
        marker.ns = "nodes";
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

    void GraphSLAM::createEdgesMarker_Obj() {

        m_edge_marker.header.frame_id = "odom";
        m_edge_marker.header.stamp = rclcpp::Clock().now();
        m_edge_marker.ns = "edges";
        
        m_edge_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        m_edge_marker.action = visualization_msgs::msg::Marker::ADD;
        m_edge_marker.scale.x = 0.01; // line width
        m_edge_marker.scale.y = 0.01; // line width

        m_edge_marker.color.r = 1.0f;
        m_edge_marker.color.g = 1.0f;
        m_edge_marker.color.b = 0.0f;
        m_edge_marker.color.a = 1.0f;

        m_edge_marker.points.clear();
        // m_edge_marker.colors.clear();

    }

    void GraphSLAM::publishNodeMarker(const graph_slam_ns::Node2D& node){
        auto marker_obj = GraphSLAM::create_marker_obj();

        marker_obj.id = node.node_id; 

        marker_obj.pose.position.x = node.x;
        marker_obj.pose.position.y = node.y;
        marker_obj.pose.position.z = 0.0;

        m_node_marker_pub->publish(marker_obj);
    }

    void GraphSLAM::publishEdgeMarker(const graph_slam_ns::Node2D& prev, const graph_slam_ns::Node2D& current, const std::string& edge_type){

        this->createEdgesMarker_Obj();

        m_edge_marker.id = prev.node_id;

        geometry_msgs::msg::Point prev_position;
        prev_position.x = prev.x;
        prev_position.y = prev.y;

        geometry_msgs::msg::Point current_position;
        current_position.x = current.x;
        current_position.y = current.y;

        m_edge_marker.points.push_back(prev_position);
        m_edge_marker.points.push_back(current_position);

        m_edge_marker.lifetime = rclcpp::Duration(0,0);

        m_edge_marker_pub->publish(m_edge_marker);

    }


    GraphSLAM::~GraphSLAM(){
        RCLCPP_INFO(this->get_logger(), "Custom Graph SLAM Node Destroyed!");
    }

}