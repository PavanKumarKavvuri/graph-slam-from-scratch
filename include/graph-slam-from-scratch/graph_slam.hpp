#include "iostream"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace graph_slam_ns{

    class GraphSLAM : public rclcpp::Node{

        public:
            GraphSLAM(const rclcpp::NodeOptions & options);

        void initialise_pub_subs();

        void gtPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
        
            ~GraphSLAM();

        private:
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_sub;
            rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_lls_sub;

    };
}