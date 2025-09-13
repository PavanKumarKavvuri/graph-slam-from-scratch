#include "iostream"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "vector"
#include "cmath"
#include <Eigen/Dense>

namespace graph_slam_ns{

    class GraphSLAM : public rclcpp::Node{

        public:
            struct Pose2D {
                public:
                    double x{};
                    double y{};
                    double theta{};

                    int node_id{};

            };
            GraphSLAM(const rclcpp::NodeOptions & options);
            
            ~GraphSLAM();

        private:
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_sub;
            rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_lls_sub;

            
            std::vector<Pose2D> PoseArray{};
            geometry_msgs::msg::Point last_point_{};

            int nodeCounter{0};
            bool is_firstFrame = true;
            double euc_dist{};
            double distance_travelled{};

            void initialise_pub_subs();

            void gtPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
            void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

            double computeDistance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2);
            double extractTheta(const geometry_msgs::msg::Quaternion& q);
            // std::ostream& operator << (std::ostream& os, const std::vector<Pose2D>& vec) {
            //     os << "[";
            //     for(auto& v : vec) {
            //         os << "{ ";
            //         os << v.x << ", " << v.y << ", " << v.theta;
            //         os << "}, ";
            //     }
            //     os << "]";

            //     return os;
            //  }

    };

    inline std::ostream& operator << (std::ostream& os, const GraphSLAM::Pose2D& pose) {
        os << "(";
        os << pose.x << ", " << pose.y << ", " << pose.theta;
        os << ")";
        os << " Node id : " << pose.node_id;

        return os;
    }

            
}