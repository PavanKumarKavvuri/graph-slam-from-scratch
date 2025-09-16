# pragma once

#include "iostream"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "opencv2/opencv.hpp"


namespace visual_odometry_ns {

    class VisualOdom : public rclcpp::Node {

        public:
            VisualOdom(const rclcpp::NodeOptions & options);

            ~VisualOdom();
        
        private:
            rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_vis_odom_pub;

            const std::string left_image_path = "/home/pavan/Music/learning_ws/slam_ws/experimental_data/vo/tsukuba_l.png";
            const std::string right_image_path = "/home/pavan/Music/learning_ws/slam_ws/experimental_data/vo/tsukuba_r.png";

            int nfeatures = 2000;
            cv::Ptr<cv::ORB> orb;

            std::vector<cv::KeyPoint> kp_left, kp_right;
            cv::Mat desc_left, desc_right;
            std::vector<cv::DMatch> matches;

            cv::Mat left_image_mat, right_image_mat;
            

            void initialise_pub_subs();

            void load_images();
            void compute_descriptors();
            void match_descriptors();
            void drawMatches();


    };

}