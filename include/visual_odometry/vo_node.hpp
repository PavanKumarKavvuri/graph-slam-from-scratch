# pragma once

#include "iostream"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include <cv_bridge/cv_bridge.h>


namespace visual_odometry_ns {

    class VisualOdom : public rclcpp::Node {

        public:
            VisualOdom(const rclcpp::NodeOptions & options);

            ~VisualOdom();
        
        private:
            rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_vis_odom_pub;
            message_filters::Subscriber<sensor_msgs::msg::Image> m_cam_0_sub;
            message_filters::Subscriber<sensor_msgs::msg::Image> m_cam_1_sub;

            const std::string left_image_path = "/home/pavan/Music/learning_ws/slam_ws/experimental_data/vo/tsukuba_l.png";
            const std::string right_image_path = "/home/pavan/Music/learning_ws/slam_ws/experimental_data/vo/tsukuba_r.png";

            int nfeatures = 2000;
            cv::Ptr<cv::ORB> orb;

            std::vector<cv::KeyPoint> kp_left, kp_right;
            cv::Mat desc_left, desc_right;
            std::vector<cv::DMatch> matches;
            cv::Mat left_image_mat, right_image_mat;
            std::vector<cv::Point3f> points3D;

            double fx = 718.856;
            double fy = 718.856;
            double cx = 607.1928;
            double cy = 185.2157;
            double baseline = 0.537; // meters

            uint32_t queue_size = 10;

            using SyncPolicy = message_filters::sync_policies::ApproximateTime<
                sensor_msgs::msg::Image, 
                sensor_msgs::msg::Image>;
            using Synchronizer = message_filters::Synchronizer<SyncPolicy>;
            std::shared_ptr<Synchronizer> m_sync;

            void initialise_pub_subs();

            void load_images();
            void compute_descriptors();
            void match_descriptors();
            void drawMatches();
            void reconstruct_3d_points();

            void syncCallback(const sensor_msgs::msg::Image::ConstSharedPtr& cam_0_msg,
                     const sensor_msgs::msg::Image::ConstSharedPtr& cam_1_msg);

            void processImages(const cv::Mat& left_mat, const cv::Mat& right_mat);


    };

}