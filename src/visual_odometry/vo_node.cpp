#include "visual_odometry/vo_node.hpp"

namespace visual_odometry_ns {

    VisualOdom::VisualOdom(const rclcpp::NodeOptions & options) : rclcpp::Node("visual_odom_node", options) {
        RCLCPP_INFO(this->get_logger(), "Visual Odometry Node started");

        VisualOdom::initialise_pub_subs();
        VisualOdom::load_images();

        orb = cv::ORB::create(nfeatures);

        VisualOdom::compute_descriptors();
        VisualOdom::match_descriptors();

        VisualOdom::drawMatches();

    }

    void VisualOdom::initialise_pub_subs() {
        rclcpp::QoS qos_settings_ = rclcpp::QoS(1).reliable();

        m_vis_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(
            "/visual_odom", qos_settings_);

    }

    void VisualOdom::load_images(){
        left_image_mat = cv::imread(left_image_path, cv::IMREAD_GRAYSCALE);
        right_image_mat = cv::imread(right_image_path, cv::IMREAD_GRAYSCALE);

        if (left_image_mat.empty() || right_image_mat.empty()){
            RCLCPP_ERROR(this->get_logger(), "Error Loading Images");
        }

        std::cout<< "Rows : " << left_image_mat.rows <<
                    "Columns : " << left_image_mat.cols <<
                    "Channels : " << left_image_mat.channels() << std::endl;
    }

    void VisualOdom::compute_descriptors(){
        orb->detectAndCompute(left_image_mat, cv::noArray(), kp_left, desc_left);
        orb->detectAndCompute(right_image_mat, cv::noArray(), kp_right, desc_right);

        RCLCPP_INFO(this->get_logger(), "Computed Key Features");
    }

    void VisualOdom::match_descriptors(){
        cv::BFMatcher matcher(cv::NORM_HAMMING, true); // crossCheck=true
        matcher.match(desc_left, desc_right, matches);

        std::sort(matches.begin(), matches.end(), [](const cv::DMatch &a, const cv::DMatch &b) {
            return a.distance < b.distance;
        });

        RCLCPP_INFO(this->get_logger(), "Descriptors Matched");
    }

    void VisualOdom::drawMatches(){
        cv::Mat debug;
        int num_draw = std::min(50, static_cast<int>(matches.size()));
        cv::drawMatches(left_image_mat, kp_left, right_image_mat, kp_right, std::vector<cv::DMatch>(matches.begin(), matches.begin() + num_draw), debug);
        
        cv::imwrite("stereo_orb_matches.png", debug);
        std::cout << "Saved debug matches to stereo_orb_matches.png" << std::endl;
    }

    VisualOdom::~VisualOdom() {
        RCLCPP_INFO(this->get_logger(), "Visal Odometry node destroyed.");
    }
}