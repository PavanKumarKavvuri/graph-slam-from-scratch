#include "visual_odometry/vo_node.hpp"

namespace visual_odometry_ns {

    VisualOdom::VisualOdom(const rclcpp::NodeOptions & options) : rclcpp::Node("visual_odom_node", options) {
        RCLCPP_INFO(this->get_logger(), "Visual Odometry Node started");

        VisualOdom::initialise_pub_subs();
        // VisualOdom::load_images();

        orb = cv::ORB::create(nfeatures);

        // VisualOdom::compute_descriptors();
        // VisualOdom::match_descriptors();

        // VisualOdom::drawMatches();

        // VisualOdom::reconstruct_3d_points();

    }

    void VisualOdom::initialise_pub_subs() {
        rclcpp::QoS qos_settings_ = rclcpp::QoS(1).reliable();

        m_vis_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(
            "/visual_odom", qos_settings_);
        
        m_cam_0_sub.subscribe(this, "/cam0/image_raw");
        m_cam_1_sub.subscribe(this, "/cam1/image_raw");
        
        // Create approximate time synchronizer
        // Parameters: queue_size (10 in this example)
        // You can adjust queue size based on your message rate and synchronization needs
        constexpr int queue_size = 10;
        m_sync = std::make_shared<Synchronizer>(SyncPolicy(queue_size), m_cam_0_sub, m_cam_1_sub);
        
        // Optional: Set maximum time difference for synchronization (default is 0.1 seconds)
        // m_sync->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.05));
        
        // Register callback for synchronized messages
        m_sync->registerCallback(std::bind(&VisualOdom::syncCallback, this, 
                                          std::placeholders::_1, std::placeholders::_2));
        
        RCLCPP_INFO(this->get_logger(), "Stereo camera synchronizer initialized");

    }

    void VisualOdom::syncCallback(const sensor_msgs::msg::Image::ConstSharedPtr& cam_0_msg,
                     const sensor_msgs::msg::Image::ConstSharedPtr& cam_1_msg)
    {
        left_image_mat  = cv_bridge::toCvCopy(cam_0_msg, "mono8")->image;
        right_image_mat = cv_bridge::toCvCopy(cam_1_msg, "mono8")->image;

        if (left_image_mat.empty() || right_image_mat.empty()){
            RCLCPP_ERROR(this->get_logger(), "Error Loading Images");
        }
        
        // Process your synchronized images here
        VisualOdom::processImage(left_image_mat, right_image_mat);
    }

    void VisualOdom::processImages(const cv::Mat& left_mat, const cv::Mat& right_mat) {

        std::vector<cv::KeyPoint> kp_left, kp_right;
        cv::Mat desc_left, desc_right;

        orb->detectAndCompute(left_image_mat, cv::noArray(), kp_left, desc_left);
        orb->detectAndCompute(right_image_mat, cv::noArray(), kp_right, desc_right);



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

    void VisualOdom::reconstruct_3d_points(){

        for (auto &m : matches) {
            cv::Point2f pl = kp_left[m.queryIdx].pt;
            cv::Point2f pr = kp_right[m.trainIdx].pt;

            double disparity = pl.x - pr.x;
            if (disparity <= 0) continue; 

            double z = fx * baseline / disparity;
            double x = (pl.x - cx) * z / fx;
            double y = (pl.y - cy) * z / fy;

            points3D.emplace_back(x, y, z);
        }

        for (size_t i = 0; i < std::min(points3D.size(), size_t(10)); i++) {
            std::cout << "3D point: " << points3D[i] << std::endl;
        }

    }

    VisualOdom::~VisualOdom() {
        RCLCPP_INFO(this->get_logger(), "Visal Odometry node destroyed.");
    }

    // void VisualOdom::load_images(const auto& img0, const auto& img1){
    //     left_image_mat  = cv_bridge::toCvCopy(cam_0_msg, "mono8")->image;
    //     right_image_mat = cv_bridge::toCvCopy(cam_1_msg, "mono8")->image;

    //     if (left_image_mat.empty() || right_image_mat.empty()){
    //         RCLCPP_ERROR(this->get_logger(), "Error Loading Images");
    //     }
    // }

}