#include "rclcpp/rclcpp.hpp"
#include "visual_odometry/vo_node.hpp"


auto main(int argc, char **argv) -> int {

    std::cout<< "Starting the Visual Odometry Project" << std::endl;

    rclcpp::init(argc, argv);

    auto node = std::make_shared<visual_odometry_ns::VisualOdom>(rclcpp::NodeOptions());

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}