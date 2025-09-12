#include "rclcpp/rclcpp.hpp"
#include "graph-slam-from-scratch/graph_slam.hpp"


auto main(int argc, char **argv) -> int {

    std::cout<< "Starting the Graph Slam Project" << std::endl;

    rclcpp::init(argc, argv);

    auto node = std::make_shared<graph_slam_ns::GraphSLAM>(rclcpp::NodeOptions());

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}