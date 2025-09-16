#pragma once

#include "Eigen/Dense"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "cmath"
#include "graph-slam-from-scratch/node_struct.hpp"
#include "graph-slam-from-scratch/loop_closure_struct.hpp"



namespace graph_slam_ns{

    inline auto relativePose2D(const Node2D& from, const Node2D& to) -> Eigen::Vector3d {

        double dx = to.x - from.x;
        double dy = to.y - from.y;
        double dtheta = to.theta - from.theta;

        while (dtheta > M_PI)  dtheta -= 2*M_PI;
        while (dtheta < -M_PI) dtheta += 2*M_PI;

        double cos_t = cos(-from.theta);
        double sin_t = sin(-from.theta);

        double local_dx = cos_t * dx - sin_t * dy;
        double local_dy = sin_t * dx + cos_t * dy;

        return Eigen::Vector3d(local_dx, local_dy, dtheta);

    }

    inline auto computeInfoMatrix(const std::array<double, 36>& c) -> Eigen::Matrix3d {
        Eigen::Matrix3d cov;
        cov <<  c[0],  c[1],  c[5],
                c[6],  c[7],  c[11],
                c[30], c[31], c[35];
        if (cov.determinant() <= 1e-9)
            return Eigen::Matrix3d::Identity() * 1e-3;

        return cov.inverse();
    }

    inline double computeDistance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2){
        return std::sqrt(
            std::pow(p2.x - p1.x, 2) +
            std::pow(p2.y - p1.y, 2)
        );
    }

    inline double extractTheta(const geometry_msgs::msg::Quaternion& q){
        Eigen::Quaterniond quaternion(q.w, q.x, q.y, q.z);
        Eigen::Vector3d euler_ang = quaternion.toRotationMatrix().eulerAngles(2, 1, 0); // Z, Y, X

        return euler_ang(0);
    }

    inline LoopClosureResult checkLoopClosure(const graph_slam_ns::Node2D& current_node, const std::vector<graph_slam_ns::Node2D> node_vec){
        double loop_threshold = 0.5;    // meters
        int min_separation = 20;        // don’t connect to immediate neighbors

        LoopClosureResult loop_closed_result{};

        for (const auto& candidate_node : node_vec){

            if ((current_node.node_id - candidate_node.node_id) < min_separation) {
                // Skip and Continue to next iteration
                continue;
            }

            double dx = current_node.gt_x - candidate_node.gt_x;
            double dy = current_node.gt_y - candidate_node.gt_y;

            double dist = std::sqrt(dx*dx + dy*dy);
            if (dist < loop_threshold) {
                std::cout << "Loop closure Detected at Nodes " << candidate_node.node_id << " and " << current_node.node_id << " with dist : " << dist << std::endl;
                
                loop_closed_result.result = true;
                loop_closed_result.closing_node = candidate_node;

                return loop_closed_result;
            }

        }
        return loop_closed_result;
    }

}