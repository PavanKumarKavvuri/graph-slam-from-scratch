#pragma once

#include "Eigen/Dense"
#include "nav_msgs/msg/odometry.hpp"
#include "cmath"
#include "graph-slam-from-scratch/node_struct.hpp"


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

}