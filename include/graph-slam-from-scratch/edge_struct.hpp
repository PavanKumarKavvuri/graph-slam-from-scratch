#pragma once

#include "Eigen/Dense"
#include "string"

namespace graph_slam_ns {

    struct Edge2D {

        public:
            int from_node_id {};
            int to_node_id {};
            Eigen::Vector3d relative_meas {};
            Eigen::Matrix3d information_mat {};

            std::string edge_type {"odom"};

            Edge2D(int& from, int& to,
                   Eigen::Vector3d& meas,
                   Eigen::Matrix3d& inf
                     ): from_node_id(from), to_node_id(to), relative_meas(meas), information_mat(inf){
                    
                   }

    };

}

