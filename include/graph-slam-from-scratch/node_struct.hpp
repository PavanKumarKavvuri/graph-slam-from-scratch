#pragma once


namespace graph_slam_ns{

    struct Node2D {
        public:
            double x{};
            double y{};
            double theta{};

            int node_id{};

            double gt_x{};
            double gt_y{};
            double gt_theta{};

            
            
    };
}