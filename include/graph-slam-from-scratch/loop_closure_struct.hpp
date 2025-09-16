#pragma once

#include "graph-slam-from-scratch/node_struct.hpp"

namespace graph_slam_ns{

    struct LoopClosureResult {
        bool result{false};
        Node2D closing_node{};
    };

}
