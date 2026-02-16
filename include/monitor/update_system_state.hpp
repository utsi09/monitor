#pragma once
#include "behaviortree_cpp/behavior_tree.h"
#include <rclcpp/rclcpp.hpp>
using namespace BT;

class UpdateSystemState : public SyncActionNode {
    public:
        UpdateSystemState(const std::string& name, const NodeConfig& config)

        static PortList providedPorts();
        NodeStatus tick() override;
    private:
        
}