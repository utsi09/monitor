#include "monitor/update_system_state.hpp"
#include <tf2/utils.h>
#include <iostream>
using namespace std;

CalculateFtti::UpdateSystemState(const string& name, const NodeConfig& config,
                                        const BT::RosNodeParams& params)
    : SyncActionNode(name,config)
{}

PortsList CalculateFtti::providedPorts() {
    return {
        InputPort<string>("important_sensors"),
        OutputPort<double>("ftti_p"),
        OutputPort<double>("ftti_n"),   
    };

NodeStatus UpdateSystemState::tick() {
    
}

}