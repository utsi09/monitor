#include "monitor/update_system_state.hpp"
using namespace std;

UpdateSystemState::UpdateSystemState(const string& name, const NodeConfig& config)
    : SyncActionNode(name,config)
{}

PortsList UpdateSystemState::providedPorts() {
    return {
        InputPort<string>("topic_vel"),
        InputPort<string>("topic_heading"),

        OutputPort<double>("current_vel"),
        OutputPort<double>("current_heading"),
        OutputPort<string>("current_w")
    };
}

NodeStatus UpdateSystemState::tick() {
    string topic_vel_name;
    getInput("topic_vel", topic_vel_name);

    double temp_vel = 12.5;
    double temp_heading = 0.123;
    
    for(int i=0; i<10000;i++){
        temp_vel++;
        temp_heading++;
        cout << temp_vel << endl;
        setOutput("current_vel", temp_vel);
        setOutput("current_heading", temp_heading);
        sleep(1);
    }
    setOutput("current_vel", temp_vel);
    setOutput("current_heading", temp_heading);

    return NodeStatus::SUCCESS;

}
