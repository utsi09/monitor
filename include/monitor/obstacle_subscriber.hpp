#pragma once
#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include <hesai_zed_fusion/msg/obstacle_array.hpp>
#include <cmath>
#include <iostream>
using namespace std;

class ObastacleSubscriber : public BT::RosTopicSubNode<hesai_zed_fusion::msg::ObstacleArray>
{
public:
    ObastacleSubscriber(const std::string& name, const BT::NodeConfig& conf,
                    const BT::RosNodeParams& params)
                    : RosTopicSubNode(name, conf, params) {}
    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({
            BT::OutputPort<double>("nearest_obstacle"),
        });
    }

    BT::NodeStatus onTick(const std::shared_ptr<hesai_zed_fusion::msg::ObstacleArray>& last_msg) override
    {
        if (last_msg) {
            hesai_zed_fusion::msg::ObstacleArray nearest_obstacle;
            double min_dist = numeric_limits<double>::max();

            auto obstacles = last_msg->obstacles;

            for (const auto &ob : obstacles) {
                if (obs.distance < min_dist) {
                    min_dist = obs.distance;
                    closest_obs = obs;
                }
            }

            setOutput("nearest_obstacle", min_dist);


        }
        return BT::NodeStatus::SUCCESS;
    }

    // float create_uclidean(float rel_x, float rel_y, float rel_z){
    //     float distance = 0;
    //     distance = sqrt(rel_x**2 + rel_y**2 + rel_z**2);
    //     return distance;
    // }

};