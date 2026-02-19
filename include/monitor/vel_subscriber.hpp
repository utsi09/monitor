#pragma once
#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <cmath>

class VelSubscriber : public BT::RosTopicSubNode<geometry_msgs::msg::TwistStamped>
{
public:
    VelSubscriber(const std::string& name, const BT::NodeConfig& conf,
                    const BT::RosNodeParams& params)
                    : RosTopicSubNode(name, conf, params) {}
    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({
            BT::OutputPort<double>("vel_x", "GPS 프레임 x 속도"),
            BT::OutputPort<double>("vel_y", "GPS 프레임 y 속도"),
        });
    }

    BT::NodeStatus onTick(const std::shared_ptr<geometry_msgs::msg::TwistStamped>& last_msg) override
    {
        if (last_msg) {
            setOutput("vel_x", last_msg->twist.linear.x);
            setOutput("vel_y", last_msg->twist.linear.y);
        }
        return BT::NodeStatus::SUCCESS;
    }
};