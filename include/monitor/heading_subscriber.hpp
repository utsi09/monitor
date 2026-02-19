#pragma once
#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <cmath>

class HeadingSubscriber : public BT::RosTopicSubNode<geometry_msgs::msg::QuaternionStamped>
{
public:
    HeadingSubscriber(const std::string& name, const BT::NodeConfig& conf,
                      const BT::RosNodeParams& params)
        : RosTopicSubNode(name, conf, params) {}

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({
            BT::OutputPort<double>("current_heading", "현재 방향 (yaw)")
        });
    }

    BT::NodeStatus onTick(const std::shared_ptr<geometry_msgs::msg::QuaternionStamped>& last_msg) override
    {
        if (last_msg) {
            double q_x = last_msg->quaternion.x;
            double q_y = last_msg->quaternion.y;
            double q_z = last_msg->quaternion.z;
            double q_w = last_msg->quaternion.w;

            double yaw = std::atan2(2.0 * (q_w * q_z + q_x * q_y),
                                    1.0 - 2.0 * (q_y * q_y + q_z * q_z));

            setOutput("current_heading", yaw);
            //std::cout << yaw << std::endl;
        }
        return BT::NodeStatus::SUCCESS;
    }
};