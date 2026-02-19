#pragma once
#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <cmath>
using namespace std;

class ImuChecker : public BT::RosTopicSubNode<sensor_msgs::msg::Imu>
{
public:
    ImuChecker(const std::string& name, const BT::NodeConfig& conf,
                    const BT::RosNodeParams& params)
                    : RosTopicSubNode(name, conf, params) {}
    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({
            BT::OutputPort<double>("imu_timeout"),
        });
    }

    BT::NodeStatus onTick(const std::shared_ptr<sensor_msgs::msg::Imu>& last_msg) override
    {
        rclcpp::Time now = node_.lock()->now();
        double timeout = 0.0;

        if (last_msg && last_msg != prev_msg_) {
            last_time_ = now;
            prev_msg_ = last_msg;
        }
        if (last_time_.nanoseconds() == 0) {
            timeout = -1.0;
            //return BT::NodeStatus::FAILURE;
        } else {
            timeout = (now - last_time_).seconds();
        }
        setOutput("imu_timeout", timeout);
        cout <<"imu 타임아웃 :" << timeout*1000000<<" ms" << endl;
        return BT::NodeStatus::SUCCESS;
    }
private:
    rclcpp::Time last_time_;
    std::shared_ptr<sensor_msgs::msg::Imu> prev_msg_;
};