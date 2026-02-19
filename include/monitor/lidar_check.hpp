#pragma once
#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cmath>
using namespace std;

class LidarChecker : public BT::RosTopicSubNode<sensor_msgs::msg::PointCloud2>
{
public:
    LidarChecker(const std::string& name, const BT::NodeConfig& conf,
                    const BT::RosNodeParams& params)
                    : RosTopicSubNode(name, conf, params) {}
    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({
            BT::OutputPort<double>("lidar_timeout", "라이다 타임아웃 시간"),
        });
    }

    BT::NodeStatus onTick(const std::shared_ptr<sensor_msgs::msg::PointCloud2>& last_msg) override
    {
        rclcpp::Time now = node_.lock()->now();
        double timeout_duration = 0.0;

        if (last_msg) {
            last_time_ = rclcpp::Time(last_msg->header.stamp);
        }
        if (last_time_.nanoseconds() == 0) {
            timeout_duration = -1.0;
        } else {
            timeout_duration = (now - last_time_).seconds();
        }
        setOutput("lidar_timeout", timeout_duration);
        cout << timeout_duration << endl;
        return BT::NodeStatus::SUCCESS;
    }
private:
    rclcpp::Time last_time_;
};