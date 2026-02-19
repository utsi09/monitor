#pragma once
#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cmath>
using namespace std;

class LidarChecker : public BT::SyncActionNode
{
public:
    LidarChecker(const std::string& name, const BT::NodeConfig& conf,
                    const BT::RosNodeParams& params)
                    : SyncActionNode(name, conf)
    {
        auto node = params.nh.lock();
        rclcpp::QoS qos(10);
        qos.best_effort();
        sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
            "lidar_points", qos,
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr){
                last_time_ = node_->now();
            }
        );
        node_ = node;
    }
    static BT::PortsList providedPorts()
    {
        return {
            BT::OutputPort<double>("lidar_timeout"),
        };
    }

    BT::NodeStatus tick() override
    {
        double timeout = -1.0;
        if (last_time_.nanoseconds() != 0) {
            timeout = (node_->now() - last_time_).seconds();

        } 
        setOutput("lidar_timeout", timeout);
        cout <<"타임아웃 : "<< timeout*1000 <<" ms"<< endl;
        return BT::NodeStatus::SUCCESS;
    }
private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Time last_time_;
};