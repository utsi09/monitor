#pragma once
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <cmath>
#include <vector>
#include <string>
using namespace BT;

"""
블랙보드에서 가장 가까운 객체까지의 거리 예를 들어 5m 받으면
현재 블랙보드에서 속도 꺼내서 속도 기반 5m까지 충돌값 ttc를 구함 (시간값 ms)
100~ttc~850 클립 진행 -> 중요센서 ftti 블랙보드 등록
비중요센서의 경우 고정 ftti 850 등록

추가 로직 ftti 가 중요센서로 인해 500이더라도 
중요센서 + 비중요 센서 고장일 경우 0.85 ftti와 0.5 ftti를 중복 적용해 빨리 오르도록

"""

class CalculateFtti : public SyncActionNode {
public:
    CalculateFtti(const std::string& name, const NodeConfig& config,
                      const BT::RosNodeParams& params);

    static PortsList providedPorts();

    NodeStatus tick() override;

private:
    // std::shared_ptr<rclcpp::Node> node_;
    // rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_pub_;
    // rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr fov_pub_;

    // std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    // std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // double prev_heading_ = 0.0;
    // double filtered_vel_ = 0.0;
    // rclcpp::Time prev_time_;
    // bool first_tick_ = true;

    // std::vector<SensorFovSpec> sensors_;
};
