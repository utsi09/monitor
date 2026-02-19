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

struct SensorFovSpec {
    std::string name;       // 센서 이름 (출력용)
    std::string frame_id;
    std::string ns;
    double h_fov_deg;
    double range;
    float r, g, b, a;       // 기본 색상
};

class UpdateSystemState : public SyncActionNode {
public:
    UpdateSystemState(const std::string& name, const NodeConfig& config,
                      const BT::RosNodeParams& params);

    static PortsList providedPorts();

    NodeStatus tick() override;

private:
    visualization_msgs::msg::Marker createFovMarker(
        const SensorFovSpec& spec, int id, const rclcpp::Time& stamp,
        bool is_important);

    bool isPathInFov(const std::vector<geometry_msgs::msg::Point>& path_points,
                     const SensorFovSpec& spec);

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr fov_pub_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    double prev_heading_ = 0.0;
    double filtered_vel_ = 0.0;
    rclcpp::Time prev_time_;
    bool first_tick_ = true;

    std::vector<SensorFovSpec> sensors_;
};
