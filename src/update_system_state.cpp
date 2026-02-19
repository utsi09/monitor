#include "monitor/update_system_state.hpp"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <iostream>
using namespace std;

UpdateSystemState::UpdateSystemState(const string& name, const NodeConfig& config,
                                        const BT::RosNodeParams& params)
    : SyncActionNode(name,config)
{
    node_= params.nh.lock();
    path_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/kinematic_path", 10);
    fov_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/sensor_fov", 10);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // {name, frame, ns, h_fov(deg), range(m), r, g, b, a}
    sensors_ = {
        {"Hesai_AT128",     "hesai_lidar",       "hesai",        120.0, 200.0,  0.0f, 1.0f, 0.0f, 0.7f},
        {"Valeo_SCALA_L",   "front_lidar_left",  "valeo_lidar",  133.0, 150.0,  0.0f, 1.0f, 0.0f, 0.7f},
        {"Valeo_SCALA_R",   "front_lidar_right", "valeo_lidar",  133.0, 150.0,  0.0f, 1.0f, 0.0f, 0.7f},
        {"ZED_X_Front",     "zedx_base_link",    "zedx",         110.0,  20.0,  0.0f, 0.9f, 0.6f, 0.7f},
        {"ZED_Back",        "zed_back",          "zed_back",     110.0,  20.0,  0.0f, 0.9f, 0.6f, 0.7f},
        {"OAK-D_Left",      "oakd_left",         "oakd",          81.0,  12.0,  0.0f, 0.9f, 0.6f, 0.7f},
        {"OAK-D_Right",     "oakd_right",        "oakd",          81.0,  12.0,  0.0f, 0.9f, 0.6f, 0.7f},
        {"Radar_FL",        "valeo_radar_fl",    "radar",        140.0, 200.0,  0.5f, 1.0f, 0.0f, 0.65f},
        {"Radar_FR",        "valeo_radar_fr",    "radar",        140.0, 200.0,  0.5f, 1.0f, 0.0f, 0.65f},
        {"Radar_RL",        "valeo_radar_rl",    "radar",        140.0, 200.0,  0.5f, 1.0f, 0.0f, 0.65f},
        {"Radar_RR",        "valeo_radar_rr",    "radar",        140.0, 200.0,  0.5f, 1.0f, 0.0f, 0.65f},
    };
}

PortsList UpdateSystemState::providedPorts() {
    return {
        InputPort<double>("vel_x"),
        InputPort<double>("vel_y"),
        InputPort<double>("current_heading"),
        OutputPort<string>("important_sensors"),
    };
}

bool UpdateSystemState::isPathInFov(
    const vector<geometry_msgs::msg::Point>& path_points,
    const SensorFovSpec& spec)
{
    geometry_msgs::msg::TransformStamped tf;
    try {
        tf = tf_buffer_->lookupTransform(
            "base_link", spec.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException&) {
        return false;
    }

    // 센서 위치와 yaw (base_link 기준)
    double sx = tf.transform.translation.x;
    double sy = tf.transform.translation.y;
    double sensor_yaw = tf2::getYaw(tf.transform.rotation);

    double half_fov = spec.h_fov_deg * M_PI / 180.0 / 2.0;

    for (const auto& pt : path_points) {
        double dx = pt.x - sx;
        double dy = pt.y - sy;
        double dist = sqrt(dx * dx + dy * dy);

        if (dist > spec.range) continue;

        double angle = atan2(dy, dx) - sensor_yaw;
        // 정규화
        while (angle > M_PI)  angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;

        if (fabs(angle) <= half_fov) {
            return true;
        }
    }
    return false;
}

visualization_msgs::msg::Marker UpdateSystemState::createFovMarker(
    const SensorFovSpec& spec, int id, const rclcpp::Time& stamp,
    bool is_important)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = spec.frame_id;
    marker.header.stamp = stamp;
    marker.ns = spec.ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    if (is_important) {
        marker.scale.x = 0.03;
        // 쨍한 주황색
        marker.color.r = 1.0f;
        marker.color.g = 0.5f;
        marker.color.b = 0.0f;
        marker.color.a = 0.9f;
    } else {
        marker.scale.x = 0.01;
        marker.color.r = spec.r;
        marker.color.g = spec.g;
        marker.color.b = spec.b;
        marker.color.a = spec.a;
    }

    double h_fov_rad = spec.h_fov_deg * M_PI / 180.0;
    double start_angle = -h_fov_rad / 2.0;
    double end_angle   =  h_fov_rad / 2.0;
    int segments = 40;
    double angle_step = (end_angle - start_angle) / segments;

    geometry_msgs::msg::Point origin;
    origin.x = 0.0; origin.y = 0.0; origin.z = 0.0;
    marker.points.push_back(origin);

    for (int i = 0; i <= segments; i++) {
        double a = start_angle + i * angle_step;
        geometry_msgs::msg::Point p;
        p.x = spec.range * cos(a);
        p.y = spec.range * sin(a);
        p.z = 0.0;
        marker.points.push_back(p);
    }

    marker.points.push_back(origin);

    return marker;
}

NodeStatus UpdateSystemState::tick() {
    double vel_x = 0.0, vel_y = 0.0;
    double current_heading = 0.0;
    getInput("vel_x", vel_x);
    getInput("vel_y", vel_y);
    getInput("current_heading", current_heading);

    double current_vel = std::hypot(vel_x, vel_y);

    rclcpp::Time now = node_->now();
    double yaw_rate = 0.0;

    if (first_tick_) {
        first_tick_ = false;
    } else {
        double dt_real = (now - prev_time_).seconds();
        if (dt_real > 0.001) {
            double dh = current_heading - prev_heading_;
            while (dh > M_PI)  dh -= 2.0 * M_PI;
            while (dh < -M_PI) dh += 2.0 * M_PI;
            yaw_rate = -dh / dt_real;
        }
    }
    prev_heading_ = current_heading;
    prev_time_ = now;

    // 예측 경로 생성
    double x = 0.0, y = 0.0;
    double heading = 0.0;
    double dt = 0.1;
    int steps = 15;

    vector<geometry_msgs::msg::Point> path_points;
    visualization_msgs::msg::MarkerArray path_array;
    visualization_msgs::msg::Marker line_strip;
    line_strip.header.frame_id = "base_link";
    line_strip.header.stamp = now;
    line_strip.ns = "prediction";
    line_strip.id = 0;
    line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_strip.action = visualization_msgs::msg::Marker::ADD;

    line_strip.scale.x = 0.15;
    line_strip.color.r = 0.0; line_strip.color.g = 0.0; line_strip.color.b = 1.0;
    line_strip.color.a = 0.8;

    geometry_msgs::msg::Point start;
    start.x = 0.0; start.y = 0.0; start.z = 0.1;
    line_strip.points.push_back(start);
    path_points.push_back(start);

    for (int i = 0; i < steps; i++) {
        heading += yaw_rate * dt;
        x += current_vel * cos(heading) * dt;
        y += current_vel * sin(heading) * dt;

        geometry_msgs::msg::Point p;
        p.x = x;
        p.y = y;
        p.z = 0.1;
        line_strip.points.push_back(p);
        path_points.push_back(p);
    }

    path_array.markers.push_back(line_strip);
    path_pub_->publish(path_array);

    // FOV 겹침 체크 & 마커 발행
    visualization_msgs::msg::MarkerArray fov_array;
    vector<string> important_list;

    for (size_t i = 0; i < sensors_.size(); i++) {
        bool important = isPathInFov(path_points, sensors_[i]);
        if (important) {
            important_list.push_back(sensors_[i].name);
        }
        fov_array.markers.push_back(
            createFovMarker(sensors_[i], static_cast<int>(i), now, important));
    }
    fov_pub_->publish(fov_array);

    // important_sensors 문자열 생성
    string important_str;
    for (size_t i = 0; i < important_list.size(); i++) {
        if (i > 0) important_str += ",";
        important_str += important_list[i];
    }

    // 블랙보드에 발행
    setOutput("important_sensors", important_str);

    // 디버깅 출력
    cout << "[Monitor] Important Sensors: "
         << (important_str.empty() ? "None" : important_str) << endl;

    return NodeStatus::SUCCESS;
}
