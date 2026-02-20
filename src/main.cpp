#include <behaviortree_cpp/basic_types.h>           // BT::Blackboard
#include "behaviortree_cpp/bt_factory.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "std_msgs/msg/float64.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <csignal>
#include <string>

#include "monitor/update_system_state.hpp"
#include "behaviortree_ros2/bt_topic_sub_node.hpp"

// 토픽 구독 노드
#include "monitor/vel_subscriber.hpp"
#include "monitor/heading_subscriber.hpp"
#include "monitor/obstacle_subscriber.hpp"
#include "monitor/lidar_check.hpp"
#include "monitor/cam_check.hpp"
#include "monitor/imu_check.hpp"
#include "monitor/gps_check.hpp"
#include "monitor/calculate_ftti.hpp"

using namespace std::chrono_literals;
using namespace BT;

std::string tree_path =
    ament_index_cpp::get_package_share_directory("monitor") +
    "/behavior_tree/monitor.xml";

bool g_should_exit = false;
void signal_handler(int) { g_should_exit = true; }

int main(int argc, char** argv)
{
    rclcpp::init(argc,argv);
    auto shared_node = rclcpp::Node::make_shared("monitor_node");

    BehaviorTreeFactory factory;
    std::signal(SIGINT, signal_handler);

    //로스 관련
    BT::RosNodeParams params;
    params.nh = shared_node;

    //구독노드
    factory.registerNodeType<VelSubscriber>("VelSubscriber", params);
    factory.registerNodeType<HeadingSubscriber>("HeadingSubscriber", params);
    factory.registerNodeType<ObstacleSubscriber>("ObstacleSubscriber", params);

    //블랙보드 설정
    auto blackboard = Blackboard::create();

    blackboard->set("vel_x", 0.0);
    blackboard->set("vel_y", 0.0);
    blackboard->set("current_heading", 0.0);

    std::string current_w = "Lidar,Camera";
    blackboard->set("current_w", current_w);
    //

    //팩토리 설정
    factory.registerNodeType<UpdateSystemState>("UpdateSystemState", params);
    factory.registerNodeType<CalculateFTTI>("CalculateFTTI");

    factory.registerNodeType<LidarChecker>("LidarChecker",params);
    factory.registerNodeType<CamChecker>("CamChecker",params);
    factory.registerNodeType<ImuChecker>("ImuChecker",params);
    factory.registerNodeType<GpsChecker>("GpsChecker",params);

    
    //
    auto tree = factory.createTreeFromFile(tree_path, blackboard);
    BT::Groot2Publisher groot_publisher(tree, 1666); //groot 퍼블리셔

    rclcpp::Rate loop_rate(10);
    while(rclcpp::ok() && !g_should_exit) {
      rclcpp::spin_some(shared_node);
      tree.tickWhileRunning(10ms);
      loop_rate.sleep();

    }
    
    rclcpp::shutdown();

    return 0;
}