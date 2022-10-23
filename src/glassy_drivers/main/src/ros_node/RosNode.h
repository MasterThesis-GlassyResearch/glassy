#ifndef _RosNode_
#define _RosNode_

#include <MavsdkNode.h>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <future>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "vehicle_interfaces/srv/arm.hpp"
#include "vehicle_interfaces/msg/actuatorsignals.hpp"
#include <vehicle_interfaces/msg/state.hpp>


class MavsdkNode;
class RosNode
{
private:
    // service and subscriber callbacks
    void arm_disarm(const std::shared_ptr<vehicle_interfaces::srv::Arm::Request> request, std::shared_ptr<vehicle_interfaces::srv::Arm::Response> response);

public:
    RosNode(std::shared_ptr<rclcpp::Node> node);
    ~RosNode(){};
    void init();

    std::shared_ptr<MavsdkNode> mav_node;
    std::shared_ptr<rclcpp::Node> ros_node;

    //services
    rclcpp::Service<vehicle_interfaces::srv::Arm>::SharedPtr arm_disarm_service;

    // subscribers
    rclcpp::Subscription<vehicle_interfaces::msg::Actuatorsignals>::SharedPtr actuator_subscriber;
    void actuator_control_callback(const vehicle_interfaces::msg::Actuatorsignals::SharedPtr msg);

    // publishers
    rclcpp::Publisher<vehicle_interfaces::msg::State>::SharedPtr state_publisher;

    // parameters
    bool state_subscription;


};

#endif