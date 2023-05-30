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
#include "glassy_interfaces/srv/arm.hpp"
#include "glassy_interfaces/msg/manualactuatorsignals.hpp"
#include "glassy_interfaces/msg/offboarddirectcontrol.hpp"
#include "glassy_interfaces/msg/offboardattituderate.hpp"
#include <glassy_interfaces/msg/state.hpp>


class MavsdkNode;
class RosNode
{
private:
    // service and subscriber callbacks
    void arm_disarm(const std::shared_ptr<glassy_interfaces::srv::Arm::Request> request, std::shared_ptr<glassy_interfaces::srv::Arm::Response> response);

public:
    RosNode(std::shared_ptr<rclcpp::Node> node);
    ~RosNode(){};
    void init();

    std::shared_ptr<MavsdkNode> mav_node;
    std::shared_ptr<rclcpp::Node> ros_node;

    //services
    rclcpp::Service<glassy_interfaces::srv::Arm>::SharedPtr arm_disarm_service;

    // subscribers
    rclcpp::Subscription<glassy_interfaces::msg::Manualactuatorsignals>::SharedPtr manual_actuator_subscriber;
    rclcpp::Subscription<glassy_interfaces::msg::Offboarddirectcontrol>::SharedPtr offboard_direct_subscriber;
    rclcpp::Subscription<glassy_interfaces::msg::Offboardattituderate>::SharedPtr offboard_attitude_rate_subscriber;

    void manual_actuator_control_callback(const glassy_interfaces::msg::Manualactuatorsignals::SharedPtr msg);
    void offboard_direct_control_callback(const glassy_interfaces::msg::Offboarddirectcontrol::SharedPtr msg);
    void offboard_attitude_rate_control_callback(const glassy_interfaces::msg::Offboardattituderate::SharedPtr msg);

    // publishers
    rclcpp::Publisher<glassy_interfaces::msg::State>::SharedPtr state_publisher;

    // parameters
    bool state_publishing;
    bool direct_offboard_subscription;
    bool attitude_rate_offboard_subscription;
    bool manual_actuators_subscription;


};

#endif