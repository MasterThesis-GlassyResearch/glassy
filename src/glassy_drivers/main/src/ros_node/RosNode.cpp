#include <MavsdkNode.h>
#include <RosNode.h>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <future>
#include "rclcpp/rclcpp.hpp"
#include "vehicle_interfaces/srv/arm.hpp"
#include "vehicle_interfaces/msg/test.hpp"
#include "vehicle_interfaces/msg/globalpos.hpp"
#include "vehicle_interfaces/msg/nedpos.hpp"
#include "vehicle_interfaces/msg/attitude.hpp"
#include "vehicle_interfaces/msg/actuatorsignals.hpp"


using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;
using namespace std::placeholders;





void RosNode::actuator_control_signals(const vehicle_interfaces::msg::Actuatorsignals::SharedPtr msg){
    this->mav_node->offboard_actuator_control(msg->steering, msg->throttle);
}


RosNode::RosNode(std::shared_ptr<rclcpp::Node> node) : ros_node(node)
{
    std::cout << "Creating RosNode ...\n";
}

void RosNode::arm_disarm(const std::shared_ptr<vehicle_interfaces::srv::Arm::Request> request, std::shared_ptr<vehicle_interfaces::srv::Arm::Response> response)
{
    (void)response;

    std::string mode;
    switch (request->mode)
    {
    case 1:
        mode = "ARMING";
        break;
    case 0:
        mode = "DISARMING";
        break;
    default:
        mode = "UNKNOWN";
        break;
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request %s",
                mode.c_str());

    this->mav_node->arm_disarm(request->mode);
}

void RosNode::init()
{

    // get and set parameters from yaml
    this->ros_node->declare_parameter("subscriptions.state", true);
    this->state_subscription = this->ros_node->get_parameter("subscriptions.state").as_bool();

    // binding arming and disarming service to the node
    rclcpp::Service<vehicle_interfaces::srv::Arm>::SharedPtr arm_disarm_service =
        this->ros_node->create_service<vehicle_interfaces::srv::Arm>("arm_disarm", std::bind(&RosNode::arm_disarm, this, _1, _2));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Arm Disarm Service Ready...");


    //
    this->actuator_subscriber = this->ros_node->create_subscription<vehicle_interfaces::msg::Actuatorsignals>("offboard_actuator_signals", 1, std::bind(&RosNode::testing, this, _1));

    // setup publishers and subscribers...
    if (state_subscription)
    {
        this->state_publisher = this->ros_node->create_publisher<vehicle_interfaces::msg::State>("state_vehicle", 1);
    }
}
