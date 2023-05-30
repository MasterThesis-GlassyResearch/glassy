#ifndef _RosNode_
#define _RosNode_

#include <iostream>
#include <chrono>
#include <thread>
#include <future>
#include "rclcpp/rclcpp.hpp"
#include "glassy_interfaces/msg/offboarddirectcontrol.hpp"
#include "glassy_interfaces/srv/arm.hpp"
#include "sensor_msgs/msg/joy.hpp"


enum mode{FULL_CONTROL, RUDDER_ONLY, THRUST_ONLY};
class JoyControlNode
{
private:
    // service and subscriber callbacks

    // values sent directly to actuators if correct mode is set
    float rudder_value = 0;
    float thrust_value = 0;  

    float rudder_increments = 0.1;
    float thrust_increments = 0.1;

    void direct_actuator_publish(float thrust, float rudder);


public:

    JoyControlNode(std::shared_ptr<rclcpp::Node> node);
    ~JoyControlNode(){};

    std::shared_ptr<rclcpp::Node> joy_glassy_node;


    // subscribe to topic comming from joystick
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_input_subscription;
    void joystick_subscription_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

    // publish directly to actuators
    rclcpp::Publisher<glassy_interfaces::msg::Offboarddirectcontrol>::SharedPtr glassy_interface_publisher;

    //client, will intercact with the arm/ disarm service
    rclcpp::Client<glassy_interfaces::srv::Arm>::SharedPtr arm_disarm_client;

    void init();

    mode joystick_mode = FULL_CONTROL;


    // if false, other sources can publish, this avoids conflicts
    bool direct_actuator_publishing;


};

#endif