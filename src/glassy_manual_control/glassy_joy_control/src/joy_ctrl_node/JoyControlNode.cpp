
#include "sensor_msgs/msg/joy.hpp"
#include "rclcpp/rclcpp.hpp"
#include "JoyControlNode.h"
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <future>

using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;
using namespace std::placeholders;

// class constructor 
JoyControlNode::JoyControlNode(std::shared_ptr<rclcpp::Node> node): joy_glassy_node(node)
{
    std::cout<<"Creating Joystic Controller Node...\n";
}


void JoyControlNode::direct_actuator_publish(float thrust, float rudder){

}   



void JoyControlNode::joystick_subscription_callback(const sensor_msgs::msg::Joy::SharedPtr msg){
    // TEMPORARY VARIABLES, MUST BE CHANGED BY PARAMETRS COMMING FROM YAML ---- TESTING ONLY
    int thrust_axis_index = 1;
    int rudder_axis_index = 2;
    int arm_button_index = 10;
    int disarm_button_index = 9;

    if(this->joystick_mode==THRUST_ONLY){

        
    }
    else if(this->joystick_mode==RUDDER_ONLY){

    } else{
        
    }

    return;
}

// for now a simple initialization, parameters may be added in the future
void JoyControlNode::init(){

    // initialize the arm disarm client
    this->arm_disarm_client = this->joy_glassy_node->create_client<glassy_interfaces::srv::Arm>("arm_disarm");

    // subscribe to the joystick topic
    this->joystick_input_subscription = this->joy_glassy_node->create_subscription<sensor_msgs::msg::Joy>("joy", 1, std::bind(&JoyControlNode::joystick_subscription_callback, this, _1));

    // initialize publisher
    this->glassy_interface_publisher = this->joy_glassy_node->create_publisher<glassy_interfaces::msg::Offboarddirectcontrol>("offboard_direct_signals", 1);

}