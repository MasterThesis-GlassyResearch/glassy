
#include "sensor_msgs/msg/joy.hpp"
#include "rclcpp/rclcpp.hpp"
#include "JoyControlNode.h"
#include "glassy_interfaces/msg/offboarddirectcontrol.hpp"
#include "glassy_interfaces/srv/arm.hpp"
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


void JoyControlNode::direct_actuator_publish(){
    this->direct_actuator_msg.rudder = this->rudder_value;
    this->direct_actuator_msg.thrust = this->thrust_value;

    this->glassy_interface_publisher->publish(this->direct_actuator_msg);
}   



void JoyControlNode::joystick_subscription_callback(const sensor_msgs::msg::Joy::SharedPtr msg){

    // get relevant information from joystick depending on joystick mode
    if(this->joystick_mode==THRUST_ONLY){
        this->thrust_value = msg->axes[this->thrust_axis_index];
    }
    else if(this->joystick_mode==RUDDER_ONLY){
        this->rudder_value = msg->axes[this->rudder_axis_index];
    } 
    else{
        this->rudder_value = msg->axes[this->rudder_axis_index];
        this->thrust_value = msg->axes[this->thrust_axis_index];
    }

    // if any control given to the joystick, publish direct controls
    if(this->joystick_mode!=NO_CONTROL){
        this->direct_actuator_publish();
    }

    // check if arming button was pressed, if so call arming service


    // check if disarm button pressed, if so call disarm service


    // check if offboard button pressed, if so call offboard enter service

    // check for increases in constant value (thrust/steering) 

    // check for decreases in constant value (thrust/steering) 


    // check for 


}



void JoyControlNode::change_mode(mode new_mode){
    this->joystick_mode = new_mode;
    this->rudder_value = 0.0;
    this->thrust_value = 0.0;
}

// for now a simple initialization, parameters may be added in the future
void JoyControlNode::init(){

    // get the necessary parameters from yaml file -> DEFAULT FOR PS4
    this->joy_glassy_node->declare_parameter("mapping.thrust", 1);
    this->joy_glassy_node->declare_parameter("mapping.steering", 2);
    this->joy_glassy_node->declare_parameter("mapping.arm", 10);
    this->joy_glassy_node->declare_parameter("mapping.disarm", 9);
    this->joy_glassy_node->declare_parameter("mapping.const_val_increase", 9);
    this->joy_glassy_node->declare_parameter("mapping.const_val_decrease", 9);
    this->joy_glassy_node->declare_parameter("mapping.step_increase", 9);
    this->joy_glassy_node->declare_parameter("mapping.step.decrease", 9);

    // Initiate the axis/button index values
    this->arm_button_index = this->joy_glassy_node->get_parameter("mapping.arm").as_int();
    this->rudder_axis_index = this->joy_glassy_node->get_parameter("mapping.steering").as_int();
    this->thrust_axis_index = this->joy_glassy_node->get_parameter("mapping.thrust").as_int();
    this->disarm_button_index = this->joy_glassy_node->get_parameter("mapping.disarm").as_int();


    // initialize the arm disarm client
    this->arm_disarm_client = this->joy_glassy_node->create_client<glassy_interfaces::srv::Arm>("arm_disarm");

    // subscribe to the joystick topic
    this->joystick_input_subscription = this->joy_glassy_node->create_subscription<sensor_msgs::msg::Joy>("joy", 1, std::bind(&JoyControlNode::joystick_subscription_callback, this, _1));

    // initialize publisher
    this->glassy_interface_publisher = this->joy_glassy_node->create_publisher<glassy_interfaces::msg::Offboarddirectcontrol>("offboard_direct_signals", 1);

}