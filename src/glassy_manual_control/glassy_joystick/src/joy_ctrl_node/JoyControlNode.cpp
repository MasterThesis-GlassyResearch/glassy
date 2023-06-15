
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

    // if any control given to the joystick, publish direct controls
    if(this->joystick_mode!=NO_CONTROL){

        // get relevant information from joystick depending on joystick mode
        if(this->joystick_mode==THRUST_ONLY){
            this->thrust_value = msg->axes[this->thrust_mapping]; 
        }
        else if(this->joystick_mode==RUDDER_ONLY){
            this->rudder_value = msg->axes[this->steering_mapping];
        } 
        else{
            this->rudder_value = msg->axes[this->steering_mapping];
            this->thrust_value = msg->axes[this->thrust_mapping];
        }
            this->direct_actuator_publish();
    }

    // check if either disarm pressed or armed pressed
    if(msg->buttons[this->disarm_mapping]==1){
        this->arm_request->mode = 0; 
        this->arm_disarm_client->async_send_request(this->arm_request);
    }
    else if(msg->buttons[this->arm_mapping]==1){
        this->arm_request->mode = 1; 
        this->arm_disarm_client->async_send_request(this->arm_request);
    }

    // check if either stop or start offboard pressed
    if(msg->buttons[this->stop_offboard_mapping]==1){
        this->offboard_request->mode = 0; 
        this->start_stop_offboard_client->async_send_request(this->offboard_request);
    } 
    else if(msg->buttons[this->start_offboard_mapping]==1){
        this->offboard_request->mode = 1;
        this->start_stop_offboard_client->async_send_request(this->offboard_request);
    }

}




void JoyControlNode::change_mode(mode new_mode){
    this->joystick_mode = new_mode;
    this->rudder_value = 0.0;
    this->thrust_value = 0.0;
}

// for now a simple initialization, parameters may be added in the future
void JoyControlNode::init(){

    // ---------------------------------------------------------------------
    //  get the necessary parameters from yaml file -> DEFAULT FOR PS4 
    //  [ x, y]
    //  if x = 1 -> button, if = 0 -> axis
    //  y = number of button/ axis 
    // ----------------------------------------------------------------------
    this->joy_glassy_node->declare_parameter("mapping.axes.thrust", 1);
    this->joy_glassy_node->declare_parameter("mapping.axes.steering", 2);

    this->joy_glassy_node->declare_parameter("mapping.buttons.arm", 0);
    this->joy_glassy_node->declare_parameter("mapping.buttons.disarm", 2);
    this->joy_glassy_node->declare_parameter("mapping.buttons.start_offboard", 3);
    this->joy_glassy_node->declare_parameter("mapping.buttons.stop_offboard", 4);

    // this->joy_glassy_node->declare_parameter("mapping.others.const_val_increase", std::vector<int>{1,3});
    // this->joy_glassy_node->declare_parameter("mapping.others.const_val_decrease", std::vector<int>{1,4});
    // this->joy_glassy_node->declare_parameter("mapping.others.step_increase", std::vector<int>{0,1});
    // this->joy_glassy_node->declare_parameter("mapping.others.step_decrease", std::vector<int>{0,1});

    // get all the parameters
    this->thrust_mapping = this->joy_glassy_node->get_parameter("mapping.axes.thrust").as_int();
    this->steering_mapping = this->joy_glassy_node->get_parameter("mapping.axes.steering").as_int();
    this->arm_mapping = this->joy_glassy_node->get_parameter("mapping.buttons.arm").as_int();
    this->disarm_mapping = this->joy_glassy_node->get_parameter("mapping.buttons.disarm").as_int();
    this->start_offboard_mapping = this->joy_glassy_node->get_parameter("mapping.buttons.start_offboard").as_int();
    this->stop_offboard_mapping = this->joy_glassy_node->get_parameter("mapping.buttons.stop_offboard").as_int();

    // this->const_val_increase_mapping = this->joy_glassy_node->get_parameter("mapping.start_offboard").as_integer_array();
    // this->const_val_decrease_mapping = this->joy_glassy_node->get_parameter("mapping.stop_offboard").as_integer_array();
    // this->step_increase_mapping = this->joy_glassy_node->get_parameter("mapping.step_increase").as_integer_array();
    // this->step_decrease_mapping = this->joy_glassy_node->get_parameter("mapping.step_decrease").as_integer_array();



    // initialize the arm disarm client
    this->arm_disarm_client = this->joy_glassy_node->create_client<glassy_interfaces::srv::Arm>("arm_disarm");

    // intialize start/stop offboard client
    this->start_stop_offboard_client = this->joy_glassy_node->create_client<glassy_interfaces::srv::Arm>("start_stop_offboard");

    // subscribe to the joystick topic
    this->joystick_input_subscription = this->joy_glassy_node->create_subscription<sensor_msgs::msg::Joy>("joy", 1, std::bind(&JoyControlNode::joystick_subscription_callback, this, _1));

    // initialize publisher
    this->glassy_interface_publisher = this->joy_glassy_node->create_publisher<glassy_interfaces::msg::Offboarddirectcontrol>("offboard_direct_signals", 1);

}
