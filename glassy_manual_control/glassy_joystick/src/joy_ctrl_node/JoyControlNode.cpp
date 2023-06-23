
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
#include <stdlib.h>

using namespace std::placeholders;

// class constructor 
JoyControlNode::JoyControlNode(std::shared_ptr<rclcpp::Node> node): joy_glassy_node(node)
{
    std::cout<<"Creating Joystic Controller Node...\n";
}


void JoyControlNode::direct_actuator_publish(){
    this->direct_actuator_msg.rudder = this->rudder_value;
    this->direct_actuator_msg.thrust = this->thrust_value;
    this->direct_actuator_msg.header.stamp = this->joy_glassy_node->get_clock()->now();

    this->glassy_interface_publisher->publish(this->direct_actuator_msg);
}   

// FIXME -> Test all of the functionalities as soon as possible
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


    // check for increases in the constant value
    if(msg->axes[this->const_val_inc_dec_mapping]==1){
        if(this->joystick_mode==THRUST_ONLY){
            this->rudder_value+=this->increments_const_val;
            this->rudder_value= std::min(this->rudder_value, 1.f);
            RCLCPP_INFO(this->joy_glassy_node->get_logger(), "Current Rudder Value = "+ std::to_string(this->rudder_value));
        }
        else if(this->joystick_mode==RUDDER_ONLY){
            this->thrust_value+=this->increments_const_val;
            this->thrust_value= std::min(this->thrust_value, 1.f);
            RCLCPP_INFO(this->joy_glassy_node->get_logger(), "Current Thrust Value = "+ std::to_string(this->thrust_value));
        }
    }
    // check for decreses in the constant value
    else if(msg->axes[this->const_val_inc_dec_mapping]==-1){
        if(this->joystick_mode==THRUST_ONLY){
            this->rudder_value-=this->increments_const_val;
            this->rudder_value= std::max(this->rudder_value, -1.f);
            RCLCPP_INFO(this->joy_glassy_node->get_logger(), "Current Rudder Value = "+ std::to_string(this->rudder_value));
        }
        else if(this->joystick_mode==RUDDER_ONLY){
            this->thrust_value-=this->increments_const_val;
            this->thrust_value= std::max(this->thrust_value, 0.f);
            RCLCPP_INFO(this->joy_glassy_node->get_logger(), "Current Thrust Value = "+ std::to_string(this->thrust_value));
        }
    }
    // check increases in the step value
    else if(msg->axes[this->step_inc_dec_mapping]==1){
        this->increments_const_val+=this->increments_on_step;
        this->increments_const_val = std::min(this->max_increment_value, this->increments_const_val);
        RCLCPP_INFO(this->joy_glassy_node->get_logger(), "Current Increment Value = "+ std::to_string(this->increments_const_val));
    }
    // check decreases in the step value
    else if(msg->axes[this->step_inc_dec_mapping]==-1){
        this->increments_const_val-=this->increments_on_step;
        this->increments_const_val = std::max(this->increments_on_step, this->increments_const_val);
        RCLCPP_INFO(this->joy_glassy_node->get_logger(), "Current Increment Value = "+ std::to_string(this->increments_const_val));
    }
    else if(msg->buttons[this->toogle_mode_mapping]){
        this->next_mode_index = (this->next_mode_index+1)%this->nmr_modes;
        RCLCPP_INFO(this->joy_glassy_node->get_logger(), "Mode To Change to -> "+ this->list_modes_names[this->next_mode_index]);
    }
    else if(msg->buttons[this->enter_mode_mapping]){
        this->change_mode(this->list_modes[this->next_mode_index]);
        RCLCPP_INFO(this->joy_glassy_node->get_logger(), "Mode Entered -> "+ this->list_modes_names[this->next_mode_index]);
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
    //  get the necessary parameters from yaml file -> DEFAULT FOR PS4 Controller
    // ----------------------------------------------------------------------
    this->joy_glassy_node->declare_parameter("mapping.axes.thrust", 1);
    this->joy_glassy_node->declare_parameter("mapping.axes.steering", 2);
    this->joy_glassy_node->declare_parameter("mapping.axes.const_val_increase_decrease", 6);
    this->joy_glassy_node->declare_parameter("mapping.axes.step_increase_decrease", 7);

    this->joy_glassy_node->declare_parameter("mapping.buttons.arm", 0);
    this->joy_glassy_node->declare_parameter("mapping.buttons.disarm", 2);
    this->joy_glassy_node->declare_parameter("mapping.buttons.start_offboard", 3);
    this->joy_glassy_node->declare_parameter("mapping.buttons.stop_offboard", 4);
    this->joy_glassy_node->declare_parameter("mapping.buttons.enter_mode", 6);
    this->joy_glassy_node->declare_parameter("mapping.buttons.toogle_mode", 7);

    this->joy_glassy_node->declare_parameter("initial.const_step_value", 0.05);
    this->joy_glassy_node->declare_parameter("initial.step_inc_dec", 0.01);
    this->joy_glassy_node->declare_parameter("initial.max_const_step_val", 0.15);


    // Initialize all the parameters
    this->thrust_mapping = this->joy_glassy_node->get_parameter("mapping.axes.thrust").as_int();
    this->steering_mapping = this->joy_glassy_node->get_parameter("mapping.axes.steering").as_int();
    this->arm_mapping = this->joy_glassy_node->get_parameter("mapping.buttons.arm").as_int();
    this->disarm_mapping = this->joy_glassy_node->get_parameter("mapping.buttons.disarm").as_int();
    this->start_offboard_mapping = this->joy_glassy_node->get_parameter("mapping.buttons.start_offboard").as_int();
    this->stop_offboard_mapping = this->joy_glassy_node->get_parameter("mapping.buttons.stop_offboard").as_int();

    this->const_val_inc_dec_mapping = this->joy_glassy_node->get_parameter("mapping.axes.const_val_increase_decrease").as_int();
    this->step_inc_dec_mapping = this->joy_glassy_node->get_parameter("mapping.axes.step_increase_decrease").as_int();

    this->increments_const_val = this->joy_glassy_node->get_parameter("initial.const_step_value").as_double();
    this->increments_on_step = this->joy_glassy_node->get_parameter("initial.step_inc_dec").as_double();
    this->max_increment_value = this->joy_glassy_node->get_parameter("initial.max_const_step_val").as_double();

    this->enter_mode_mapping = this->joy_glassy_node->get_parameter("mapping.buttons.enter_mode").as_int();
    this->toogle_mode_mapping = this->joy_glassy_node->get_parameter("mapping.buttons.toogle_mode").as_int();

    this->nmr_modes = this->list_modes.size();



    // initialize the arm disarm client
    this->arm_disarm_client = this->joy_glassy_node->create_client<glassy_interfaces::srv::Arm>("arm_disarm");

    // intialize start/stop offboard client
    this->start_stop_offboard_client = this->joy_glassy_node->create_client<glassy_interfaces::srv::Arm>("start_stop_offboard");

    // subscribe to the joystick topic
    this->joystick_input_subscription = this->joy_glassy_node->create_subscription<sensor_msgs::msg::Joy>("joy", 1, std::bind(&JoyControlNode::joystick_subscription_callback, this, _1));

    // initialize publisher
    this->glassy_interface_publisher = this->joy_glassy_node->create_publisher<glassy_interfaces::msg::Offboarddirectcontrol>("offboard_direct_signals", 1);

}
