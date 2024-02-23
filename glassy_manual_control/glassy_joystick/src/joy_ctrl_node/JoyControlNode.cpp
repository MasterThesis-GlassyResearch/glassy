
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

void JoyControlNode::keep_track_button_pressed(int* new_button, int new_val){
    *(this->last_pressed_btn) = 0;
    this->last_pressed_btn = new_button;
    *(this->last_pressed_btn) = new_val;
}

void JoyControlNode::direct_actuator_publish(){

    // this->direct_actuator_msg.rudder = this->rudder_value + this->rudder_trim;
    // this->direct_actuator_msg.thrust = std::min(this->thrust_value, this->thrust_gain);
    // this->direct_actuator_msg.header.stamp = this->joy_glassy_node->get_clock()->now();

    this->direct_actuator_msg.rudder = std::min(std::max(this->rudder_trim + this->rudder_gain *this->rudder_value * (1 - this->rudder_trim/this->rudder_gain), -1.f), 1.f);
    this->direct_actuator_msg.thrust = std::min(std::max(this->thrust_trim + this->thrust_gain *  this->thrust_value * (1 - this->thrust_trim/this->thrust_gain), 0.f), 1.f);
    this->direct_actuator_msg.header.stamp = this->joy_glassy_node->get_clock()->now();

    // std::cout<< "Thrust: " << this->direct_actuator_msg.thrust << std::endl;
    
    this->glassy_interface_publisher->publish(this->direct_actuator_msg);
}   


void JoyControlNode::joystick_subscription_callback(const sensor_msgs::msg::Joy::SharedPtr msg){

    // if any control given to the joystick, publish direct controls
    if(this->joystick_mode!=NO_CONTROL){
        this->rudder_value = -msg->axes[this->steering_mapping];
        this->thrust_value = msg->axes[this->thrust_mapping];
        this->direct_actuator_publish();
    }


    // check if either disarm pressed or armed pressed
    if(msg->buttons[this->disarm_mapping]==1){
        if(this->disarm_previous_value!=0) return;
        this->arm_request->mode = 0; 
        this->arm_disarm_client->async_send_request(this->arm_request);
        RCLCPP_INFO(this->joy_glassy_node->get_logger(), "Sending DISARM request");

        this->keep_track_button_pressed(&(this->disarm_previous_value), 1);
    }
    // check if either stop or start offboard pressed
    else if(msg->buttons[this->stop_offboard_mapping]==1){
        if(this->stop_offboard_previous_value!=0) return;
        this->offboard_request->mode = 0; 
        this->start_stop_offboard_client->async_send_request(this->offboard_request);
        RCLCPP_INFO(this->joy_glassy_node->get_logger(), "Sending STOP OFFBOARD request");

        this->keep_track_button_pressed(&(this->stop_offboard_previous_value), 1);
    } 
    else if(msg->buttons[this->start_offboard_mapping]==1){
        if(this->start_offboard_previous_value!=0) return;
        this->offboard_request->mode = 1;
        this->start_stop_offboard_client->async_send_request(this->offboard_request);
        RCLCPP_INFO(this->joy_glassy_node->get_logger(), "Sending START OFFBOARD request");

        this->keep_track_button_pressed(&(this->start_offboard_previous_value), 1);
    }
    else if(msg->buttons[this->arm_mapping]==1){
        if(this->arm_previous_value!=0) return;

        this->arm_request->mode = 1; 
        this->arm_disarm_client->async_send_request(this->arm_request);
        RCLCPP_INFO(this->joy_glassy_node->get_logger(), "Sending ARM request");

        this->keep_track_button_pressed(&(this->arm_previous_value), 1);
    }
    // check for increases in the constant value
    else if(msg->axes[this->const_val_inc_dec_mapping]==1){
        if(this->const_val_inc_dec_previous_value==1) return;

        if(this->joystick_mode==THRUST_TRIM){
            this->thrust_trim+=this->increments_const_val;
            this->thrust_trim= std::min(this->thrust_trim, 1.f);
            RCLCPP_INFO(this->joy_glassy_node->get_logger(), "Current Thrust Trim Value = %.2f", this->thrust_trim);
        }
        else if(this->joystick_mode==RUDDER_TRIM){
            this->rudder_trim+=this->increments_const_val;
            this->rudder_trim= std::min(this->rudder_trim, 1.f);
            RCLCPP_INFO(this->joy_glassy_node->get_logger(), "Current Rudder Trim Value = %.2f", this->rudder_trim);
        }
        else if(this->joystick_mode==FULL_CONTROL){
            this->thrust_gain+=this->increments_const_val;
            this->thrust_gain= std::min(this->thrust_gain, 1.f);
            RCLCPP_INFO(this->joy_glassy_node->get_logger(), "Current Thrust Gain Value = ",this->thrust_gain);
        }
        else if(this->joystick_mode==RUDDER_GAIN){
            this->rudder_gain+=this->increments_const_val;
            this->rudder_gain= std::min(this->rudder_gain, 1.f);
            RCLCPP_INFO(this->joy_glassy_node->get_logger(), "Current Thrust Gain Value = ",this->rudder_gain);
        }
        this->keep_track_button_pressed(&(this->const_val_inc_dec_previous_value), 1);
    }
    // check for decreses in the constant value
    else if(msg->axes[this->const_val_inc_dec_mapping]==-1){
        if(this->const_val_inc_dec_previous_value==-1) return;

        if(this->joystick_mode==THRUST_TRIM){
            this->thrust_trim-=this->increments_const_val;
            this->thrust_trim= std::max(this->thrust_trim, 0.f);
            RCLCPP_INFO(this->joy_glassy_node->get_logger(), "Current Thrust Value = %.2f",this->thrust_trim);
        }
        else if(this->joystick_mode==RUDDER_TRIM){
            this->rudder_trim-=this->increments_const_val;
            this->rudder_trim= std::max(this->rudder_trim, -1.f);
            RCLCPP_INFO(this->joy_glassy_node->get_logger(), "Current Rudder Trim = %.2f",this->rudder_trim);
        }
        else if(this->joystick_mode==FULL_CONTROL){
            this->thrust_gain-=this->increments_const_val;
            this->thrust_gain= std::max(this->thrust_gain, 0.f);
            RCLCPP_INFO(this->joy_glassy_node->get_logger(), "Current Thrust Gain Value = %.2f",this->thrust_gain);
        }
        else if(this->joystick_mode==RUDDER_GAIN){
            this->rudder_gain-=this->increments_const_val;
            this->rudder_gain= std::max(this->rudder_gain, 0.f);
            RCLCPP_INFO(this->joy_glassy_node->get_logger(), "Current Thrust Gain Value = %.2f",this->rudder_gain);
        }
        this->keep_track_button_pressed(&(this->const_val_inc_dec_previous_value), -1);
    }
    // check increases in the step value
    else if(msg->axes[this->step_inc_dec_mapping]==-1){
        if(this->step_inc_dec_previous_value==1) return;
        this->increments_const_val+=this->increments_on_step;
        this->increments_const_val = std::min(this->max_increment_value, this->increments_const_val);
        RCLCPP_INFO(this->joy_glassy_node->get_logger(), "Current Increment Value = %.2f",this->increments_const_val);

        this->keep_track_button_pressed(&(this->step_inc_dec_previous_value), 1);
    }
    // check decreases in the step value
    else if(msg->axes[this->step_inc_dec_mapping]==1 ){
        if(this->step_inc_dec_previous_value==-1) return;
        this->increments_const_val-=this->increments_on_step;
        this->increments_const_val = std::max(this->increments_on_step, this->increments_const_val);
        RCLCPP_INFO(this->joy_glassy_node->get_logger(), "Current Increment Value = %.2f",this->increments_const_val);

        this->keep_track_button_pressed(&(this->step_inc_dec_previous_value), -1);
    }

    // CHANGE AND ENTER MODES ------------------------------------------
    else if(msg->buttons[this->toogle_mode_mapping]==1){
        if(this->toogle_mode_previous_value!=0) return;
        this->next_mode_index = (this->next_mode_index+1)%this->nmr_modes;
        RCLCPP_INFO(this->joy_glassy_node->get_logger(), "Mode To Change to -> %s", this->list_modes_names[this->next_mode_index].c_str());

        this->keep_track_button_pressed(&(this->toogle_mode_previous_value), 1);
    }
    else if(msg->buttons[this->enter_mode_mapping]==1){
        if(this->enter_mode_previous_value!=0) return;
        this->change_mode(this->list_modes[this->next_mode_index]);
        RCLCPP_INFO(this->joy_glassy_node->get_logger(), "Mode Entered -> %s",this->list_modes_names[this->next_mode_index].c_str());

        this->keep_track_button_pressed(&(this->enter_mode_previous_value), 1);
    }
    
    // If nothing pressed, set previoulsy set button to 0 ---------------
    else{
        *(this->last_pressed_btn) = 0;
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
    //TODO make default logitech controller
    // ----------------------------------------------------------------------
    this->joy_glassy_node->declare_parameter("mapping.axes.thrust", 1);
    this->joy_glassy_node->declare_parameter("mapping.axes.steering", 3);
    this->joy_glassy_node->declare_parameter("mapping.axes.const_val_increase_decrease", 7);
    this->joy_glassy_node->declare_parameter("mapping.axes.step_increase_decrease", 6);

    this->joy_glassy_node->declare_parameter("mapping.buttons.arm", 1);
    this->joy_glassy_node->declare_parameter("mapping.buttons.disarm", 0);
    this->joy_glassy_node->declare_parameter("mapping.buttons.start_offboard", 2);
    this->joy_glassy_node->declare_parameter("mapping.buttons.stop_offboard", 3);
    this->joy_glassy_node->declare_parameter("mapping.buttons.enter_mode", 5);
    this->joy_glassy_node->declare_parameter("mapping.buttons.toogle_mode", 4);

    this->joy_glassy_node->declare_parameter("initial.const_step_value", 0.05);
    this->joy_glassy_node->declare_parameter("initial.step_inc_dec", 0.01);
    this->joy_glassy_node->declare_parameter("initial.max_const_step_val", 0.15);
    
    this->joy_glassy_node->declare_parameter("thrust_trim", 0.0);
    this->joy_glassy_node->declare_parameter("thrust_gain", 1.0);
    this->joy_glassy_node->declare_parameter("rudder_gain", 1.0);
    this->joy_glassy_node->declare_parameter("rudder_trim", 0.15);




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


    this->thrust_gain = this->joy_glassy_node->get_parameter("thrust_gain").as_double();
    this->thrust_trim = this->joy_glassy_node->get_parameter("thrust_trim").as_double();
    this->rudder_gain = this->joy_glassy_node->get_parameter("rudder_gain").as_double();
    this->rudder_trim = this->joy_glassy_node->get_parameter("rudder_trim").as_double();

    this->nmr_modes = this->list_modes.size();

    // all buttons start unpressed
    this->arm_previous_value = 0;        
    this->disarm_previous_value = 0;
    this->start_offboard_previous_value = 0;
    this->stop_offboard_previous_value = 0;
    this->const_val_inc_dec_previous_value = 0;
    this->step_inc_dec_previous_value = 0;
    this->toogle_mode_previous_value = 0;
    this->enter_mode_previous_value = 0;

    

    this->last_pressed_btn = &(this->disarm_previous_value);

    // initialize the arm disarm client
    this->arm_disarm_client = this->joy_glassy_node->create_client<glassy_interfaces::srv::Arm>("arm_disarm");

    // intialize start/stop offboard client
    this->start_stop_offboard_client = this->joy_glassy_node->create_client<glassy_interfaces::srv::Arm>("start_stop_offboard");

    // subscribe to the joystick topic
    this->joystick_input_subscription = this->joy_glassy_node->create_subscription<sensor_msgs::msg::Joy>("joy", 1, std::bind(&JoyControlNode::joystick_subscription_callback, this, _1));

    // initialize publisher
    this->glassy_interface_publisher = this->joy_glassy_node->create_publisher<glassy_interfaces::msg::Offboarddirectcontrol>("offboard_direct_signals", 1);

}
