#include "rclcpp/rclcpp.hpp"
#include "PIDControlNode.h"
#include "glassy_interfaces/msg/offboarddirectcontrol.hpp"
#include "glassy_interfaces/msg/innerloopreferences.hpp"
#include "glassy_interfaces/msg/state.hpp"
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <future>
#include <stdlib.h>

using namespace std::chrono_literals;
using namespace std::placeholders;

// class constructor 
PIDControlNode::PIDControlNode(std::shared_ptr<rclcpp::Node> node): pid_glassy_node(node)
{
    std::cout<<"Creating PID Controller Node...\n";
    this->prev_time = this->pid_glassy_node->get_clock()->now();
    this->ctrlType = SURGE_YAW;
}



void PIDControlNode::direct_actuator_publish(){

    // this->direct_actuator_msg.rudder = this->rudder_value + this->rudder_trim;
    // this->direct_actuator_msg.thrust = std::min(this->thrust_value, this->thrust_gain);
    // this->direct_actuator_msg.header.stamp = this->pid_glassy_node->get_clock()->now();

    // this->direct_actuator_msg.rudder = std::min(std::max(this->rudder_trim + this->rudder_gain *this->rudder_value * (1 - this->rudder_trim/this->rudder_gain), -1.f), 1.f);
    // this->direct_actuator_msg.thrust = std::min(std::max(this->thrust_trim + this->thrust_gain *  this->thrust_value * (1 - this->thrust_trim/this->thrust_gain), 0.f), 1.f);
    // this->direct_actuator_msg.header.stamp = this->pid_glassy_node->get_clock()->now();
    

    //float duration = this->pid_glassy_node->get_clock()->now() - this->prev_time;

    rclcpp::Time current_time = this->pid_glassy_node->get_clock()->now();

    rclcpp::Duration duration = current_time- this->prev_time;

    this->prev_time = current_time;
    


    // Start by taking care of the surge componnent:
    float pidValSurge = this->surgePIDCtrl.computePIDOutput(this->surge, this->surge_ref, duration.nanoseconds()/10e9, false);

    // Now take care of YAW or YAWRATE
    float pidValYaw =0.0;
    if(this->ctrlType == SURGE_YAW){
        if(this->yaw_ref-this->yaw>M_PI){
            this->yaw +=2*M_PI;
        } else if(this->yaw_ref-this->yaw<-M_PI){
            this->yaw -=2*M_PI;
        }

        pidValYaw = this->yawPIDCtrl.computePIDOutput(this->yaw, this->yaw_ref, duration.nanoseconds()/10e9, true);
    } 
    else{
        // pidValYaw = this->yawRatePIDCtrl.computePIDOutput(this->yawRate, this->yawRate_ref, duration.nanoseconds()/10e9, false);
    }




    /* --------------------------
        ADD THE 'CANCELLING PART'
    ----------------------------*/

    


    
    this->direct_actuator_msg.rudder = pidValYaw;
    this->direct_actuator_msg.thrust = std::min(std::max( pidValSurge, 0.f), 1.f);
    this->direct_actuator_msg.rudder = std::min(std::max( pidValYaw, -1.f), 1.f);

    this->direct_actuator_msg.header.stamp = current_time;





    this->actuator_publisher->publish(this->direct_actuator_msg);
}   





void PIDControlNode::referrence_subscription_callback(const glassy_interfaces::msg::Innerloopreferences::SharedPtr msg){
    // Set the correct references to track...
    this->surge_ref = msg->surge_ref;
    this->yaw_ref = msg->yaw_ref;
    this->yawRate_ref = msg->yaw_rate_ref;
}

void PIDControlNode::state_subscription_callback(const glassy_interfaces::msg::State::SharedPtr msg){
    this->surge = msg->surge_velocity;
    this->yaw = msg->yaw;
    this->yawRate = msg->yaw_rate;
}

void PIDControlNode::update_gains_surge_callback(const std::shared_ptr<glassy_interfaces::srv::Pidgains::Request> request, std::shared_ptr<glassy_interfaces::srv::Pidgains::Response> response){
    (void) response;
    this->surgePIDCtrl.set_gains(request->kp, request->ki, request->kd);
    this->surgePIDCtrl.reset_integral();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Surge PID gains set to: kp = %f, ki = %f, kd = %f", request->kp, request->ki, request->kd);
}

void PIDControlNode::update_gains_yaw_callback(const std::shared_ptr<glassy_interfaces::srv::Pidgains::Request> request, std::shared_ptr<glassy_interfaces::srv::Pidgains::Response> response){
    (void) response;
    this->yawPIDCtrl.set_gains(request->kp, request->ki, request->kd);
    this->yawPIDCtrl.reset_integral();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Yaw PID gains set to: kp = %f, ki = %f, kd = %f", request->kp, request->ki, request->kd);
}
void PIDControlNode::update_gains_yawRate_callback(const std::shared_ptr<glassy_interfaces::srv::Pidgains::Request> request, std::shared_ptr<glassy_interfaces::srv::Pidgains::Response> response){
    (void) response;
    this->yawRatePIDCtrl.set_gains(request->kp, request->ki, request->kd);
    this->yawRatePIDCtrl.reset_integral();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Yaw Rate PID gains set to: kp = %f, ki = %f, kd = %f", request->kp, request->ki, request->kd);
}

// for now a simple initialization, parameters may be added in the future
void PIDControlNode::init(){


    

    // /* -----------------------------
    //     Parameter Initialization
    // -------------------------------*/

    // // Declare all of the parameters 
    // this->pid_glassy_node->declare_parameter("mapping.axes.thrust", 1);

    // // Initialize all the parameters
    // this->thrust_mapping = this->pid_glassy_node->get_parameter("mapping.axes.thrust").as_int();

    // /* -----------------------------
    //     Variable Initialization
    // -------------------------------*/

    // this->thrust_gain = this->pid_glassy_node->get_parameter("thrust_gain").as_double();

    this->surgePIDCtrl.set_gains(1.f,0.1f,0);
    this->yawPIDCtrl.set_gains(0.01f,0.00001f,0.1f);



    
    /* -----------------------------
        ROS2 Service Initialization
    -------------------------------*/


    // subscribe to the reference topic
    this->ref_subscription = this->pid_glassy_node->create_subscription<glassy_interfaces::msg::Innerloopreferences>("inner_loop_ref", 1, std::bind(&PIDControlNode::referrence_subscription_callback, this, _1));

    // subscribe to the joystick topic
    this->state_subscription = this->pid_glassy_node->create_subscription<glassy_interfaces::msg::State>("state_vehicle", 1, std::bind(&PIDControlNode::state_subscription_callback, this, _1));

    // initialize publisher
    this->actuator_publisher = this->pid_glassy_node->create_publisher<glassy_interfaces::msg::Offboarddirectcontrol>("offboard_direct_signals", 1);

    this->timer = this->pid_glassy_node->create_wall_timer(50ms, std::bind(&PIDControlNode::direct_actuator_publish, this));

    // Services
    this->change_gains_surge = this->pid_glassy_node->create_service<glassy_interfaces::srv::Pidgains>("pid_gains_surge", std::bind(&PIDControlNode::update_gains_surge_callback, this, _1, _2));
    this->change_gains_yaw = this->pid_glassy_node->create_service<glassy_interfaces::srv::Pidgains>("pid_gains_yaw", std::bind(&PIDControlNode::update_gains_yaw_callback, this, _1, _2));
    this->change_gains_yawRate = this->pid_glassy_node->create_service<glassy_interfaces::srv::Pidgains>("pid_gains_yaw_rate", std::bind(&PIDControlNode::update_gains_yawRate_callback, this, _1, _2));
   
   
   
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Inner Loops ready...");
}
