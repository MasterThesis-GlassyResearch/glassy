#include "rclcpp/rclcpp.hpp"
#include "PIDControlNode.h"
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


/*-------------------------------------
         Timer CallBack
-------------------------------------*/

void PIDControlNode::direct_actuator_publish(){

    // first check whether or not it is active
    if(!this->is_active){
        return;
    }


    rclcpp::Time current_time = this->pid_glassy_node->get_clock()->now();

    rclcpp::Duration duration = current_time - this->prev_time;

    this->prev_time = current_time;
    


    // Start by taking care of the surge componnent:
    float pidValSurge = this->surgePIDCtrl.computePIDOutput(this->surge, this->surge_ref, duration.nanoseconds()/10e9, true);

    // Now take care of YAW or YAWRATE
    float pidValYaw =0.0;
    if(this->ctrlType == SURGE_YAW){

        // take care of ensuring shortest way to desired yaw
        if(this->yaw_ref-this->yaw>M_PI){
            this->yaw +=2*M_PI;
        } else if(this->yaw_ref-this->yaw<-M_PI){
            this->yaw -=2*M_PI;
        }

        pidValYaw = this->yawPIDCtrl.computePIDOutput(this->yaw, this->yaw_ref, duration.nanoseconds()/10e9, false);
    } 
    else if(this->ctrlType == SURGE_YAWRATE){
        pidValYaw = this->yawRatePIDCtrl.computePIDOutput(this->yawRate, this->yawRate_ref, duration.nanoseconds()/10e9, false);
    } else{
        pidValYaw=0.0;
    }




    /* --------------------------
        ADD THE 'CANCELLING PART'
    ----------------------------*/

    // calculate surge canceling part:
    float cancel_surge = this->surgeParams[0]*this->sway*this->yawRate + this->surgeParams[1]*this->surge + this->surgeParams[2]*this->surge*this->surge+ abs(this->yawRate)*this->surgeParams[7]*this->surge + abs(this->yawRate)*this->surgeParams[8]*this->surge*this->surge;
    float cancel_yaw = this->yawRateParams[0]*this->sway*this->surge + this->yawRateParams[1]*this->yawRate + this->yawRateParams[2]*this->yawRate*abs(this->yawRate) + this->surge*this->yawRate*this->yawRateParams[5] + this->surge*this->surge*this->yawRateParams[6];

    float surge_force = pidValSurge - cancel_surge;
    float yaw_force = pidValYaw - cancel_yaw;

    std::cout<< "yaw_force" <<yaw_force <<std::endl;


    float Eff = this->surgeParams[3]*(1-exp(-(this->surgeParams[4]* this->surge*this->surge + this->surgeParams[5])/(abs(this->yawRate)+this->epsilon_cnst)));

    if(std::isnan(Eff)){
        Eff = this->surgeParams[3];
    }

    float thrust_val = (surge_force/Eff)/700 + this->surgeParams[6]/1000;

    float thrust_usefull_pwm = surge_force/Eff+this->surgeParams[6];


    float sin_rudder_angle_degrees;
    if(this->surge>10e-10){
        sin_rudder_angle_degrees = yaw_force/(this->yawRateParams[3]*this->surge*this->surge + this->yawRateParams[4]*thrust_usefull_pwm);
        std::cout<< "Sin rudder degrees" <<sin_rudder_angle_degrees <<std::endl;
    } else{
        sin_rudder_angle_degrees = 0.0;
    }

    sin_rudder_angle_degrees = std::min(std::max(sin_rudder_angle_degrees,-1.f), 1.f);
    std::cout<< "Sin rudder degrees" <<sin_rudder_angle_degrees <<std::endl;


    // float rudder_pwm = (asin(sin_rudder_angle_degrees)*180/M_PI -( this->angle_params[1]))/this->angle_params[0];
    float rudder_pwm = (asin(sin_rudder_angle_degrees)*180/M_PI)/this->angle_params[0];
    std::cout<< "Rudder val" <<rudder_pwm <<std::endl;

    float rudder_val = (rudder_pwm)/600;
    std::cout<< "Rudder val" <<rudder_val <<std::endl;




    
    /* --------------------------
        Publish to the actuators
    ----------------------------*/

    this->direct_actuator_msg.thrust = std::min(std::max( thrust_val, 0.f), 1.f);
    this->direct_actuator_msg.rudder = std::min(std::max( rudder_val, -1.f), 1.f);

    this->direct_actuator_msg.header.stamp = current_time;
    this->actuator_publisher->publish(this->direct_actuator_msg);
}   


/*-------------------------------------
         Subscription CallBacks 
-------------------------------------*/


void PIDControlNode::referrence_subscription_callback(const glassy_interfaces::msg::InnerLoopReferences::SharedPtr msg){
    // Set the correct references to track...
    this->surge_ref = msg->surge_ref;
    this->yaw_ref = msg->yaw_ref;
    this->yawRate_ref = msg->yaw_rate_ref;
}

void PIDControlNode::state_subscription_callback(const glassy_interfaces::msg::State::SharedPtr msg){
    this->surge = msg->surge_velocity;
    this->yaw = msg->yaw;
    this->yawRate = msg->yaw_rate;
    this->sway = msg->sway_velocity;
}


/*-------------------------------------
         Service CallBacks 
-------------------------------------*/

void PIDControlNode::update_gains_surge_callback(const std::shared_ptr<glassy_interfaces::srv::PidGains::Request> request, std::shared_ptr<glassy_interfaces::srv::PidGains::Response> response){
    (void) response;
    this->surgePIDCtrl.set_gains(request->kp, request->ki, request->kd);
    this->surgePIDCtrl.reset_integral();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Surge PID gains set to: kp = %f, ki = %f, kd = %f", request->kp, request->ki, request->kd);
}

void PIDControlNode::update_gains_yaw_callback(const std::shared_ptr<glassy_interfaces::srv::PidGains::Request> request, std::shared_ptr<glassy_interfaces::srv::PidGains::Response> response){
    (void) response;
    this->yawPIDCtrl.set_gains(request->kp, request->ki, request->kd);
    this->yawPIDCtrl.reset_integral();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Yaw PID gains set to: kp = %f, ki = %f, kd = %f", request->kp, request->ki, request->kd);
}
void PIDControlNode::update_gains_yawRate_callback(const std::shared_ptr<glassy_interfaces::srv::PidGains::Request> request, std::shared_ptr<glassy_interfaces::srv::PidGains::Response> response){
    (void) response;
    this->yawRatePIDCtrl.set_gains(request->kp, request->ki, request->kd);
    this->yawRatePIDCtrl.reset_integral();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Yaw Rate PID gains set to: kp = %f, ki = %f, kd = %f", request->kp, request->ki, request->kd);
}
void PIDControlNode::activate_deactivate_srv_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, const std::shared_ptr<std_srvs::srv::SetBool::Response> response ){
    (void) response;
    if(request->data){
        this->activate();
        std::cout<<"STARTED INNER LOOP SUCCESSFULLY"<<std::endl;
    } else{
        this->deactivate();
        std::cout<< "PATH PLANNING STARTED SUCCESSFULLY"<<std::endl;
        // publish no 0 actuators
        this->direct_actuator_msg.thrust = 0.0;
        this->direct_actuator_msg.rudder = 0.0;

        this->direct_actuator_msg.header.stamp =  this->pid_glassy_node->get_clock()->now();
        this->actuator_publisher->publish(this->direct_actuator_msg);
    }
}


void PIDControlNode::update_integral_limits_surge_callback(const std::shared_ptr<glassy_interfaces::srv::SetLimits::Request> request, std::shared_ptr<glassy_interfaces::srv::SetLimits::Response> response){
    (void) response;
    this->surgePIDCtrl.set_integral_max_min(request->max_value, request->min_value);
}
void PIDControlNode::update_integral_limits_yaw_callback(const std::shared_ptr<glassy_interfaces::srv::SetLimits::Request> request, std::shared_ptr<glassy_interfaces::srv::SetLimits::Response> response){
    (void) response;
    this->yawPIDCtrl.set_integral_max_min(request->max_value, request->min_value);
}
void PIDControlNode::update_integral_limits_yawRate_callback(const std::shared_ptr<glassy_interfaces::srv::SetLimits::Request> request, std::shared_ptr<glassy_interfaces::srv::SetLimits::Response> response){
    (void) response;
    this->yawRatePIDCtrl.set_integral_max_min(request->max_value, request->min_value);
}



/*-----------------------------------
    Activate and deactivate logic
------------------------------------*/


void PIDControlNode::activate(){
    this->yawPIDCtrl.full_reset();
    this->yawRatePIDCtrl.full_reset();
    this->surgePIDCtrl.full_reset();

    std::cout<< "STARTED INNER LOOP SUCCESSFULLY"<< std::endl;

    this->is_active=true;
}
void PIDControlNode::deactivate(){
    this->is_active=false;
    this->yawPIDCtrl.full_reset();
    this->yawRatePIDCtrl.full_reset();
    this->surgePIDCtrl.full_reset();
}


/*---------------------------------
      Initialization Part
---------------------------------*/

// for now a simple initialization, parameters may be added in the future
void PIDControlNode::init(){


    

    /* -----------------------------
        Parameter Initialization
    -------------------------------*/

    // // Declare all of the parameters 
    // this->pid_glassy_node->declare_parameter("mapping.axes.thrust", 1);

    // // Initialize all the parameters
    // this->thrust_mapping = this->pid_glassy_node->get_parameter("mapping.axes.thrust").as_int();

    /* -----------------------------
        Variable Initialization
    -------------------------------*/

    // this->thrust_gain = this->pid_glassy_node->get_parameter("thrust_gain").as_double();

    this->surgePIDCtrl.set_gains(1.f,0.1f,0);
    this->yawPIDCtrl.set_gains(1.f,0.01f,0.1f);



    
    /* -----------------------------
        ROS2 Service Initialization
    -------------------------------*/


    // subscribe to the reference topic
    this->ref_subscription = this->pid_glassy_node->create_subscription<glassy_interfaces::msg::InnerLoopReferences>("inner_loop_ref", 1, std::bind(&PIDControlNode::referrence_subscription_callback, this, _1));

    // subscribe to the joystick topic
    this->state_subscription = this->pid_glassy_node->create_subscription<glassy_interfaces::msg::State>("state_vehicle", 1, std::bind(&PIDControlNode::state_subscription_callback, this, _1));

    // initialize publisher
    this->actuator_publisher = this->pid_glassy_node->create_publisher<glassy_interfaces::msg::OffboardDirectControl>("offboard_direct_signals", 1);

    this->timer = this->pid_glassy_node->create_wall_timer(50ms, std::bind(&PIDControlNode::direct_actuator_publish, this));

    // Services
    this->change_gains_surge = this->pid_glassy_node->create_service<glassy_interfaces::srv::PidGains>("pid_gains_surge", std::bind(&PIDControlNode::update_gains_surge_callback, this, _1, _2));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service Ready...");
    this->change_gains_yaw = this->pid_glassy_node->create_service<glassy_interfaces::srv::PidGains>("pid_gains_yaw", std::bind(&PIDControlNode::update_gains_yaw_callback, this, _1, _2));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service Ready...");
    this->change_gains_yawRate = this->pid_glassy_node->create_service<glassy_interfaces::srv::PidGains>("pid_gains_yaw_rate", std::bind(&PIDControlNode::update_gains_yawRate_callback, this, _1, _2));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service Ready...");
    this->activate_deactivate_inner_loop = this->pid_glassy_node->create_service<std_srvs::srv::SetBool>("activate_deactivate_innerloop", std::bind(&PIDControlNode::activate_deactivate_srv_callback, this, _1, _2));

    this->change_integral_limits_surge = this->pid_glassy_node->create_service<glassy_interfaces::srv::SetLimits>("pid_integral_limits_surge", std::bind(&PIDControlNode::update_integral_limits_surge_callback, this, _1, _2));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service Ready...");


    this->change_integral_limits_yaw = this->pid_glassy_node->create_service<glassy_interfaces::srv::SetLimits>("pid_integral_limits_yaw", std::bind(&PIDControlNode::update_integral_limits_yaw_callback, this, _1, _2));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service Ready...");


    this->change_integral_limits_yawRate = this->pid_glassy_node->create_service<glassy_interfaces::srv::SetLimits>("pid_integral_limits_yaw_rate", std::bind(&PIDControlNode::update_integral_limits_yawRate_callback, this, _1, _2));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service Ready...");
   
   
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Inner Loops ready...");
}
