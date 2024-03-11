#ifndef _PIDControlNode_
#define _PIDControlNode_

#include <iostream>
#include <chrono>
#include <thread>
#include <future>
#include "../control_lib/PidController.h"

#include "glassy_interfaces/msg/offboard_direct_control.hpp"
#include "glassy_interfaces/msg/inner_loop_references.hpp"
#include "glassy_interfaces/msg/state.hpp"
#include "glassy_interfaces/srv/pid_gains.hpp"
#include "rclcpp/rclcpp.hpp"



enum controlType{SURGE_YAW, SURGE_YAWRATE};

class PIDControlNode
{
private:
    // service and subscriber callbacks

    // values sent directly to actuators if correct mode is set
    
    void direct_actuator_publish();

    float max_integral_value;

    // define the references
    float surge_ref = 3;
    float yaw_ref = 0;
    float yawRate_ref = 0;
    controlType ctrlType = SURGE_YAW;

    // define the current state
    float surge = 0;
    float yaw = 0;
    float yawRate = 0;


   PidController surgePIDCtrl;
   PidController yawPIDCtrl;
   PidController yawRatePIDCtrl;

    


public:

    PIDControlNode(std::shared_ptr<rclcpp::Node> node);
    ~PIDControlNode(){};

    std::shared_ptr<rclcpp::Node> pid_glassy_node;


    // subscribe to topic comming from references (outer loop)
    rclcpp::Subscription<glassy_interfaces::msg::InnerLoopReferences>::SharedPtr ref_subscription;
    void referrence_subscription_callback(const glassy_interfaces::msg::InnerLoopReferences::SharedPtr msg);

    // subscribe to state
    rclcpp::Subscription<glassy_interfaces::msg::State>::SharedPtr state_subscription;
    void state_subscription_callback(const glassy_interfaces::msg::State::SharedPtr msg);
    
    rclcpp::Time prev_time;




    glassy_interfaces::msg::OffboardDirectControl direct_actuator_msg;

    // publish directly to actuators
    rclcpp::Publisher<glassy_interfaces::msg::OffboardDirectControl>::SharedPtr actuator_publisher;

    //timer
    rclcpp::TimerBase::SharedPtr timer;

    // Services 
    rclcpp::Service<glassy_interfaces::srv::PidGains>::SharedPtr change_gains_surge;
    rclcpp::Service<glassy_interfaces::srv::PidGains>::SharedPtr change_gains_yaw;
    rclcpp::Service<glassy_interfaces::srv::PidGains>::SharedPtr change_gains_yawRate;
    

    // service callbacks
    void update_gains_surge_callback(const std::shared_ptr<glassy_interfaces::srv::PidGains::Request> request, std::shared_ptr<glassy_interfaces::srv::PidGains::Response> response);
    void update_gains_yaw_callback(const std::shared_ptr<glassy_interfaces::srv::PidGains::Request> request, std::shared_ptr<glassy_interfaces::srv::PidGains::Response> response);
    void update_gains_yawRate_callback(const std::shared_ptr<glassy_interfaces::srv::PidGains::Request> request, std::shared_ptr<glassy_interfaces::srv::PidGains::Response> response);

    void init();

    // if false, other sources can publish, this avoids conflicts -> maybe //TODO
    bool direct_actuator_publishing;

};

#endif