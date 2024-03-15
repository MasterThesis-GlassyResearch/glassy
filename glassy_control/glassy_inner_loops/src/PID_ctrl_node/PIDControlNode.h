#ifndef _PIDControlNode_
#define _PIDControlNode_

#include <iostream>
#include <chrono>
#include <thread>
#include <future>
#include "./../control_lib/PidController.h"

#include "glassy_interfaces/msg/offboard_direct_control.hpp"
#include "glassy_interfaces/msg/inner_loop_references.hpp"
#include "glassy_interfaces/msg/state.hpp"
#include "glassy_interfaces/srv/pid_gains.hpp"
#include "glassy_interfaces/srv/set_limits.hpp"
#include "std_srvs/srv/set_bool.hpp"
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
    float surge_ref = 0;
    float yaw_ref = 0;
    float yawRate_ref = 0;
    controlType ctrlType = SURGE_YAW;

    // define the current state
    float surge = 0.0;
    float yaw = 0.0;
    float yawRate = 0.0;
    float sway = 0.0;

    float surgeParams[10] = { 1.1965, -0.6218, -0.0216, 0.0164,  0.0976,  0.5056, 335.2551, -0.1154, -0.0025, -0.2088};
    float yawRateParams[7] = { 2.1225, -0.8592, -0.0963,  2.2910, -0.0055, -1.9001,  0.0123};
    float angle_params[2] = { 0.1000, -0.6986};

    float epsilon_cnst = 10e-12;


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

    rclcpp::Service<glassy_interfaces::srv::SetLimits>::SharedPtr change_integral_limits_surge;
    rclcpp::Service<glassy_interfaces::srv::SetLimits>::SharedPtr change_integral_limits_yaw;
    rclcpp::Service<glassy_interfaces::srv::SetLimits>::SharedPtr change_integral_limits_yawRate;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr activate_deactivate_inner_loop;
    

    // service callbacks
    void update_gains_surge_callback(const std::shared_ptr<glassy_interfaces::srv::PidGains::Request> request, std::shared_ptr<glassy_interfaces::srv::PidGains::Response> response);
    void update_gains_yaw_callback(const std::shared_ptr<glassy_interfaces::srv::PidGains::Request> request, std::shared_ptr<glassy_interfaces::srv::PidGains::Response> response);
    void update_gains_yawRate_callback(const std::shared_ptr<glassy_interfaces::srv::PidGains::Request> request, std::shared_ptr<glassy_interfaces::srv::PidGains::Response> response);


    void update_integral_limits_surge_callback(const std::shared_ptr<glassy_interfaces::srv::SetLimits::Request> request, std::shared_ptr<glassy_interfaces::srv::SetLimits::Response> response);
    void update_integral_limits_yaw_callback(const std::shared_ptr<glassy_interfaces::srv::SetLimits::Request> request, std::shared_ptr<glassy_interfaces::srv::SetLimits::Response> response);
    void update_integral_limits_yawRate_callback(const std::shared_ptr<glassy_interfaces::srv::SetLimits::Request> request, std::shared_ptr<glassy_interfaces::srv::SetLimits::Response> response);


    void activate_deactivate_srv_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response);





    void init();



    /*-----------------------------------
        Activate and deactivate logic
    ------------------------------------*/

    bool is_active=false;
    void activate();
    void deactivate();

};

#endif