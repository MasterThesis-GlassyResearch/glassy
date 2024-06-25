#ifndef _PIDControlNode_
#define _PIDControlNode_

#include <iostream>
#include <unistd.h>
#include <stdlib.h>
#include <vector>
#include <algorithm>

#include "./../control_lib/PidController.h"

#include "glassy_msgs/msg/actuators.hpp"
#include "glassy_msgs/msg/inner_loop_references.hpp"
#include "glassy_msgs/msg/state.hpp"
#include "glassy_msgs/srv/pid_gains.hpp"
#include "glassy_msgs/srv/set_limits.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "rclcpp/rclcpp.hpp"
#include "MissionTypesInnerLoop.h"
#include <glassy_utils/GlassyGeneralUtils.h>


using namespace std::chrono_literals;
using namespace std::placeholders;



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

    // define control type (either surge-yaw or surge-yawrate)
    controlType ctrlType = SURGE_YAW;

    // define the current state
    float surge = 0.0;
    float yaw = 0.0;
    float yawRate = 0.0;
    float sway = 0.0;

    // define the parameters of the dynamical model :)
    float surgeParams[10] = { 1.1965, -0.6218, -0.0216, 0.0164,  0.0976,  0.5056, 335.2551, -0.1154, -0.0025, -0.2088};
    float yawRateParams[7] = { 2.1225, -0.8592, -0.0963,  2.2910, -0.0055, -1.9001,  0.0123};
    float angle_params[2] = { 0.1000, -0.6986};
    float epsilon_cnst = 10e-12;


    // define the mission info
    int mission_type = glassy_msgs::msg::MissionInfo::MISSION_OFF;

    // define the PID controllers for each one of the variables to control
    PidController surgePIDCtrl;
    PidController yawPIDCtrl;
    PidController yawRatePIDCtrl;


public:

    PIDControlNode(std::shared_ptr<rclcpp::Node> node);
    ~PIDControlNode(){};


    std::shared_ptr<rclcpp::Node> pid_glassy_node;


    // subscribe to topic comming from references (outer loop)
    rclcpp::Subscription<glassy_msgs::msg::InnerLoopReferences>::SharedPtr ref_subscription;
    void referrence_subscription_callback(const glassy_msgs::msg::InnerLoopReferences::SharedPtr msg);

    // subscribe to the mission info
    rclcpp::Subscription<glassy_msgs::msg::MissionInfo>::SharedPtr mission_info_subscription;
    void mission_info_subscription_callback(const glassy_msgs::msg::MissionInfo::SharedPtr msg);

    // subscribe to state
    rclcpp::Subscription<glassy_msgs::msg::State>::SharedPtr state_subscription;
    void state_subscription_callback(const glassy_msgs::msg::State::SharedPtr msg);
    
    rclcpp::Time prev_time;
    


    glassy_msgs::msg::Actuators direct_actuator_msg;

    // publish directly to actuators
    rclcpp::Publisher<glassy_msgs::msg::Actuators>::SharedPtr actuator_publisher;

    //timer
    rclcpp::TimerBase::SharedPtr timer;

    // Services 
    rclcpp::Service<glassy_msgs::srv::PidGains>::SharedPtr change_gains_surge;
    rclcpp::Service<glassy_msgs::srv::PidGains>::SharedPtr change_gains_yaw;
    rclcpp::Service<glassy_msgs::srv::PidGains>::SharedPtr change_gains_yawRate;
    rclcpp::Service<glassy_msgs::srv::SetLimits>::SharedPtr change_integral_limits_surge;
    rclcpp::Service<glassy_msgs::srv::SetLimits>::SharedPtr change_integral_limits_yaw;
    rclcpp::Service<glassy_msgs::srv::SetLimits>::SharedPtr change_integral_limits_yawRate;
    

    // service callbacks
    void update_gains_surge_callback(const std::shared_ptr<glassy_msgs::srv::PidGains::Request> request, std::shared_ptr<glassy_msgs::srv::PidGains::Response> response);
    void update_gains_yaw_callback(const std::shared_ptr<glassy_msgs::srv::PidGains::Request> request, std::shared_ptr<glassy_msgs::srv::PidGains::Response> response);
    void update_gains_yawRate_callback(const std::shared_ptr<glassy_msgs::srv::PidGains::Request> request, std::shared_ptr<glassy_msgs::srv::PidGains::Response> response);
    void update_integral_limits_surge_callback(const std::shared_ptr<glassy_msgs::srv::SetLimits::Request> request, std::shared_ptr<glassy_msgs::srv::SetLimits::Response> response);
    void update_integral_limits_yaw_callback(const std::shared_ptr<glassy_msgs::srv::SetLimits::Request> request, std::shared_ptr<glassy_msgs::srv::SetLimits::Response> response);
    void update_integral_limits_yawRate_callback(const std::shared_ptr<glassy_msgs::srv::SetLimits::Request> request, std::shared_ptr<glassy_msgs::srv::SetLimits::Response> response);

    void init();


    /*-----------------------------------
        Activate and deactivate logic
    ------------------------------------*/

    bool is_active=false;
    void activate();
    void deactivate();

};

#endif