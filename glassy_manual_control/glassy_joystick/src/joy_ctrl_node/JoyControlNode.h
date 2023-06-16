#ifndef _RosNode_
#define _RosNode_

#include <iostream>
#include <chrono>
#include <thread>
#include <future>
#include "rclcpp/rclcpp.hpp"
#include "glassy_interfaces/msg/offboarddirectcontrol.hpp"
#include "glassy_interfaces/srv/arm.hpp"
#include "sensor_msgs/msg/joy.hpp"


enum mode{FULL_CONTROL, RUDDER_ONLY, THRUST_ONLY, NO_CONTROL};
class JoyControlNode
{
private:
    // service and subscriber callbacks

    // values sent directly to actuators if correct mode is set
    float rudder_value = 0;
    float thrust_value = 0;  

    float rudder_increments = 0.1;
    float thrust_increments = 0.1;

    void direct_actuator_publish();
    void change_mode(mode new_mode);

    // define buttons and axis needed
    int thrust_axis_index = 1;
    int rudder_axis_index = 2;
    int arm_button_index = 10;
    int disarm_button_index = 9;

    // parameters that define the mapping between functions and actuations
    int thrust_mapping;
    int steering_mapping; 
    int arm_mapping;        
    int disarm_mapping;
    int start_offboard_mapping;
    int stop_offboard_mapping;
    std::vector<long int> const_val_increase_mapping{0,7};
    std::vector<long int> const_val_decrease_mapping{0,7};
    std::vector<long int> step_increase_mapping{0,6};
    std::vector<long int> step_decrease_mapping{0,6};

    mode joystick_mode = FULL_CONTROL;


    // values initialized so that they are not 

public:

    JoyControlNode(std::shared_ptr<rclcpp::Node> node);
    ~JoyControlNode(){};

    std::shared_ptr<rclcpp::Node> joy_glassy_node;


    // subscribe to topic comming from joystick
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_input_subscription;
    void joystick_subscription_callback(const sensor_msgs::msg::Joy::SharedPtr msg);


    glassy_interfaces::msg::Offboarddirectcontrol direct_actuator_msg;
    glassy_interfaces::srv::Arm arm_disarm_svr_msg;

    // publish directly to actuators
    rclcpp::Publisher<glassy_interfaces::msg::Offboarddirectcontrol>::SharedPtr glassy_interface_publisher;

    //client, will intercact with the arm/ disarm service
    rclcpp::Client<glassy_interfaces::srv::Arm>::SharedPtr arm_disarm_client;

    // create request variables for service
    std::shared_ptr<glassy_interfaces::srv::Arm::Request> arm_request = std::make_shared<glassy_interfaces::srv::Arm::Request>();
    std::shared_ptr<glassy_interfaces::srv::Arm::Request> offboard_request = std::make_shared<glassy_interfaces::srv::Arm::Request>();

    //client, will intercact with the start/stop offboard service
    rclcpp::Client<glassy_interfaces::srv::Arm>::SharedPtr start_stop_offboard_client;

    void init();


    // if false, other sources can publish, this avoids conflicts
    bool direct_actuator_publishing;

};

#endif