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

    
    std::vector<int> thrust_mapping{0,1};
    std::vector<int> steering_mapping{0,2};
    std::vector<int> arm_mapping{1, 0};
    std::vector<int> disarm_mapping{1, 2};
    std::vector<int> start_offboard_mapping{1,3};
    std::vector<int> stop_offboard_mapping{1,4};
    std::vector<int> const_val_increase_mapping{0,7};
    std::vector<int> const_val_decrease_mapping{0,7};
    std::vector<int> step_increase_mapping{0,6};
    std::vector<int> step_decrease_mapping{0,6};

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

    void init();

    mode joystick_mode = FULL_CONTROL;


    // if false, other sources can publish, this avoids conflicts
    bool direct_actuator_publishing;


};

#endif