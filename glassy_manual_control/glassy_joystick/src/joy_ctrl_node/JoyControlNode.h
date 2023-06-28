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

    float increments_const_val;
    float increments_on_step;
    float max_increment_value;

    void direct_actuator_publish();
    void change_mode(mode new_mode);
    void keep_track_button_pressed(int* new_button, int new_val);



    // parameters that define the mapping between functions and actuations
    int thrust_mapping;
    int steering_mapping; 
    int arm_mapping;        
    int disarm_mapping;
    int start_offboard_mapping;
    int stop_offboard_mapping;
    int const_val_inc_dec_mapping;
    int step_inc_dec_mapping;
    int toogle_mode_mapping;
    int enter_mode_mapping;


    // keep track of wether button clicked now or still clicking from before
    int arm_previous_value;        
    int disarm_previous_value;
    int start_offboard_previous_value;
    int stop_offboard_previous_value;
    int const_val_inc_dec_previous_value;
    int step_inc_dec_previous_value;
    int toogle_mode_previous_value;
    int enter_mode_previous_value;

    int* last_pressed_btn;

    // whether the extra functions are buttons or axis
    int extra_type;

    mode joystick_mode = FULL_CONTROL;
    int next_mode_index = 0;

    std::vector<mode> list_modes{FULL_CONTROL, RUDDER_ONLY, THRUST_ONLY, NO_CONTROL};
    std::vector<std::string> list_modes_names{"FULL_CONTROL", "RUDDER_ONLY", "THRUST_ONLY", "NO_CONTROL"};
    int nmr_modes = 4;


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