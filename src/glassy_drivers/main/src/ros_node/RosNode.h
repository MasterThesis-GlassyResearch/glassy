#ifndef _RosNode_
#define _RosNode_

#include <MavsdkNode.h>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <future>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "vehicle_interfaces/srv/arm.hpp"    
#include "vehicle_interfaces/msg/test.hpp"    
#include "vehicle_interfaces/msg/globalpos.hpp"    
#include "vehicle_interfaces/msg/nedpos.hpp"    


class MavsdkNode;
class RosNode
{
private:
    // service and subscriber callbacks
    void arm_disarm(const std::shared_ptr<vehicle_interfaces::srv::Arm::Request> request, std::shared_ptr<vehicle_interfaces::srv::Arm::Response> response);


    
public:
    RosNode();
    ~RosNode(){ };
    void init();

    MavsdkNode* mav_node;   //consider adding to private variables, using then public function to set it 
    std::shared_ptr<rclcpp::Node> ros_node;   //consider adding to private variables, using then public function to set it 
    

    // publishers
    rclcpp::Publisher<vehicle_interfaces::msg::Globalpos>::SharedPtr global_position_publisher;
    rclcpp::Publisher<vehicle_interfaces::msg::Nedpos>::SharedPtr ned_position_publisher;
    rclcpp::Publisher<vehicle_interfaces::msg::Test>::SharedPtr attitude_publisher;
};

#endif