#include <MavsdkNode.h>
#include <RosNode.h>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <future>
#include "rclcpp/rclcpp.hpp"
#include "vehicle_interfaces/srv/arm.hpp"     
// #include "std_msgs/msg/string.hpp"

using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;
using namespace std::placeholders;



RosNode::RosNode()
{
    std::cout << "Creating RosNode ...\n";
}

void RosNode::arm_disarm(const std::shared_ptr<vehicle_interfaces::srv::Arm::Request> request, std::shared_ptr<vehicle_interfaces::srv::Arm::Response> response)
{
    (void) response;

    std::string mode;
    switch (request->mode)
    {
    case 1: mode = "ARMING"; break;
    case 0: mode = "DISARMING"; break;
    default: mode = "UNKNOWN"; break;
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request %s",
                mode.c_str());
    //to be finished

    this->mav_node->arm_disarm(request->mode);
}

void RosNode::init()
{
    //creating a node 
    this->ros_node = rclcpp::Node::make_shared("ros2_comunication_server");

    // binding arming and disarming service to the node
    rclcpp::Service<vehicle_interfaces::srv::Arm>::SharedPtr service =                 
        this->ros_node->create_service<vehicle_interfaces::srv::Arm>("arm_disarm", std::bind(&RosNode::arm_disarm, this, _1, _2)); 
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Arm Disarm Service Ready..."); 

    //setup publishers and subscribers...



    // RCLCPP_INFO("ROS2 comunication node should be ready..."); 
    rclcpp::spin(this->ros_node);
    rclcpp::shutdown();   //  ---------------------------------------------------check if this is needed
}
