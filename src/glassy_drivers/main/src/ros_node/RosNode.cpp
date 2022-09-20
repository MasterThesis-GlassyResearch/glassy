#include <MavsdkNode.h>
#include <RosNode.h>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <future>
#include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"


using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;



RosNode::RosNode(){
    std::cout<< "Creating RosNode ...\n";
}


void RosNode::init(){
    this->ros_node = rclcpp::Node::make_shared("ros2_comunication_server");

    // rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service =
    // node->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", &add);

}