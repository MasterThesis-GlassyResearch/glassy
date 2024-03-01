#include "rclcpp/rclcpp.hpp"
#include "PathManagementNode.h"
#include "glassy_interfaces/msg/offboarddirectcontrol.hpp"
#include "glassy_interfaces/msg/innerloopreferences.hpp"
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
PathManagementNode::PathManagementNode(std::shared_ptr<rclcpp::Node> node): pathmanagement_node(node), test_line1(Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(-10000, -10000)), current_pose(0.0,0.0), test_arc1(Eigen::Vector2d(20.0, 0.0), Eigen::Vector2d(20.0, 0.0), Eigen::Vector2d(0.0, 0.0))
{
    std::cout<<"Creating Path Management Controller Node...\n";

}


void setPath(std::string file_location){
    
};



void PathManagementNode::ref_publish(){

    // Eigen::Vector2d result = this->test_line1.getClosestPoint(this->current_pose);
    Eigen::Vector2d result = this->test_arc1.getClosestPoint(this->current_pose);
    
    this->pathref_msg.x_ref = result(0);
    this->pathref_msg.y_ref = result(1);

    // this->pathref_msg.tangent_heading = this->test_line1.getTangHeading();
    this->pathref_msg.tangent_heading = this->test_arc1.getTangHeading(this->current_pose);

    this->pathref_msg.header.stamp = this->pathmanagement_node->get_clock()->now();

    // publish message
    this->path_publisher->publish(this->pathref_msg);
}   




void PathManagementNode::state_subscription_callback(const glassy_interfaces::msg::State::SharedPtr msg){
    this->current_pose(0) = msg->north;
    this->current_pose(1) = msg->east;
}


// for now a simple initialization, parameters may be added in the future
void PathManagementNode::init(){


    
    /* -----------------------------
        ROS2 Service Initialization
    -------------------------------*/


    // subscribe to the joystick topic
    this->state_subscription = this->pathmanagement_node->create_subscription<glassy_interfaces::msg::State>("state_vehicle", 1, std::bind(&PathManagementNode::state_subscription_callback, this, _1));

    // initialize publisher
    this->path_publisher = this->pathmanagement_node->create_publisher<glassy_interfaces::msg::Pathreferences>("path_refs", 1);

    // initialize timer, -> dictates when to publish
    this->timer = this->pathmanagement_node->create_wall_timer(100ms, std::bind(&PathManagementNode::ref_publish, this));

}
