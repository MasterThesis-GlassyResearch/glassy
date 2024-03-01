#include "rclcpp/rclcpp.hpp"
#include "PathFollowingNode.h"
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
PathFollowingNode::PathFollowingNode(std::shared_ptr<rclcpp::Node> node): pathfollowing_node(node)
{
    std::cout<<"Creating Path Following Controller Node...\n";
}



void PathFollowingNode::ref_publish(){

    
    this->inner_loop_ref_msg.yaw_ref = 0.0;
    this->inner_loop_ref_msg.surge_ref = 0.0;
    this->inner_loop_ref_msg.yaw_rate_ref = 0.0;

    std::vector<float> res_los = this->LOSPathFollowing.computeOutput(this->pose_ref, this->pose,this->tangent_heading, this->speed);

    this->inner_loop_ref_msg.header.stamp = this->pathfollowing_node->get_clock()->now();

    //TODO make sure we can select other path followings...
    // load the message
    this->inner_loop_ref_msg.surge_ref= res_los[0];
    this->inner_loop_ref_msg.yaw_ref= res_los[1];
    this->inner_loop_ref_msg.yaw_rate_ref = 0.0;


    // publish message
    this->reference_publisher->publish(this->inner_loop_ref_msg);
}   





void PathFollowingNode::path_subscription_callback(const glassy_interfaces::msg::Pathreferences::SharedPtr msg){
    // Set the correct references to track...
    this->pose_ref(0) = msg->x_ref;
    this->pose_ref(1) = msg->y_ref;
    this->tangent_heading = msg->tangent_heading;
;


}

void PathFollowingNode::state_subscription_callback(const glassy_interfaces::msg::State::SharedPtr msg){
    this->pose(0) = msg->north;
    this->pose(1) = msg->east;
}


// for now a simple initialization, parameters may be added in the future
void PathFollowingNode::init(){


    
    /* -----------------------------
        ROS2 Service Initialization
    -------------------------------*/


    // subscribe to the reference topic
    this->path_subscription = this->pathfollowing_node->create_subscription<glassy_interfaces::msg::Pathreferences>("path_refs", 1, std::bind(&PathFollowingNode::path_subscription_callback, this, _1));

    // subscribe to the joystick topic
    this->state_subscription = this->pathfollowing_node->create_subscription<glassy_interfaces::msg::State>("state_vehicle", 1, std::bind(&PathFollowingNode::state_subscription_callback, this, _1));

    // initialize publisher
    this->reference_publisher = this->pathfollowing_node->create_publisher<glassy_interfaces::msg::Innerloopreferences>("inner_loop_ref", 1);

    this->timer = this->pathfollowing_node->create_wall_timer(100ms, std::bind(&PathFollowingNode::ref_publish, this));

    std::cout<<"initialized correctly..."<<std::endl;
}
