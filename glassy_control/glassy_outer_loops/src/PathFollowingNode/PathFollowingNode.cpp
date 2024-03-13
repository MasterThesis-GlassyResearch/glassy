#include "rclcpp/rclcpp.hpp"
#include "PathFollowingNode.h"
#include "glassy_interfaces/msg/inner_loop_references.hpp"
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

    if(!this->is_active){
        this->inner_loop_ref_msg.yaw_ref = 0.0;
        this->inner_loop_ref_msg.surge_ref = 0.0;
        this->inner_loop_ref_msg.yaw_rate_ref = 0.0;


        // publish message
        this->reference_publisher->publish(this->inner_loop_ref_msg);
    }


    std::vector<float> res_los = this->LOSPathFollowing.computeOutput(this->pose_ref, this->pose,this->tangent_heading, this->speed);

    this->inner_loop_ref_msg.header.stamp = this->pathfollowing_node->get_clock()->now();

    // load the message
    this->inner_loop_ref_msg.surge_ref= this->surge_ref;
    this->inner_loop_ref_msg.yaw_ref= res_los[1];
    this->inner_loop_ref_msg.yaw_rate_ref = 0.0;


    // publish message
    this->reference_publisher->publish(this->inner_loop_ref_msg);
}   





void PathFollowingNode::path_subscription_callback(const glassy_interfaces::msg::PathReferences::SharedPtr msg){
    // Set the correct references to track...
    this->pose_ref(0) = msg->x_ref;
    this->pose_ref(1) = msg->y_ref;
    this->tangent_heading = msg->tangent_heading;
    this->surge_ref = msg->path_vel;
    this->is_active = msg->is_active;

}

void PathFollowingNode::state_subscription_callback(const glassy_interfaces::msg::State::SharedPtr msg){
    this->pose(0) = msg->north;
    this->pose(1) = msg->east;
}


void PathFollowingNode::activate_deactivate_srv_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response){
    (void) response;
    auto request_il = std::make_shared<std_srvs::srv::SetBool::Request>();
    if(request->data){
        this->activate();
        request_il->data = true;
        std::cout<<"STARTED PATH FOLLOWING NICELLY"<<std::endl;
    } else{
        request_il->data = false;
        this->deactivate();
    }
    this->activate_deactivate_innerloop_client->async_send_request(request_il);
}



void PathFollowingNode::activate(){
    this->is_active=true;
    //FIXME add integrator/ integrator reset
}
void PathFollowingNode::deactivate(){
    this->is_active=false;
}





// for now a simple initialization, parameters may be added in the future
void PathFollowingNode::init(){


    
    /* -----------------------------
        ROS2 Service Initialization
    -------------------------------*/


    // subscribe to the reference topic
    this->path_subscription = this->pathfollowing_node->create_subscription<glassy_interfaces::msg::PathReferences>("path_refs", 1, std::bind(&PathFollowingNode::path_subscription_callback, this, _1));

    // subscribe to the joystick topic
    this->state_subscription = this->pathfollowing_node->create_subscription<glassy_interfaces::msg::State>("state_vehicle", 1, std::bind(&PathFollowingNode::state_subscription_callback, this, _1));

    // initialize publisher
    this->reference_publisher = this->pathfollowing_node->create_publisher<glassy_interfaces::msg::InnerLoopReferences>("inner_loop_ref", 1);

    this->timer = this->pathfollowing_node->create_wall_timer(100ms, std::bind(&PathFollowingNode::ref_publish, this));


    // Initialize the services
    this->activate_deactivate_pathfollowing = this->pathfollowing_node->create_service<std_srvs::srv::SetBool>("activate_deactivate_path_following", std::bind(&PathFollowingNode::activate_deactivate_srv_callback, this, _1, _2));
    

    // Initialize the clients
    this->activate_deactivate_innerloop_client = this->pathfollowing_node->create_client<std_srvs::srv::SetBool>("activate_deactivate_innerloop");

    std::cout<<"initialized correctly..."<<std::endl;
}
