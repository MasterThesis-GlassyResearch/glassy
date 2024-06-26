#include "rclcpp/rclcpp.hpp"
#include "PathFollowingNode.h"
#include "glassy_msgs/msg/inner_loop_references.hpp"
#include "glassy_msgs/msg/state.hpp"
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





void PathFollowingNode::path_subscription_callback(const glassy_msgs::msg::PathReferences::SharedPtr msg){
    // Set the correct references to track...
    this->pose_ref(0) = msg->x_ref;
    this->pose_ref(1) = msg->y_ref;
    this->tangent_heading = msg->tangent_heading;
    this->surge_ref = msg->path_vel;
    this->is_active = msg->is_active;

}

void PathFollowingNode::state_subscription_callback(const glassy_msgs::msg::State::SharedPtr msg){
    this->pose(0) = msg->p_ned[0];
    this->pose(1) = msg->p_ned[1];
}

/**
 * @brief Callback function for the mission info subscription
 *
 * @param msg 
 */
void PathFollowingNode::mission_info_subscription_callback(const glassy_msgs::msg::MissionInfo::SharedPtr msg){
    if(this->is_active && this->mission_type != msg->mission_mode){
        this->deactivate();
        this->mission_type = msg->mission_mode;
    } else{
        if(std::find(MissionTypesOuterLoop.begin(), MissionTypesOuterLoop.end(), msg->mission_mode) != MissionTypesOuterLoop.end()){
            this->activate();
        }
        this->mission_type = msg->mission_mode;
    }
}


void PathFollowingNode::setLOSParams_callback(const std::shared_ptr<glassy_msgs::srv::LosParams::Request> request, std::shared_ptr<glassy_msgs::srv::LosParams::Response> response){

    float look_ahead = request->look_ahead_dist;
    float sigma = request->sigma;

    response->result = this->LOSPathFollowing.set_params(look_ahead, sigma);
}


void PathFollowingNode::activate(){
    this->is_active=true;
    this->LOSPathFollowing.reset_integrator();
}
void PathFollowingNode::deactivate(){
    this->is_active=false;
    this->LOSPathFollowing.reset_integrator();
}





// for now a simple initialization, parameters may be added in the future
void PathFollowingNode::init(){


    
    /* -----------------------------
        ROS2 Service Initialization
    -------------------------------*/
    float look_ahead = 5;
    float sigma = 0;

    this->LOSPathFollowing.set_params(look_ahead, sigma);

    // subscribe to the reference topic
    this->path_subscription = this->pathfollowing_node->create_subscription<glassy_msgs::msg::PathReferences>("/glassy/path_refs", 1, std::bind(&PathFollowingNode::path_subscription_callback, this, _1));

    //subscribe to the mission info
    this->mission_info_subscription = this->pathfollowing_node->create_subscription<glassy_msgs::msg::MissionInfo>("/glassy/mission_status", 1, std::bind(&PathFollowingNode::mission_info_subscription_callback, this, _1));

    // subscribe to the joystick topic
    this->state_subscription = this->pathfollowing_node->create_subscription<glassy_msgs::msg::State>("/glassy/state", 1, std::bind(&PathFollowingNode::state_subscription_callback, this, _1));

    // initialize publisher
    this->reference_publisher = this->pathfollowing_node->create_publisher<glassy_msgs::msg::InnerLoopReferences>("/glassy/innerloop_refs", 1);

    this->timer = this->pathfollowing_node->create_wall_timer(100ms, std::bind(&PathFollowingNode::ref_publish, this));


    // Initialize the services
    this->setLOSParams = this->pathfollowing_node->create_service<glassy_msgs::srv::LosParams>("set_LOS_params", std::bind(&PathFollowingNode::setLOSParams_callback, this, _1, _2));

    // Initialize the clients
    this->activate_deactivate_innerloop_client = this->pathfollowing_node->create_client<std_srvs::srv::SetBool>("activate_deactivate_innerloop");

    std::cout<<"initialized correctly..."<<std::endl;
}
