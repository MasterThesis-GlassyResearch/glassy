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
        return;
    }

    long long int current_time = this->pathfollowing_node->get_clock()->now().nanoseconds();
    double duration = double(current_time - this->last_time_publishing_nanosecs)/1e9;

    RCLCPP_DEBUG(this->pathfollowing_node->get_logger(), "Duration: %f", duration);
    RCLCPP_DEBUG(this->pathfollowing_node->get_logger(), "Current_time: %lld", current_time);
    RCLCPP_DEBUG(this->pathfollowing_node->get_logger(), "Prev time: %lld", this->last_time_publishing_nanosecs);

    // load the message
    this->inner_loop_ref_msg.surge_ref= 0.0;
    this->inner_loop_ref_msg.yaw_ref= 0.0;
    this->inner_loop_ref_msg.yaw_rate_ref = 0.0;

    if(this->controller_type=="LOS"){
        // compute the output of the LOS controller (surge, yaw, yaw_rate)
        std::vector<float> res_los = this->LOSPathFollowing.computeOutput(this->pose_ref, this->pose,this->tangent_heading, this->speed, duration);
        this->inner_loop_ref_msg.surge_ref= this->surge_ref;
        this->inner_loop_ref_msg.yaw_ref= res_los[1];
        this->inner_loop_ref_msg.ctrl_type = InnerLoopReferences::SURGE_YAW;
    } else if(this->controller_type=="LOS-r"){

        // compute the output of the LOS controller (surge, yaw, yaw_rate)
        std::vector<float> res_los_yr = this->LOSPathFollowingYawRate.computeOutput(this->pose_ref, this->pose, this->yaw ,this->tangent_heading, this->curvature, this->speed, duration);
        // load the message
        RCLCPP_INFO(this->pathfollowing_node->get_logger(), "Yaw Rate ref: %f", res_los_yr[1]);
        
        this->inner_loop_ref_msg.surge_ref= this->surge_ref;
        this->inner_loop_ref_msg.yaw_ref= 0.0;
        this->inner_loop_ref_msg.yaw_rate_ref = res_los_yr[1];
        this->inner_loop_ref_msg.ctrl_type = InnerLoopReferences::SURGE_YAW_RATE;
    }

    this->inner_loop_ref_msg.header.stamp = this->pathfollowing_node->get_clock()->now();



    // publish message
    this->reference_publisher->publish(this->inner_loop_ref_msg);
    this->last_time_publishing_nanosecs = current_time;
}   





void PathFollowingNode::path_subscription_callback(const glassy_msgs::msg::PathReferences::SharedPtr msg){
    // Set the correct references to track...
    this->pose_ref(0) = msg->x_ref;
    this->pose_ref(1) = msg->y_ref;
    this->tangent_heading = msg->tangent_heading;
    this->surge_ref = msg->path_vel;
    this->curvature = msg->curvature;
    // this->is_active = msg->is_active;

}

void PathFollowingNode::state_subscription_callback(const glassy_msgs::msg::State::SharedPtr msg){
    this->pose(0) = msg->p_ned[0];
    this->pose(1) = msg->p_ned[1];
    this->yaw = msg->yaw;
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
        if(!this->is_active && msg->mission_mode == MissionInfo::PATH_FOLLOWING){
            this->activate();
            this->mission_type = msg->mission_mode;
        }
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
        Get the required parameters
    -------------------------------*/
    this->pathfollowing_node->declare_parameter("controller_type", "LOS");
    this->pathfollowing_node->declare_parameter("LOS_gains.look_ahead", 5.0);
    this->pathfollowing_node->declare_parameter("LOS_gains.sigma", 0.0);
    this->pathfollowing_node->declare_parameter("LOS_gains.max_int", 5.0);
    this->pathfollowing_node->declare_parameter("rate", 20.0);
    this->pathfollowing_node->declare_parameter("LOS_yr_gains.k1", 10.0);
    this->pathfollowing_node->declare_parameter("LOS_yr_gains.k2", 10.0);


    float look_ahead = this->pathfollowing_node->get_parameter("LOS_gains.look_ahead").as_double();
    float sigma = this->pathfollowing_node->get_parameter("LOS_gains.sigma").as_double();
    float max_int = this->pathfollowing_node->get_parameter("LOS_gains.max_int").as_double();
    float rate = this->pathfollowing_node->get_parameter("rate").as_double();

    float k1 = this->pathfollowing_node->get_parameter("LOS_yr_gains.k1").as_double();
    float k2 = this->pathfollowing_node->get_parameter("LOS_yr_gains.k2").as_double();

    this->controller_type = this->pathfollowing_node->get_parameter("controller_type").as_string();


    // print all the parameters 
    RCLCPP_INFO(this->pathfollowing_node->get_logger(), "Controller Type: %s", this->controller_type.c_str());
    RCLCPP_INFO(this->pathfollowing_node->get_logger(), "LOS Look Ahead: %f", look_ahead);
    RCLCPP_INFO(this->pathfollowing_node->get_logger(), "LOS Sigma: %f", sigma);
    RCLCPP_INFO(this->pathfollowing_node->get_logger(), "LOS max int: %f", max_int);
    RCLCPP_INFO(this->pathfollowing_node->get_logger(), "OuterLoop raye: %f", rate);

    // prepare the LOS controller
    if(this->controller_type=="LOS"){
        this->LOSPathFollowing = LOSouterloop(look_ahead, sigma);
        this->LOSPathFollowing.set_max_int(max_int);
    }else if(this->controller_type=="LOS-r")
    {
        this->LOSPathFollowingYawRate = LOSouterloopYawRate(k1, k2);
    }


    // subscribe to the reference topic
    this->path_subscription = this->pathfollowing_node->create_subscription<glassy_msgs::msg::PathReferences>("/glassy/path_refs", 1, std::bind(&PathFollowingNode::path_subscription_callback, this, _1));

    //subscribe to the mission info
    this->mission_info_subscription = this->pathfollowing_node->create_subscription<glassy_msgs::msg::MissionInfo>("/glassy/mission_status", 1, std::bind(&PathFollowingNode::mission_info_subscription_callback, this, _1));

    // subscribe to the joystick topic
    this->state_subscription = this->pathfollowing_node->create_subscription<glassy_msgs::msg::State>("/glassy/state", 1, std::bind(&PathFollowingNode::state_subscription_callback, this, _1));

    // initialize publisher
    this->reference_publisher = this->pathfollowing_node->create_publisher<glassy_msgs::msg::InnerLoopReferences>("/glassy/innerloop_refs", 1);

    this->timer = this->pathfollowing_node->create_wall_timer(1.0s/rate, std::bind(&PathFollowingNode::ref_publish, this));


    // Initialize the services
    this->setLOSParams = this->pathfollowing_node->create_service<glassy_msgs::srv::LosParams>("set_LOS_params", std::bind(&PathFollowingNode::setLOSParams_callback, this, _1, _2));

    // Initialize the clients
    this->activate_deactivate_innerloop_client = this->pathfollowing_node->create_client<std_srvs::srv::SetBool>("activate_deactivate_innerloop");

    std::cout<<"initialized correctly..."<<std::endl;
}
