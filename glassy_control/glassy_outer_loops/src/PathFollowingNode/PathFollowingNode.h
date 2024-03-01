#ifndef _PathFollowingNode_
#define _PathFollowingNode_

#include <iostream>
#include <chrono>
#include <thread>
#include <future>
#include "../control_lib/LOSouterloop.h"

#include "glassy_interfaces/msg/innerloopreferences.hpp"
#include "glassy_interfaces/msg/pathreferences.hpp"
#include "glassy_interfaces/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include <eigen3/Eigen/Core>



class PathFollowingNode
{
private:
    // service and subscriber callbacks

    // values sent directly to actuators if correct mode is set


   LOSouterloop LOSPathFollowing;

    Eigen::Vector2d pose;
    Eigen::Vector2d pose_ref;
    float tangent_heading;

   float max_surge = 5;
   float min_surge = 0;

   float max_yawrate = 1;
   float min_yawrate = -1;

   float speed = 2;

   void ref_publish();

    


public:

    PathFollowingNode(std::shared_ptr<rclcpp::Node> node);
    ~PathFollowingNode(){};

    std::shared_ptr<rclcpp::Node> pathfollowing_node;


    // subscribe to topic comming from references (outer loop)
    rclcpp::Subscription<glassy_interfaces::msg::Pathreferences>::SharedPtr path_subscription;
    void path_subscription_callback(const glassy_interfaces::msg::Pathreferences::SharedPtr msg);

    // subscribe to state
    rclcpp::Subscription<glassy_interfaces::msg::State>::SharedPtr state_subscription;
    void state_subscription_callback(const glassy_interfaces::msg::State::SharedPtr msg);


    
    

    glassy_interfaces::msg::Innerloopreferences inner_loop_ref_msg;

    // publish directly to actuators
    rclcpp::Publisher<glassy_interfaces::msg::Innerloopreferences>::SharedPtr reference_publisher;

    //timer
    rclcpp::TimerBase::SharedPtr timer;

    void init();

};

#endif