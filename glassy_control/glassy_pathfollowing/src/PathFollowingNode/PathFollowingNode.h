#ifndef _PathFollowingNode_
#define _PathFollowingNode_

#include <iostream>
#include <chrono>
#include <thread>
#include <future>
#include <string>
#include <stdlib.h>

#include "../control_lib/LOSouterloop.h"
#include "../control_lib/LOSouterloopYawRate.h"
#include "../control_lib/VanniOuterLoop.h"

#include "MissionTypesOuterLoop.h"
#include "glassy_msgs/msg/inner_loop_references.hpp"
#include "glassy_msgs/msg/path_references.hpp"
#include "glassy_msgs/srv/los_params.hpp"
#include "glassy_msgs/msg/state.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "glassy_msgs/msg/mission_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include <eigen3/Eigen/Core>
#include <std_msgs/msg/float64.hpp>

class PathFollowingNode
{
private:

    // a controller of each type, the choice of which to use is done later
    std::string controller_type = "LOS";
    LOSouterloop LOSPathFollowing;
    LOSouterloopYawRate LOSPathFollowingYawRate;
    VanniOuterLoop VanniPathFollowing;

    Eigen::Vector2d pose;
    Eigen::Vector2d pose_ref;

    Eigen::Vector2d p_deriv;
    Eigen::Vector2d p_2nd_deriv;

    float yaw;
    float surge_ref = 0.0;

    bool is_active = false;

    float speed = 1;

    void runController();

    long long int last_time_publishing_nanosecs = 0;

    // define the mission info
    int mission_type = glassy_msgs::msg::MissionInfo::PATH_FOLLOWING;

public:
    PathFollowingNode(std::shared_ptr<rclcpp::Node> node);
    ~PathFollowingNode(){};

    std::shared_ptr<rclcpp::Node> pathfollowing_node;

    // subscribe to topic comming from references (outer loop)
    rclcpp::Subscription<glassy_msgs::msg::PathReferences>::SharedPtr path_subscription;
    void path_subscription_callback(const glassy_msgs::msg::PathReferences::SharedPtr msg);

    // subscribe to the mission info
    rclcpp::Subscription<glassy_msgs::msg::MissionInfo>::SharedPtr mission_info_subscription;
    void mission_info_subscription_callback(const glassy_msgs::msg::MissionInfo::SharedPtr msg);

    // subscribe to state
    rclcpp::Subscription<glassy_msgs::msg::State>::SharedPtr state_subscription;
    void state_subscription_callback(const glassy_msgs::msg::State::SharedPtr msg);

    glassy_msgs::msg::InnerLoopReferences inner_loop_ref_msg;

    // Publishers ROS2
    rclcpp::Publisher<glassy_msgs::msg::InnerLoopReferences>::SharedPtr reference_publisher;

    // gamma parameter publisher in case of a virtual target path following
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gamma_publisher;

    // Timer ROS2
    rclcpp::TimerBase::SharedPtr timer;

    // Services ROS2
    rclcpp::Service<glassy_msgs::srv::LosParams>::SharedPtr setLOSParams;
    void setLOSParams_callback(const std::shared_ptr<glassy_msgs::srv::LosParams::Request> request, std::shared_ptr<glassy_msgs::srv::LosParams::Response> response);

    // Clients ROS2
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr activate_deactivate_innerloop_client;

    // Initialization Logic 
    void init();

    /*
        Activation Logic
    */
    void activate();
    void deactivate();
};

#endif