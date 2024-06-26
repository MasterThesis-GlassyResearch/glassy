#ifndef _PathFollowingNode_
#define _PathFollowingNode_

#include <iostream>
#include <chrono>
#include <thread>
#include <future>
#include "../control_lib/LOSouterloop.h"

#include "MissionTypesOuterLoop.h"
#include "glassy_msgs/msg/inner_loop_references.hpp"
#include "glassy_msgs/msg/path_references.hpp"
#include "glassy_msgs/srv/los_params.hpp"
#include "glassy_msgs/msg/state.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "glassy_msgs/msg/mission_info.hpp"
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

    float surge_ref = 0.0;

    bool is_active = false;

    float max_surge = 5;
    float min_surge = 0;

    float max_yawrate = 1;
    float min_yawrate = -1;

    float speed = 2;

    void ref_publish();

    // define the mission info
    int mission_type = glassy_msgs::msg::MissionInfo::MISSION_OFF;

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

    // Timer ROS2
    rclcpp::TimerBase::SharedPtr timer;

    // Services ROS2
    void activate_deactivate_srv_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr activate_deactivate_pathfollowing;
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