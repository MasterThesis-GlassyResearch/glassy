#ifndef _PathManagementNode_
#define _PathManagementNode_

#include <iostream>
#include <chrono>
#include <thread>
#include <future>
#include <fstream>
#include <algorithm>

#include "glassy_msgs/msg/path_references.hpp"
#include "glassy_msgs/msg/state.hpp"
#include "glassy_msgs/srv/set_path.hpp"
#include "glassy_msgs/msg/mission_info.hpp"
#include "glassy_msgs/msg/path_info.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "rclcpp/rclcpp.hpp"
#include "MissionTypesPathManager.h"
#include "std_msgs/msg/float64.hpp"

#include "../PathTypes/Line.h"
#include "../PathTypes/Arc.h"
#include "../PathTypes/PathBase.h"
#include "eigen3/Eigen/Core"

class PathManagementNode
{
private:


    void ComputePathPointProperties();

    Eigen::Vector2d pose_ned;

    std::string path_file_directory = "/home/joaolehodey/glassy_ws/src/glassy_planning/glassy_pathplanning/PathExamples/";

    bool correct_home_position();

    bool path_is_set = false;

    float lat = 0.0;
    float lon = 0.0;

    // simulation mode
    bool is_simulation = true;

    // home position if not in simulation
    float home_lat = 38.766144;
    float home_lon = -9.093334;

    // correction values applied to path in NED frame
    float x_correction = 0.0;
    float y_correction = 0.0;

    // if the path is a loop (if should restart)
    bool loop = false;

    // maximum and minimum surges that can be requested
    float max_surge = 12;
    float min_surge = 0;


    bool is_active = false;

    float rate = 10;

    void ref_publish();

    std::vector<std::shared_ptr<PathBase>> path_segments;
    std::vector<float> requested_surge;
    int path_index = 0;

public:
    PathManagementNode(std::shared_ptr<rclcpp::Node> node);
    ~PathManagementNode(){};

    void setPath(std::string file_location);

    std::shared_ptr<rclcpp::Node> pathmanagement_node;

    // subscribe to state
    rclcpp::Subscription<glassy_msgs::msg::State>::SharedPtr state_subscription;
    void state_subscription_callback(const glassy_msgs::msg::State::SharedPtr msg);

    // subscribe to the mission info
    rclcpp::Subscription<glassy_msgs::msg::MissionInfo>::SharedPtr mission_info_subscription;
    void mission_info_subscription_callback(const glassy_msgs::msg::MissionInfo::SharedPtr msg);

    // subscribe to the path
    rclcpp::Subscription<glassy_msgs::msg::PathInfo>::SharedPtr path_info_subscription;
    void path_info_subscription_callback(const glassy_msgs::msg::PathInfo::SharedPtr msg);

    // subscribe to gamma
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gamma_subscription;
    void gamma_subscription_callback(const std_msgs::msg::Float64::SharedPtr msg);

    glassy_msgs::msg::PathReferences pathref_msg;

    // publish directly to actuators
    rclcpp::Publisher<glassy_msgs::msg::PathReferences>::SharedPtr path_publisher;

    // timer
    rclcpp::TimerBase::SharedPtr timer;

    // services...
    void set_path_srv_callback(const std::shared_ptr<glassy_msgs::srv::SetPath::Request> request, std::shared_ptr<glassy_msgs::srv::SetPath::Response> response);
    rclcpp::Service<glassy_msgs::srv::SetPath>::SharedPtr set_path_srv;

    // initialize the node
    void init();
    int mission_type = glassy_msgs::msg::MissionInfo::MISSION_OFF;

    /*
        Activation Logic
    */
    void activate();
    void deactivate();
};

#endif