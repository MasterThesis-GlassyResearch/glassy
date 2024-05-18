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
#include "std_srvs/srv/set_bool.hpp"
#include "rclcpp/rclcpp.hpp"
#include "MissionTypesPathManager.h"

#include "../PathTypes/Line.h"
#include "../PathTypes/Arc.h"
#include "../PathTypes/PathBase.h"
#include "eigen3/Eigen/Core"

class PathManagementNode
{
private:
    // service and subscriber callbacks

    // values sent directly to actuators if correct mode is set

    void ComputePathPointProperties(){};

    Eigen::Vector2d pose_ned;

    std::string path_file_directory = "/home/joaolehodey/glassy_ws/src/glassy_planning/glassy_pathplanning/PathExamples/";

    bool correct_home_position();


    bool path_is_set=false;

    bool is_active=false;

   float lat = 0.0;
   float lon = 0.0;

   float home_lat = 38.766144;
   float home_lon = -9.093334;

   float x_correction=0.0;
   float y_correction=0.0;

   bool loop=false;



   float max_surge = 12;
   float min_surge = 0;

   float max_yawrate = 1;
   float min_yawrate = -1;

   float speed = 2;

   void ref_publish();

    
    Line test_line1;
    Arc test_arc1;

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


    glassy_msgs::msg::PathReferences pathref_msg;

    // publish directly to actuators
    rclcpp::Publisher<glassy_msgs::msg::PathReferences>::SharedPtr path_publisher;

    //timer
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