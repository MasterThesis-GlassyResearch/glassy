#ifndef _PathManagementNode_
#define _PathManagementNode_

#include <iostream>
#include <chrono>
#include <thread>
#include <future>

#include "glassy_interfaces/msg/path_references.hpp"
#include "glassy_interfaces/msg/state.hpp"
#include "glassy_interfaces/srv/set_path.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "rclcpp/rclcpp.hpp"

#include "../PathTypes/Line.h"
#include "../PathTypes/Arc.h"
#include "../PathTypes/PathBase.h"
#include "eigen3/Eigen/Core"
#include <fstream>

class PathManagementNode
{
private:
    // service and subscriber callbacks

    // values sent directly to actuators if correct mode is set

    void ComputePathPointProperties(){};

    Eigen::Vector2d current_pose;

    std::string path_file_directory = "/home/joaolehodey/glassy_ws/src/glassy_planning/glassy_pathplanning/PathExamples/";

    bool correct_home_position();


    bool path_is_set=false;

    bool is_active=false;

   float x=0.0;
   float x_ref=0.0;
   float y=0.0;
   float y_ref=0.0;

   float lat = 0.0;
   float lon = 0.0;

   float home_lat = 38.766144;
   float home_lon = -9.093334;


   float x_correction=0.0;
   float y_correction=0.0;





   float max_surge = 5;
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
    void setPath();

    std::shared_ptr<rclcpp::Node> pathmanagement_node;


    // subscribe to state
    rclcpp::Subscription<glassy_interfaces::msg::State>::SharedPtr state_subscription;
    void state_subscription_callback(const glassy_interfaces::msg::State::SharedPtr msg);


    glassy_interfaces::msg::PathReferences pathref_msg;

    // publish directly to actuators
    rclcpp::Publisher<glassy_interfaces::msg::PathReferences>::SharedPtr path_publisher;

    //timer
    rclcpp::TimerBase::SharedPtr timer;

    // services...
    void activate_deactivate_srv_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr activate_deactivate_pathplanning;

    void set_path_srv_callback(const std::shared_ptr<glassy_interfaces::srv::SetPath::Request> request, std::shared_ptr<glassy_interfaces::srv::SetPath::Response> response);
    rclcpp::Service<glassy_interfaces::srv::SetPath>::SharedPtr set_path_srv;


    // clients ...
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr activate_deactivate_pathfollowing_client;




    void init();


    /*
        Activation Logic
    */
    void activate();
    void deactivate();
};

#endif