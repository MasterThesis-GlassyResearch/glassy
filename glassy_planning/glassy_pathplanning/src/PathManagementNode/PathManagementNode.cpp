#include "rclcpp/rclcpp.hpp"
#include "PathManagementNode.h"
#include "glassy_interfaces/msg/offboarddirectcontrol.hpp"
#include "glassy_interfaces/msg/innerloopreferences.hpp"
#include "glassy_interfaces/msg/state.hpp"
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <future>
#include <stdlib.h>
#include <fstream>
#include <string>



using namespace std::chrono_literals;
using namespace std::placeholders;

// class constructor 
PathManagementNode::PathManagementNode(std::shared_ptr<rclcpp::Node> node): pathmanagement_node(node), test_line1(Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(-10000, -10000)), current_pose(0.0,0.0), test_arc1(Eigen::Vector2d(20.0, 0.0), Eigen::Vector2d(40.0, 0.0), M_PI)
{
    std::cout<<"Creating Path Management Controller Node...\n";

}


void PathManagementNode::setPath(std::string file_location){
  std::ifstream myfile(file_location);
  std::string line;
  if (myfile.is_open())
  {
    while(getline(myfile, line)) {
      std::cout << line << std::endl;
    }
    myfile.close();
  }
  else std::cout << "Unable to open file"; 


};

void PathManagementNode::setPath(){
    // this->path_segments.push_back(std::make_shared<Line>(Line(Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(0, 100))));
    this->path_segments.push_back(std::make_shared<Arc>(Arc(Eigen::Vector2d(0, 100), Eigen::Vector2d(-20, 100), M_PI)));
    this->path_segments.push_back(std::make_shared<Arc>(Arc(Eigen::Vector2d(-40, 100), Eigen::Vector2d(-20, 100), M_PI)));
    this->path_segments.push_back(std::make_shared<Arc>(Arc(Eigen::Vector2d(0, 100), Eigen::Vector2d(-20, 100), M_PI)));
    this->path_segments.push_back(std::make_shared<Arc>(Arc(Eigen::Vector2d(-40, 100), Eigen::Vector2d(-20, 100), M_PI)));
    this->path_segments[0]->activate();
    this->path_index = 0;
    this->path_is_set=true;
};



void PathManagementNode::ref_publish(){
    if(!path_is_set){
        return;
    }

    if(!this->path_segments[this->path_index]->is_active){
        if(this->path_index<this->path_segments.size()-1){
            this->path_index+=1;
            this->path_segments[this->path_index]->activate();
        } else{
            this->pathref_msg.x_ref = 0.0;
            this->pathref_msg.y_ref = 0.0;   
            this->pathref_msg.is_active = 0;
            this->path_publisher->publish(this->pathref_msg);
            return;
        }
    }
    Eigen::Vector2d result = this->path_segments[this->path_index]->getClosestPoint(this->current_pose);
    
    this->pathref_msg.path_segment_index = this->path_index;
    this->pathref_msg.x_ref = result(0);
    this->pathref_msg.y_ref = result(1);
    this->pathref_msg.is_active=1;

    this->pathref_msg.tangent_heading = this->path_segments[this->path_index]->getTangHeading(this->current_pose);

    this->pathref_msg.header.stamp = this->pathmanagement_node->get_clock()->now();

    // publish message
    this->path_publisher->publish(this->pathref_msg);
}   




void PathManagementNode::state_subscription_callback(const glassy_interfaces::msg::State::SharedPtr msg){
    this->current_pose(0) = msg->north;
    this->current_pose(1) = msg->east;
}


// for now a simple initialization, parameters may be added in the future
void PathManagementNode::init(){
    this->setPath();

    
    /* -----------------------------
        ROS2 Service Initialization
    -------------------------------*/


    // subscribe to the joystick topic
    this->state_subscription = this->pathmanagement_node->create_subscription<glassy_interfaces::msg::State>("state_vehicle", 1, std::bind(&PathManagementNode::state_subscription_callback, this, _1));

    // initialize publisher
    this->path_publisher = this->pathmanagement_node->create_publisher<glassy_interfaces::msg::Pathreferences>("path_refs", 1);

    // initialize timer, -> dictates when to publish
    this->timer = this->pathmanagement_node->create_wall_timer(100ms, std::bind(&PathManagementNode::ref_publish, this));

}
