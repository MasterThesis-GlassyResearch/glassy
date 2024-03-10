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
#include <cstring>



using namespace std::chrono_literals;
using namespace std::placeholders;

// class constructor 
PathManagementNode::PathManagementNode(std::shared_ptr<rclcpp::Node> node): pathmanagement_node(node)
{
    std::cout<<"Creating Path Management Controller Node...\n";

}

void PathManagementNode::setPath(std::string file_location){

    // this->correct_home_position();
    std::ifstream myfile(file_location.c_str());
    std::cout<<"my file var created "<<std::endl;
    
    std::string line;
    std::vector<std::string> FileContents;
    try
    {
        // myfile.open();
        if (myfile.is_open())
        {
            while(getline(myfile, line)) {
                FileContents.push_back(line);
            }
            myfile.close();
            std::cout << "File opened and contents extracted..."<<std::endl; 

        }
        else{
            std::cout << "Unable to open file" <<std::endl;
            return;
        } 

    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }

  //* Now that the file is parsed, lets extract the information
    char *str;
    std::string test_str;
    for(int i =0; i<FileContents.size(); i++){
            std::cout << "Starting to go through file contents..."<<std::endl; 
            std::vector<std::string> line_bits;
            std::stringstream ss(FileContents[i]);
            while (ss.good()) {
                std::getline( ss, test_str, ';' );
                line_bits.push_back(test_str);
            }
            if(line_bits[0]=="arc"){
                try {

                    this->path_segments.push_back(std::make_shared<Arc>(
                        Arc(Eigen::Vector2d(std::stof(line_bits[1])-this->x_correction, std::stof(line_bits[2])-this->y_correction), 
                        Eigen::Vector2d(std::stof(line_bits[3])-this->x_correction,
                        std::stof(line_bits[4])-this->y_correction), std::stof(line_bits[5])*M_PI/180)
                        ));
                    this->requested_surge.push_back(std::stof(line_bits[6]));

                } catch (...) {
                    // Conversion failed,
                    std::cout<< "path segment failed to be parsed..." << std::endl;
                }
            } else if(line_bits[0]=="line"){
                try {

                    this->path_segments.push_back(std::make_shared<Line>(
                        Line(Eigen::Vector2d(std::stof(line_bits[1])-this->x_correction, std::stof(line_bits[2])-this->y_correction),
                        Eigen::Vector2d(std::stof(line_bits[3])-this->x_correction,
                        std::stof(line_bits[4])-this->y_correction))
                        ));
                    this->requested_surge.push_back(std::stof(line_bits[5]));

                } catch (...) {
                    // Conversion failed,
                    std::cout<< "path segment failed to be parsed..." << std::endl;
                }
            }
    }

    std::cout<< "finished......." << std::endl;
    std::cout<< "size of path: "<<this->path_segments.size() <<" : " << this->requested_surge.size()<< std::endl;

    this->path_index = 0;
    this->path_segments[this->path_index]->activate();
    this->path_is_set=true;
};

void PathManagementNode::setPath(){
    this->path_segments.push_back(std::make_shared<Line>(Line(Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(0, 100))));
    this->requested_surge.push_back(2.f);
    this->path_segments.push_back(std::make_shared<Arc>(Arc(Eigen::Vector2d(0, 100), Eigen::Vector2d(-20, 100), M_PI)));
    this->requested_surge.push_back(2.f);
    this->path_segments.push_back(std::make_shared<Arc>(Arc(Eigen::Vector2d(-40, 100), Eigen::Vector2d(-20, 100), M_PI)));
    this->requested_surge.push_back(2.f);
    this->path_segments.push_back(std::make_shared<Arc>(Arc(Eigen::Vector2d(0, 100), Eigen::Vector2d(-20, 100), M_PI)));
    this->requested_surge.push_back(2.f);
    this->path_segments.push_back(std::make_shared<Arc>(Arc(Eigen::Vector2d(-40, 100), Eigen::Vector2d(-20, 100), M_PI)));
    this->requested_surge.push_back(2.f);


    // this->path_segments.push_back(std::make_shared<Line>(Line(Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(0, -100))));
    // this->requested_surge.push_back(2.f);
    // this->path_segments.push_back(std::make_shared<Arc>(Arc(Eigen::Vector2d(0, -100), Eigen::Vector2d(-20, -100), -M_PI)));
    // this->requested_surge.push_back(2.f);
    // this->path_segments.push_back(std::make_shared<Arc>(Arc(Eigen::Vector2d(-40, -100), Eigen::Vector2d(-20, -100), -M_PI)));
    // this->requested_surge.push_back(2.f);
    // this->path_segments.push_back(std::make_shared<Arc>(Arc(Eigen::Vector2d(0, -100), Eigen::Vector2d(-20, -100), -M_PI)));
    // this->requested_surge.push_back(2.f);
    // this->path_segments.push_back(std::make_shared<Arc>(Arc(Eigen::Vector2d(-40, -100), Eigen::Vector2d(-20, -100), -M_PI)));
    // this->requested_surge.push_back(2.f);



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
            this->pathref_msg.path_vel = 0.0;
            this->path_publisher->publish(this->pathref_msg);
            return;
        }
    }
    Eigen::Vector2d result = this->path_segments[this->path_index]->getClosestPoint(this->current_pose);
    
    this->pathref_msg.path_segment_index = this->path_index;
    this->pathref_msg.x_ref = result(0);
    this->pathref_msg.y_ref = result(1);
    this->pathref_msg.is_active=1;
    this->pathref_msg.path_vel = this->requested_surge[this->path_index];
    
    std::cout<< "path_seg = " << this->path_index <<std::endl;


    this->pathref_msg.tangent_heading = this->path_segments[this->path_index]->getTangHeading(this->current_pose);

    this->pathref_msg.header.stamp = this->pathmanagement_node->get_clock()->now();

    // publish message
    this->path_publisher->publish(this->pathref_msg);
}   


void PathManagementNode::correct_home_position(){
    if(!this->lat){
        std::cout<<"no state available..."<<std::endl;
        return;
    }
    float R_earth = 6378.137;

    double lat_dif = M_PI/180*(this->home_lat-this->lat);
    double lon_dif = M_PI/180*(this->home_lon-this->lon);


    this->x_correction = R_earth * lat_dif+this->x;
    this->y_correction = R_earth * lon_dif*cos(this->lat)+this->y;

}


void PathManagementNode::state_subscription_callback(const glassy_interfaces::msg::State::SharedPtr msg){
    this->current_pose(0) = msg->north;
    this->current_pose(1) = msg->east;

    this->lat = msg->latitude;
    this->lon = msg->longitude;

}


// for now a simple initialization, parameters may be added in the future
void PathManagementNode::init(){
    std::cout<< " starting this path manager node"<<std::endl;

    std::string file_loc = "/home/joaolehodey/glassy_ws/src/glassy_planning/glassy_pathplanning/PathExamples/lawmower_10_5.txt";

    
    this->setPath(file_loc);
    // this->setPath();

    
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
