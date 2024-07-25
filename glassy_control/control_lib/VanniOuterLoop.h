
#ifndef _VanniOuterLoop_
#define _VanniOuterLoop_

#include <vector>
#include <eigen3/Eigen/Core>
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <glassy_utils/GlassyGeneralUtils.h>
#include "glassy_msgs/msg/inner_loop_references.hpp"
#include <OuterLoop.h>
#include <std_msgs/msg/float64.hpp>


class VanniOuterLoop
{
private:
    /* data */

    float k1_;
    float k2_;
    float gamma_ = 0.0;
    float gamma_dot = 0.0;
    float gamma_dot_dot = 0.0;


    // publishers for inner loop and gamma
    rclcpp::Publisher<glassy_msgs::msg::InnerLoopReferences>::SharedPtr ref_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gamma_publisher;
    
    // < surge, yaw >
    std::vector<float> references;
    rclcpp::Clock::SharedPtr clock;

    glassy_msgs::msg::InnerLoopReferences inner_loop_ref_msg;



public:
    VanniOuterLoop(){};
    VanniOuterLoop(std::shared_ptr<rclcpp::Node> nd, rclcpp::Publisher<glassy_msgs::msg::InnerLoopReferences>::SharedPtr inner_loop_ref_pub, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gamma_pub);
    ~VanniOuterLoop(){};
    void computeOutput(Eigen::Vector2d pose_ref, Eigen::Vector2d pose,Eigen::Vector2d p_deriv,Eigen::Vector2d p_2nd_deriv, float speed, float duration) ;

};


#endif