
#ifndef _VanniIntegrated_
#define _VanniIntegrated_

#include <vector>
#include <eigen3/Eigen/Core>
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <glassy_utils/GlassyGeneralUtils.h>
#include "glassy_msgs/msg/inner_loop_references.hpp"
#include "glassy_msgs/msg/actuators.hpp"
#include <OuterLoop.h>
#include <std_msgs/msg/float64.hpp>
#include <glassy_msgs/msg/state.hpp>
#include <glassy_utils/GlassyDynamics.h>


class VanniIntegrated : public OuterLoop
{
private:
    /* data */

    float k1_;
    float k2_;
    float gamma_ = 0.0;
    float gamma_dot_ = 0.0;
    float gamma_dot_dot_ = 0.0;

    float prev_u_star_ = 0.0;
    float prev_r_star_ = 0.0;

    Eigen::Vector2d integral_vec_;

    float prev_time_ = 0.0;
    bool is_on_=false;

    // publishers for inner loop and gamma
    rclcpp::Publisher<glassy_msgs::msg::Actuators>::SharedPtr ref_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gamma_publisher;
    
    // < surge, yaw >
    std::vector<float> references;
    rclcpp::Clock::SharedPtr clock;

    glassy_msgs::msg::Actuators actuator_msg;
    std_msgs::msg::Float64 gamma_msg_;

    rclcpp::Node::SharedPtr node_ptr_;



public:
    VanniIntegrated(){};
    VanniIntegrated(std::shared_ptr<rclcpp::Node> nd, rclcpp::Publisher<glassy_msgs::msg::Actuators>::SharedPtr inner_loop_ref_pub, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gamma_pub);
    ~VanniIntegrated(){};
    void computeOutput(glassy_msgs::msg::State::SharedPtr state, Eigen::Vector2d pose_ref,Eigen::Vector2d p_deriv,Eigen::Vector2d p_2nd_deriv, float speed, float duration) ;
    void reset();
};


#endif