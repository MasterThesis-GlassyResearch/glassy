#ifndef _LOSouterloop_
#define _LOSouterloop_

#include <vector>
#include <eigen3/Eigen/Core>
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include "glassy_utils/GlassyGeneralUtils.h"
#include "glassy_msgs/msg/inner_loop_references.hpp"
#include "glassy_msgs/msg/state.hpp"
#include <OuterLoop.h>

class LOSouterloop : public OuterLoop
{
private:
    /* data */

    float look_ahead_dist = 5; // delta H
    float integral_val = 0.0;
    float sigma = 0.0;
    float max_int = 5;

    rclcpp::Clock::SharedPtr clock;
    
    
    // < surge, yaw >
    std::vector<float> references;
    rclcpp::Publisher<glassy_msgs::msg::InnerLoopReferences>::SharedPtr publisher;


public:
    LOSouterloop(std::shared_ptr<rclcpp::Node> nd, rclcpp::Publisher<glassy_msgs::msg::InnerLoopReferences>::SharedPtr inner_loop_ref_pub);
    LOSouterloop(float look_ahead_dist);
    LOSouterloop(float look_ahead_dist, float sigma);
    LOSouterloop(){};
    ~LOSouterloop(){};
    void computeOutput(glassy_msgs::msg::State::SharedPtr state, Eigen::Vector2d pose_ref,Eigen::Vector2d p_deriv,Eigen::Vector2d p_2nd_deriv, float speed, float duration) override;

    void change_look_ahead_dist(float param);
    void change_sigma(float param);

    bool set_params(float look_ahead, float sigma_int){
        if(look_ahead<=0 || sigma<0.0){
            return false;
        }
        this->look_ahead_dist=look_ahead;
        this->sigma = sigma_int;
        return true;
    }

    void reset_integrator(){
        this->integral_val = 0.0;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Integral value RESET...");
    }

    glassy_msgs::msg::InnerLoopReferences inner_loop_ref_msg;
    

    void set_max_int(float);
};


#endif