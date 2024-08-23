
#ifndef _LOSouterloopYawRate_
#define _LOSouterloopYawRate_

#include <vector>
#include <eigen3/Eigen/Core>
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <glassy_utils/GlassyGeneralUtils.h>
#include <glassy_msgs/msg/state.hpp>
#include <glassy_msgs/msg/inner_loop_references.hpp>
#include "OuterLoop.h"


class LOSouterloopYawRate : public OuterLoop
{
private:
    /* data */

    float k1_;
    float k2_;
    float integral_val = 0.0;
    float delta_prev_ = 0.0;
    // < surge, yaw >
    std::vector<float> references;


    float delta_function(float y1, float u){
        // set theta as pi/2
        float theta = M_PI/4 * 0.1;
        float k_delta = 1.0;
        return -theta*tanh(k_delta*y1*u);
    }

    float delta_function_derivative(float delta, float delta_prev, float duration){
        return (delta - delta_prev)/duration;
    }


public:
    LOSouterloopYawRate(float k1, float k2) : k1_(k1), k2_(k2){references.push_back(0.0); references.push_back(0.0);};
    LOSouterloopYawRate(std::shared_ptr<rclcpp::Node> nd, rclcpp::Publisher<glassy_msgs::msg::InnerLoopReferences>::SharedPtr inner_loop_ref_pub);
    LOSouterloopYawRate(){};
    ~LOSouterloopYawRate(){};
    void computeOutput(glassy_msgs::msg::State::SharedPtr state, Eigen::Vector2d pose_ref,Eigen::Vector2d p_deriv,Eigen::Vector2d p_2nd_deriv, float speed, float duration) override;


    glassy_msgs::msg::InnerLoopReferences inner_loop_ref_msg_;
    rclcpp::Publisher<glassy_msgs::msg::InnerLoopReferences>::SharedPtr publisher;
    

    bool set_params(float k1, float k2){
        if(k1<=0 || k2<0.0){
            return false;
        }
        k1_ = k1;
        k2_ = k2;
        return true;
    }

    void reset_integrator(){
        this->integral_val = 0.0;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Integral value RESET...");
    }
};


#endif