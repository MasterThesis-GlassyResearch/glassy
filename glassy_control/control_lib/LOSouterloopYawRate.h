
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


class LOSouterloopYawRate
{
private:
    /* data */

    float k1_;
    float k2_;
    float integral_val = 0.0;
    
    // < surge, yaw >
    std::vector<float> references;


    float delta_function(float y1, float u){
        (void) y1;
        (void) u;
        return 0;
    }

    float delta_function_derivative(float y1, float u){
        (void) y1;
        (void) u;
        return 0;
    }


public:
    LOSouterloopYawRate(float k1, float k2) : k1_(k1), k2_(k2){references.push_back(0.0); references.push_back(0.0);};
    LOSouterloopYawRate(){};
    ~LOSouterloopYawRate(){};
    std::vector<float> computeOutput(Eigen::Vector2d pose_ref, Eigen::Vector2d pose, float yaw,float tangent_heading, float signed_curvature, float speed, float duration);


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