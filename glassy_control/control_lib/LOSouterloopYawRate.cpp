#include "./LOSouterloopYawRate.h"
#include <rclcpp/rclcpp.hpp>

std::vector<float> LOSouterloopYawRate::computeOutput(Eigen::Vector2d pose_ref, Eigen::Vector2d pose, float yaw,float tangent_heading, float signed_curvature, float speed, float duration){

    (void) duration;    

    Eigen::Matrix2d rot;
    rot << cos(tangent_heading), sin(tangent_heading),
           -sin(tangent_heading), cos(tangent_heading);
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pose ref: %f, %f", pose_ref(0), pose_ref(1));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pose: %f, %f", pose(0), pose(1));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Yaw: %f", yaw);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Tangent heading: %f", tangent_heading);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Curvature: %f", signed_curvature);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Speed: %f", speed);


    Eigen::Vector2d error_path_coord =  rot*(pose - pose_ref);
    float y1 = error_path_coord(1);

    float psi_err = wrapToPi(yaw-tangent_heading);
    float psi_tild = psi_err - this->delta_function(y1, speed);
    // float psi_tild = 


    float u_p = speed*cos(psi_err)/(1-y1*signed_curvature);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "u_p: %f", u_p);

    float part_k2= 0;
    if(psi_tild>10e-10){
        part_k2 = k2_*y1*speed*(sin(psi_err)-sin(this->delta_function(y1, speed)))/(psi_tild);
    }

    float r_ref = signed_curvature*u_p + this->delta_function_derivative(y1, speed) - k1_*psi_tild-
                part_k2;

    if(isnanf(r_ref)){
        r_ref = 0.0;
    }
    this->references[0] = speed;
    this->references[1] = r_ref;

    return this->references;
}