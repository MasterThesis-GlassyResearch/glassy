#include "./LOSouterloopYawRate.h"
#include <rclcpp/rclcpp.hpp>






LOSouterloopYawRate::LOSouterloopYawRate(std::shared_ptr<rclcpp::Node> nd, rclcpp::Publisher<glassy_msgs::msg::InnerLoopReferences>::SharedPtr inner_loop_ref_pub){
    this->references.push_back(0.0);
    this->references.push_back(0.0);

        /* -----------------------------
        Get the required parameters
    -------------------------------*/
    nd->declare_parameter("LOS_yr_gains.k1", 10.0);
    nd->declare_parameter("LOS_yr_gains.k2", 10.0);

    k1_ = nd->get_parameter("LOS_yr_gains.k1").as_double();
    k2_ = nd->get_parameter("LOS_yr_gains.k2").as_double();

    RCLCPP_INFO(nd->get_logger(), "LOS_yr_gains.k1: %f", k1_);
    RCLCPP_INFO(nd->get_logger(), "LOS_yr_gains.k2: %f", k2_);


    inner_loop_ref_msg_.surge_ref = 0.0;
    inner_loop_ref_msg_.yaw_ref = 0.0;
    inner_loop_ref_msg_.yaw_rate_ref = 0.0;
    inner_loop_ref_msg_.ctrl_type = glassy_msgs::msg::InnerLoopReferences::SURGE_YAW;

    publisher = inner_loop_ref_pub;

}

    // nd->declare_parameter("LOS_yr_gains.k1", 10.0);
    // nd->declare_parameter("LOS_yr_gains.k2", 10.0);

    // float k1 = this->pathfollowing_node->get_parameter("LOS_yr_gains.k1").as_double();
    // float k2 = this->pathfollowing_node->get_parameter("LOS_yr_gains.k2").as_double();

void LOSouterloopYawRate::computeOutput(glassy_msgs::msg::State::SharedPtr state, Eigen::Vector2d pose_ref,Eigen::Vector2d p_deriv,Eigen::Vector2d p_2nd_deriv, float speed, float duration){

    Eigen::Vector2d pose;
    pose << state->p_ned[0], state->p_ned[1];

    float tangent_heading = atan2(p_deriv(1), p_deriv(0));

    float signed_curvature = 0.0;
    if(p_deriv.norm()>0.0000000000001){
        signed_curvature = p_deriv(0)*p_2nd_deriv(1) - p_deriv(1)*p_2nd_deriv(0);
        signed_curvature = signed_curvature/pow(p_deriv.norm(),3);
    }
    float yaw = state->yaw;

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

    float delta_current = this->delta_function(y1, speed);

    // aproximation of the derivative of the delta function
    float delta_dot = this->delta_function_derivative(delta_current, delta_prev_, duration);

    float part_k2= 0;
    if(psi_tild>10e-10){
        part_k2 = k2_*y1*speed*(sin(psi_err)-sin(this->delta_function(y1, speed)))/(psi_tild);
    }

    float r_ref = signed_curvature*u_p + delta_dot - k1_*psi_tild-
                part_k2;

    if(isnanf(r_ref)){
        r_ref = 0.0;
    }
    
    inner_loop_ref_msg_.surge_ref = speed;
    inner_loop_ref_msg_.yaw_rate_ref = r_ref;
    inner_loop_ref_msg_.ctrl_type = glassy_msgs::msg::InnerLoopReferences::SURGE_YAW_RATE;

    // publish the message
    publisher->publish(inner_loop_ref_msg_);
}