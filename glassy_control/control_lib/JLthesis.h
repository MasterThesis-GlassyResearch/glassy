
#ifndef _JLthesis_
#define _JLthesis_

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
#include "glassy_msgs/msg/path_following_debug.hpp"


class JLthesis : public OuterLoop
{
private:
    /* data */

    float k1_;
    float k2_;
    float k_gamma_;
    float kp_u_;
    float kp_r_;
    float ki_u_;
    float ki_r_;


    float integral_u_max_ = 0.5;
    float integral_r_max_ = 0.5;

    float kd_u_ = 0.1;
    float kd_r_ = 0.1;

    bool traj_tracking_ = true;
    bool smc = false;


    float gamma_ = 0.0;
    float gamma_dot_ = 0.0;
    float gamma_dot_dot_ = 0.0;

    float prev_vd_ = 0.0;
    float gamma_d_err_prev_ = 0.0;

    float prev_u_star_ = 0.0;
    float prev_r_star_ = 0.0;

    bool changed_segment_ = false;
    bool reset_r_integral_ = false;

    float prev_gamma_speed_ = 0.0;

    Eigen::Vector2d integral_vec_;

    Eigen::Matrix2d delta_mat_;
    Eigen::Matrix2d delta_mat_inv_;
    Eigen::Vector2d delta_vec_;

    Eigen::Matrix2d K_mat_;
    Eigen::Matrix2d Kp_;
    Eigen::Matrix2d Ki_;
    Eigen::Matrix2d Kd_;

    Eigen::MatrixXd DragDynamicsMatrix;
    Eigen::MatrixXd K_drag_est_;

    Eigen::Vector2d prev_tracking_err_;
    bool first_time_ = true;

    // initial dynamic model drag params

    // u_dot_cancel = p1vr + p2u + p3u2 + Et*Ut + p4u|r| + p5u2|r|
    // ET = a4
    std::vector<double> surgeParamsDrag = { 1.1965, -0.6218, -0.0216, -0.1154, -0.0025};
    // r_dot_cancel = p1vu + p2r + p3r|r| + p4ur + p5u2
    std::vector<double> yawRateParamsDrag = { 2.1225, -0.8592, -0.0963, -1.9001,  0.0123};
    
    Eigen::VectorXd params_estimate;

    float prev_time_ = 0.0;
    bool is_on_=false;

    // publishers for inner loop and gamma
    rclcpp::Publisher<glassy_msgs::msg::Actuators>::SharedPtr ref_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gamma_publisher;
    rclcpp::Publisher<glassy_msgs::msg::PathFollowingDebug>::SharedPtr debug_publisher;

    // debug msg 
    glassy_msgs::msg::PathFollowingDebug debug_msg;
    
    // < surge, yaw >
    std::vector<float> references;
    rclcpp::Clock::SharedPtr clock;

    glassy_msgs::msg::Actuators actuator_msg;
    std_msgs::msg::Float64 gamma_msg_;

    rclcpp::Node::SharedPtr node_ptr_;



public:
    JLthesis(){};
    JLthesis(std::shared_ptr<rclcpp::Node> nd, rclcpp::Publisher<glassy_msgs::msg::Actuators>::SharedPtr inner_loop_ref_pub, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gamma_pub);
    ~JLthesis(){};
    void computeOutput(glassy_msgs::msg::State::SharedPtr state, Eigen::Vector2d pose_ref,Eigen::Vector2d p_deriv,Eigen::Vector2d p_2nd_deriv, float speed, float duration) ;
    void reset();
    void set_segment_change_flag(){this->changed_segment_ = true;};
};


#endif