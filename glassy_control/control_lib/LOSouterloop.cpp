#include "./LOSouterloop.h"
#include "glassy_utils/GlassyGeneralUtils.h"


LOSouterloop::LOSouterloop(std::shared_ptr<rclcpp::Node> nd, rclcpp::Publisher<glassy_msgs::msg::InnerLoopReferences>::SharedPtr inner_loop_ref_pub){
    this->references.push_back(0.0);
    this->references.push_back(0.0);

        /* -----------------------------
        Get the required parameters
    -------------------------------*/
    nd->declare_parameter("LOS_gains.look_ahead", 5.0);
    nd->declare_parameter("LOS_gains.sigma", 0.0);
    nd->declare_parameter("LOS_gains.max_int", 5.0);

    clock = nd->get_clock();


    float look_ahead = nd->get_parameter("LOS_gains.look_ahead").as_double();
    float sigma = nd->get_parameter("LOS_gains.sigma").as_double();
    float max_int = nd->get_parameter("LOS_gains.max_int").as_double();

    RCLCPP_INFO(nd->get_logger(), "LOS Look Ahead: %f", look_ahead);
    RCLCPP_INFO(nd->get_logger(), "LOS Sigma: %f", sigma);
    RCLCPP_INFO(nd->get_logger(), "LOS max int: %f", max_int);

    this->look_ahead_dist = look_ahead;
    this->sigma = sigma;
    this->max_int = max_int;

    this->inner_loop_ref_msg.surge_ref = 0.0;
    this->inner_loop_ref_msg.yaw_ref = 0.0;
    this->inner_loop_ref_msg.yaw_rate_ref = 0.0;
    this->inner_loop_ref_msg.ctrl_type = glassy_msgs::msg::InnerLoopReferences::SURGE_YAW;

    this->publisher = inner_loop_ref_pub;

}
LOSouterloop::LOSouterloop(float look_ahead_dist): look_ahead_dist(look_ahead_dist), sigma(0.0){

    this->references.push_back(0.0);
    this->references.push_back(0.0);
};

LOSouterloop::LOSouterloop(float look_ahead_dist, float sigma): look_ahead_dist(look_ahead_dist), sigma(sigma){
    this->references.push_back(0.0);
    this->references.push_back(0.0);
};

void LOSouterloop::set_max_int(float max_int){
    this->max_int = max_int;
}

void LOSouterloop::computeOutput(glassy_msgs::msg::State::SharedPtr state, Eigen::Vector2d pose_ref, Eigen::Vector2d p_deriv,Eigen::Vector2d p_2nd_deriv, float speed, float duration){

    // to avoid warnings
    (void) p_2nd_deriv;





    Eigen::Vector2d pose(state->p_ned[0], state->p_ned[1]);  

    Eigen::Matrix2d rot;
    float tangent_heading = atan2(p_deriv(1), p_deriv(0));
    rot << cos(tangent_heading), sin(tangent_heading),
           -sin(tangent_heading), cos(tangent_heading);



    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "LOS outer loop computing output...");
    // info variaables to debug
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pose ref: %f, %f", pose_ref(0), pose_ref(1));    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "p_deriv: %f, %f", p_deriv(0), p_deriv(1));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "p_2nd_deriv: %f, %f", p_2nd_deriv(0), p_2nd_deriv(1));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Speed: %f", speed);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Duration: %f", duration);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pose: %f, %f", state->p_ned[0], state->p_ned[1]);  
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Tangent heading: %f", tangent_heading); 

    Eigen::Vector2d error_path_coord =  rot*(pose - pose_ref);

    // check if any nan value
    if(std::isnan(error_path_coord(0)) || std::isnan(error_path_coord(1))){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "NAN value detected in error path coord...");
        return;
    }

    if(this->sigma!=0 && duration<0.2){
        this->integral_val = clip_val((this->integral_val+ duration*(this->look_ahead_dist*(error_path_coord(1))/((error_path_coord(1) +
        this->look_ahead_dist)+ this->look_ahead_dist*this->look_ahead_dist))), this->max_int, -this->max_int);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Integral value: %f", this->integral_val);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Duration: %f", duration);
    }



    this->references[0] = speed;

    float heading_ref = fmod((tangent_heading + atan(-(error_path_coord(1)+ this->integral_val*this->sigma)/look_ahead_dist)),(2*M_PI));
    if (heading_ref < 0){
        heading_ref += 2*M_PI;
    }
    if (heading_ref>M_PI){
        heading_ref -= 2*M_PI;
    }
    this->references[1] = heading_ref;

    this->inner_loop_ref_msg.surge_ref = this->references[0];
    this->inner_loop_ref_msg.yaw_ref = this->references[1];
    this->inner_loop_ref_msg.header.stamp = this->clock->now();

    this->publisher->publish(this->inner_loop_ref_msg);
}


void LOSouterloop::change_look_ahead_dist(float param){
    // has a minimum value
    this->look_ahead_dist = std::max(0.0001f, param);
}

void LOSouterloop::change_sigma(float param){
    // has a minimum value
    this->sigma = param;
}