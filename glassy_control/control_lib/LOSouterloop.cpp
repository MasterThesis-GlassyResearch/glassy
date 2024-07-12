#include "./LOSouterloop.h"
#include "glassy_utils/GlassyGeneralUtils.h"


LOSouterloop::LOSouterloop(){
    this->references.push_back(0.0);
    this->references.push_back(0.0);
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

std::vector<float> LOSouterloop::computeOutput(Eigen::Vector2d pose_ref, Eigen::Vector2d pose,float tangent_heading,float speed, float duration){

    Eigen::Matrix2d rot;
    rot << cos(tangent_heading), sin(tangent_heading),
           -sin(tangent_heading), cos(tangent_heading);
    

    Eigen::Vector2d error_path_coord =  rot*(pose - pose_ref);

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

    return references;
}


void LOSouterloop::change_look_ahead_dist(float param){
    // has a minimum value
    this->look_ahead_dist = std::max(0.0001f, param);
}

void LOSouterloop::change_sigma(float param){
    // has a minimum value
    this->sigma = param;
}