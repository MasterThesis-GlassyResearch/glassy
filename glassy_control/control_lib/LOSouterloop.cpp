#include <LOSouterloop.h>


LOSouterloop::LOSouterloop(){
    this->references.push_back(0.0);
    this->references.push_back(0.0);
}
LOSouterloop::LOSouterloop(float look_ahead_dist): look_ahead_dist(look_ahead_dist){

    this->references.push_back(0.0);
    this->references.push_back(0.0);
};


std::vector<float> LOSouterloop::computeOutput(Eigen::Vector2d pose_ref, Eigen::Vector2d pose,float tangent_heading,float speed){

    Eigen::Matrix2d rot;
    rot << cos(tangent_heading), sin(tangent_heading),
           -sin(tangent_heading), cos(tangent_heading);
    

    Eigen::Vector2d error_path_coord =  rot*(pose - pose_ref);

    this->references[0] = speed;

    float heading_ref = fmod((tangent_heading + atan(-error_path_coord(1)/look_ahead_dist)),(2*M_PI));
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
    // has a minimal value
    this->look_ahead_dist = std::max(0.0001f, param);
}