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

    // if(this->sigma!=0){
    // float duration = 0.0; ---> see how to make this...

    // this->integral_val = this->integral_val+ duration*(this->look_ahead_dist*(error_path_coord(1))/((error_path_coord(1) +
    //  this->look_ahead_dist)+ this->look_ahead_dist*this->look_ahead_dist));
    // }

    //FIXME add integral stuff

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