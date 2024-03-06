#include "Arc.h"

Arc::Arc(Eigen::Vector2d start_point, Eigen::Vector2d final_point, Eigen::Vector2d center_circ):
center(center_circ), initial_point(start_point), final_point(final_point)
{
    if(this->initial_point == this->final_point){
        this->is_circle=true;
        angle_scale = 2*M_PI;
    } 
    else{
            Eigen::Vector2d dif_ini = this->initial_point-this->center;
        // Eigen::Vector2d dif_fin = this->final_point-this->center;
        this->ini_angle = atan2(dif_ini(1),dif_ini(0));
        
    
    }

    this->radius = (this->initial_point-this->center).norm();

};

    
Arc::Arc(Eigen::Vector2d start_point, Eigen::Vector2d center_circ, float angle_scale): center(center_circ), initial_point(start_point), angle_scale(angle_scale) {


    Eigen::Vector2d dif_ini = this->initial_point-this->center;
    this->ini_angle = atan2(dif_ini(1),dif_ini(0));



    this->radius = (this->initial_point-this->center).norm();
};



float Arc::getTangHeading(Eigen::Vector2d point){

    float sign = 0;
    if(this->angle_scale<0){
        sign = 1;
    }

    // return wrapToPi(this->current_tang_heading + sign*M_PI);
    return this->current_tang_heading;
}




Eigen::Vector2d Arc::getPoint(float gamma){


    gamma= std::min(std::max(0.f, gamma), 1.f);
    float angle = wrapToPi(this->ini_angle+gamma*(this->angle_scale));

    Eigen::Vector2d angular_contrib(cos(angle), sin(angle));

    return this->center + this->radius * angular_contrib;
};


Eigen::Vector2d Arc::getPointGivenAngle(float angle){

    Eigen::Vector2d angular_contrib;

    angular_contrib << cos(angle), sin(angle);


    return this->center + this->radius * angular_contrib;
};


Eigen::Vector2d Arc::getClosestPoint(Eigen::Vector2d point){

    float angle = atan2(point(1)-this->center(1),point(0)- this->center(0));


    float corresponding_gamma = this->wrapToTwoPi(this->wrapToPi(angle)-this->wrapToPi(this->ini_angle))/angle_scale;

    if(corresponding_gamma<0 || corresponding_gamma>1){
        // handle this... check which is closest...
        if((point-getPointGivenAngle(this->ini_angle)).norm()< (point-getPointGivenAngle(this->ini_angle+this->angle_scale)).norm()){
            corresponding_gamma = 0;
        } else{
            corresponding_gamma = 1;
            this->deactivate(); 
        }
        angle = this->ini_angle+this->angle_scale*corresponding_gamma;
    } else if(corresponding_gamma==0){
        angle = this->ini_angle;
    }


    std::cout<<" Debug: gamma = " << corresponding_gamma << " angle = " << angle << std::endl;
    std::cout<<" Debug: ini_angle = " << this->ini_angle << " angle = " << angle << std::endl;

    // DO NOT FORGET atan(YYYY/XXXX)
    this->current_tang_heading = atan2(cos(angle), -sin(angle));


    return getPointGivenAngle(angle);

};