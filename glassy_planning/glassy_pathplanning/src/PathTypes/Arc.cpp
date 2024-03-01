#include "Arc.h"

Arc::Arc(Eigen::Vector2d start_point, Eigen::Vector2d final_point, Eigen::Vector2d center_circ):
center(center_circ), initial_point(start_point), final_point(final_point)
{
    if(this->initial_point == this->final_point){
        this->is_circle=true;
    } 
    else{
            Eigen::Vector2d dif_ini = this->initial_point-this->center;
        Eigen::Vector2d dif_fin = this->final_point-this->center;
        this->ini_angle = atan2(dif_ini(1),dif_ini(0));
        this->final_angle = atan2(dif_fin(1),dif_fin(0));
    }

    this->radius = (this->initial_point-this->center).norm();


};




float Arc::getTangHeading(Eigen::Vector2d point){
    float angle = atan2(point(1)-this->center(1),point(0)- this->center(0));


    Eigen::Vector2d angular_contrib(-sin(angle), cos(angle));

    return atan2(angular_contrib(1), angular_contrib(0));
}




Eigen::Vector2d Arc::getPoint(float gamma){


    gamma= std::min(std::max(0.f, gamma), 1.f);
    float angle = this->ini_angle+gamma*(this->final_angle-this->ini_angle);

    Eigen::Vector2d angular_contrib(cos(angle), sin(angle));

    return this->center + this->radius * angular_contrib;
};


Eigen::Vector2d Arc::getPointGivenAngle(float angle){

    Eigen::Vector2d angular_contrib;
    if(this->is_circle){
        angular_contrib << cos(angle), sin(angle);
    } else{
        /*
            CHECK ANGLE LIMITS----------------------
        */
        angular_contrib << cos(angle), sin(angle);
    }


    return this->center + this->radius * angular_contrib;
};


Eigen::Vector2d Arc::getClosestPoint(Eigen::Vector2d point){

    float angle = atan2(point(1)-this->center(1),point(0)- this->center(0));
    return getPointGivenAngle(angle);

};