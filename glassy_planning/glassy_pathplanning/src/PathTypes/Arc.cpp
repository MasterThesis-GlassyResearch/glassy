#include "Arc.h"
#include <glassy_utils/GlassyGeneralUtils.h>

Arc::Arc(Eigen::Vector2d start_point, Eigen::Vector2d final_point, Eigen::Vector2d center_circ):
 initial_point(start_point), final_point(final_point), center(center_circ)
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

    
Arc::Arc(Eigen::Vector2d start_point, Eigen::Vector2d center_circ, float angle_scale): initial_point(start_point), center(center_circ), angle_scale(angle_scale)  {


    Eigen::Vector2d dif_ini = this->initial_point-this->center;
    this->ini_angle = atan2(dif_ini(1),dif_ini(0));

    std::cout<<"Initializing Arc with center: " << this->center << std::endl;

    this->radius = (this->initial_point-this->center).norm();
};



float Arc::getTangHeading(Eigen::Vector2d point){
    (void) point;
    float sign = 0;
    if(this->angle_scale<0){
        sign = 1;
    }

    return this->current_tang_heading+ sign*M_PI;
}




Eigen::Vector2d Arc::getPoint(float gamma){


    gamma= std::min(std::max(0.f, gamma), 1.f);
    float angle = wrapToPi((this->ini_angle+gamma*(this->angle_scale)));

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


    float angle_diff = wrapToTwoPi((wrapToPi(angle)-wrapToPi(this->ini_angle)));

    if(this->angle_scale<0){
        angle_diff-=2*M_PI;
    }

    float corresponding_gamma =  angle_diff/angle_scale;


    if(corresponding_gamma<0 || corresponding_gamma>=1){
        // handle this... check which is closest...
        std::cout<< "point_x: "<< point(0) << "point_y: "<< point(1)<<std::endl;
        std::cout<< "initial point_x: "<< getPointGivenAngle(this->ini_angle)(0) << "point_y: "<< getPointGivenAngle(this->ini_angle)(1)<<std::endl;
        std::cout<< "final point_x: "<< getPointGivenAngle(this->ini_angle+this->angle_scale)(0) << "point_y: "<< getPointGivenAngle(this->ini_angle+this->angle_scale)(1)<<std::endl;
        std::cout<< "Dist between point and beg: "<< (point-getPointGivenAngle(this->ini_angle)).norm() << "dist point end: "<< (point-getPointGivenAngle(this->ini_angle+this->angle_scale)).norm()<<std::endl;
        if( (point-getPointGivenAngle(this->ini_angle)).norm()< (point-getPointGivenAngle(this->ini_angle+this->angle_scale)).norm()){
            corresponding_gamma = 0;
        } else{
            corresponding_gamma = 1;
            this->deactivate(); 
        }
        angle = this->ini_angle+this->angle_scale*corresponding_gamma;
    } else if(corresponding_gamma==0){
        angle = this->ini_angle;
    }


    // std::cout<<" Debug: gamma = " << corresponding_gamma << " angle = " << angle << std::endl;
    // std::cout<<" Debug: ini_angle = " << this->ini_angle << " angle = " << angle << std::endl;

    this->current_tang_heading = atan2(cos(angle), -sin(angle));


    return getPointGivenAngle(angle);

};