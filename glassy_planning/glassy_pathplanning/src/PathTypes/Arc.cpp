#include "Arc.h"
#include <glassy_utils/GlassyGeneralUtils.h>


/**
 * @brief Construct a new Arc:: Arc object
 * 
 * @param start_point 
 * @param center_circ 
 * @param angle_scale 
 */    
Arc::Arc(Eigen::Vector2d start_point, Eigen::Vector2d center_circ, float angle_scale): initial_point(start_point), center(center_circ), angle_scale(angle_scale)  {


    Eigen::Vector2d dif_ini = this->initial_point-this->center;
    this->ini_angle = atan2(dif_ini(1),dif_ini(0));

    std::cout<<"Initializing Arc with center: " << this->center << std::endl;

    this->radius = (this->initial_point-this->center).norm();

    float sign = 0.0;
    if(this->angle_scale>0){
        sign = 1;
    } else{
        sign = -1;
    }
    this->curvature = sign/this->radius;
    
};


/**
 * @brief Get the Tang Heading object
 * 
 * @param point 
 * @return float 
 */
float Arc::getTangHeading(Eigen::Vector2d point){
    (void) point;
    float sign = 0;
    if(this->angle_scale<0){
        sign = 1;
    }

    return this->current_tang_heading+ sign*M_PI;
}



/**
 * @brief Get the point corresponding to a given gamma (parameterization of the arc)
 * 
 *  ----->  p(gamma) = center + radius*[cos(ini_angle+gamma*angle_scale), sin(ini_angle+gamma*angle_scale)]
 * 
 * @param gamma 
 * @return Eigen::Vector2d 
 */
Eigen::Vector2d Arc::getPoint(float gamma){


    gamma= std::min(std::max(0.f, gamma), 1.f);
    float angle = wrapToPi((this->ini_angle+gamma*(this->angle_scale)));

    Eigen::Vector2d angular_contrib(cos(angle), sin(angle));

    return this->center + this->radius * angular_contrib;
};

/**
 * @brief Get the point corresponding to a given angle
 * 
 * @param angle 
 * @return Eigen::Vector2d 
 */
Eigen::Vector2d Arc::getPointGivenAngle(float angle){

    Eigen::Vector2d angular_contrib;

    angular_contrib << cos(angle), sin(angle);


    return this->center + this->radius * angular_contrib;
};

/**
 * @brief Get the closest point to a given point
 * 
 * @param point 
 * @return Eigen::Vector2d 
 */
Eigen::Vector2d Arc::getClosestPoint(Eigen::Vector2d point){

    float angle = atan2(point(1)-this->center(1),point(0)- this->center(0));


    float angle_diff = wrapToTwoPi((wrapToPi(angle)-wrapToPi(this->ini_angle)));

    if(this->angle_scale<0){
        angle_diff-=2*M_PI;
    }

    float corresponding_gamma =  angle_diff/angle_scale;


    if(corresponding_gamma<0 || corresponding_gamma>=1){
        // handle this... check which is closest...
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

    this->current_tang_heading = atan2(cos(angle), -sin(angle));


    return getPointGivenAngle(angle);

};

/**
 * @brief Get the derivative of the arc at a given point
 * 
 * @param gamma 
 * @return Eigen::Vector2d 
 */
Eigen::Vector2d Arc::getPathDerivative(float gamma){
    gamma= std::min(std::max(0.f, gamma), 1.f);
    float angle = wrapToPi((this->ini_angle+gamma*(this->angle_scale)));

    Eigen::Vector2d path_deriv(-sin(angle), cos(angle));

    return path_deriv;
};

/**
 * @brief Get the second derivative of the arc at a given point
 * 
 * @param gamma 
 * @return Eigen::Vector2d 
 */
Eigen::Vector2d Arc::getPathSecondDerivative(float gamma){
    gamma= std::min(std::max(0.f, gamma), 1.f);
    float angle = wrapToPi((this->ini_angle+gamma*(this->angle_scale)));

    Eigen::Vector2d path_deriv_deriv(-cos(angle), -sin(angle));

    return path_deriv_deriv;
};
