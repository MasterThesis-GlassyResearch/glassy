#include "Line.h"

Line::Line(/* args */)
{
}

Line::Line(Eigen::Vector2d start_point, Eigen::Vector2d final_point): initial_point(start_point), final_point(final_point), path_dot_dot(0,0)
{

    /*
        eq = x0 + l*(x1-x0)
        -> (x1-x0) = grad;
    */

    this->path_dot = this->final_point-this->initial_point;

    this->line_heading = atan2(this->path_dot(1), this->path_dot(0));
    std::cout<< "Initialized Line with starting point: " << this->initial_point << " and final point "<< this->final_point << std::endl;

}

float Line::getTangHeading(Eigen::Vector2d point){
    (void) point;
    return this->line_heading;
};


Eigen::Vector2d Line::getPoint(float gamma){
    gamma = std::min(std::max(gamma, 0.0f),1.0f);
    return this->initial_point + gamma*(this->final_point -this->initial_point);
}


Eigen::Vector2d Line::getClosestPoint(Eigen::Vector2d point){

    float grad_norm = this->path_dot.squaredNorm();
    float gamma_closest = 0.f;
    if(grad_norm>10e-15){
        gamma_closest = (this->path_dot.dot((point-this->initial_point)))/grad_norm;
    } else{
        return this->initial_point;
    }

    if(gamma_closest>=1){
        this->deactivate();
        gamma_closest=1.0;
    }
    gamma_closest = std::max(0.f, gamma_closest);
    return this->getPoint(gamma_closest);
}



Line::~Line()
{
}