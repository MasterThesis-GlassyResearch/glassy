#ifndef _PathBase_
#define _PathBase_

#include "eigen3/Eigen/Core"
#include <iostream>
#include <stdlib.h>
#include <glassy_utils/GlassyGeneralUtils.h>


class PathBase
{
private:
    /* data */
    Eigen::Vector2d initial_point;
    Eigen::Vector2d final_point;
    Eigen::Vector2d path_dot;
    Eigen::Vector2d path_dot_dot;

    virtual Eigen::Vector2d getPoint(float gamma){
        (void) gamma;
        return(Eigen::Vector2d(0.0, 0.0));};
    float PathBase_heading;

public:
    bool is_active=false;
    
    void activate(){
        this->is_active = true;
    };
    void deactivate(){
        this->is_active = false;
    };

    PathBase(/* args */){};
    // PathBase(Eigen::Vector2d start_point, Eigen::Vector2d final_point);
    virtual Eigen::Vector2d getClosestPoint(Eigen::Vector2d point) {(void) point; return(Eigen::Vector2d(0.0, 0.0));};

    virtual float getTangHeading(Eigen::Vector2d point){
        (void) point;
        return(0.0);};


    ~PathBase(){};





};




#endif