#ifndef _PathBase_
#define _PathBase_

#include "eigen3/Eigen/Core"
#include <iostream>
#include <stdlib.h>

class PathBase
{
private:
    /* data */
    Eigen::Vector2d path_dot;
    Eigen::Vector2d path_dot_dot;

    virtual Eigen::Vector2d getPoint(float gamma){return(Eigen::Vector2d(0.0, 0.0));};
    float PathBase_heading;

public:

    double wrapToPi(double x){
        x = fmod(x + M_PI,2*M_PI);
        if (x < 0)
            x += 2*M_PI;
        return x - M_PI;
    }

    double wrapToTwoPi(double x){
        x = fmod(x ,2*M_PI);
        if (x < 0)
            x += 2*M_PI;
        return x;
    }

    bool is_active=false;
    
    void activate(){
        this->is_active = true;
    };
    void deactivate(){
        this->is_active = false;
    };

    PathBase(/* args */){};
    // PathBase(Eigen::Vector2d start_point, Eigen::Vector2d final_point);
    virtual Eigen::Vector2d getClosestPoint(Eigen::Vector2d point) {return(Eigen::Vector2d(0.0, 0.0));};

    virtual float getTangHeading(Eigen::Vector2d point){return(6.0);};


    ~PathBase(){};


    Eigen::Vector2d initial_point;
    Eigen::Vector2d final_point;


};




#endif