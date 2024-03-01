#ifndef _Arc_
#define _Arc_

#include "eigen3/Eigen/Core"
#include <iostream>
#include <stdlib.h>
#include "PathBase.h"



class Arc: PathBase
{
private:
    /* data */
    Eigen::Vector2d path_dot;
    Eigen::Vector2d path_dot_dot;
    float ini_angle;
    float final_angle;
    bool is_circle=false;

    

    virtual Eigen::Vector2d getPoint(float gamma);
    virtual Eigen::Vector2d getPointGivenAngle(float angle);
    float PathBase_heading;

public:
    Arc(/* args */){};
    Arc(Eigen::Vector2d start_point, Eigen::Vector2d final_point, Eigen::Vector2d center_circ);
    Eigen::Vector2d getClosestPoint(Eigen::Vector2d point);

    float getTangHeading(Eigen::Vector2d point);


    ~Arc(){};


    Eigen::Vector2d initial_point;
    Eigen::Vector2d final_point;
    Eigen::Vector2d center;
    float radius = 0.0;


};


#endif