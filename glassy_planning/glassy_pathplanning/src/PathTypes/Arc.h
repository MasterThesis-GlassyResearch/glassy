#ifndef _Arc_
#define _Arc_

#include "eigen3/Eigen/Core"
#include <iostream>
#include <stdlib.h>
#include "PathBase.h"
#include <glassy_utils/GlassyGeneralUtils.h>




class Arc: public PathBase
{
private:
    /* data */
    Eigen::Vector2d initial_point;
    Eigen::Vector2d final_point;
    Eigen::Vector2d center;
    Eigen::Vector2d path_dot;
    Eigen::Vector2d path_dot_dot;
    float radius = 0.0;


    float ini_angle;
    float angle_scale;
    float final_angle;
    bool is_circle=false;

    float current_tang_heading=0;

    bool isAngleBetween(float start, float end, float mid) {     
        end = (end - start) < 0.0f ? end - start + 2*M_PI : end - start;    
        mid = (mid - start) < 0.0f ? mid - start + 2*M_PI : mid - start;
        if(this->angle_scale>0){
            return (mid < end); 
        } else{
            return (end<mid);
        }
    }

    

    virtual Eigen::Vector2d getPoint(float gamma);
    virtual Eigen::Vector2d getPointGivenAngle(float angle);
    float PathBase_heading;

public:
    Arc(/* args */){};
    Arc(Eigen::Vector2d start_point, Eigen::Vector2d final_point, Eigen::Vector2d center_circ);
    Arc(Eigen::Vector2d start_point, Eigen::Vector2d center_circ, float angle_scale);
    Eigen::Vector2d getClosestPoint(Eigen::Vector2d point);

    float getTangHeading(Eigen::Vector2d point);


    ~Arc(){};

};


#endif