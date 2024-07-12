#ifndef _Line_
#define _Line_

#include "eigen3/Eigen/Core"
#include <iostream>
#include <stdlib.h>
#include "PathBase.h"
#include <glassy_utils/GlassyGeneralUtils.h>


class Line: public PathBase
{
private:
    /* data */
    Eigen::Vector2d initial_point;
    Eigen::Vector2d final_point;
    Eigen::Vector2d path_dot;
    Eigen::Vector2d path_dot_dot;

    Eigen::Vector2d getPoint(float gamma);
    float line_heading;

public:
    Line(/* args */);
    Line(Eigen::Vector2d start_point, Eigen::Vector2d final_point);
    Eigen::Vector2d getClosestPoint(Eigen::Vector2d point);

    float getTangHeading(Eigen::Vector2d point);

    ~Line();




};


#endif