#ifndef _LOSouterloop_
#define _LOSouterloop_

#include <vector>
#include <eigen3/Eigen/Core>
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <cmath>


class LOSouterloop
{
private:
    /* data */

    float look_ahead_dist = 5;
    
    // < surge, yaw >
    std::vector<float> references;

public:
    LOSouterloop(/* args */);
    LOSouterloop(float look_ahead_dist);
    ~LOSouterloop(){};
    std::vector<float> computeOutput(Eigen::Vector2d pose_ref, Eigen::Vector2d pose,float tangent_heading,float speed);

    void change_look_ahead_dist(float param);
};


#endif