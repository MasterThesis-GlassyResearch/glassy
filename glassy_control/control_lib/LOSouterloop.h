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

    float look_ahead_dist = 5; // delta H
    float integral_val = 0.0;
    float sigma = 0.0;
    
    // < surge, yaw >
    std::vector<float> references;


public:
    LOSouterloop(/* args */);
    LOSouterloop(float look_ahead_dist);
    ~LOSouterloop(){};
    std::vector<float> computeOutput(Eigen::Vector2d pose_ref, Eigen::Vector2d pose,float tangent_heading,float speed);

    void change_look_ahead_dist(float param);

    bool set_params(float look_ahead, float sigma_int){
        if(look_ahead<=0 || sigma<0.0){
            return false;
        }
        this->look_ahead_dist=look_ahead;
        this->sigma = sigma_int;
        std::cout<<"PARAMETERS SET SUCCESSFULLY look ahead = "<< this->look_ahead_dist << " --- sigma = "<<this->sigma<<std::endl;
        return true;
    }

    void reset_integrator(){
        this->integral_val = 0.0;
    }
};


#endif