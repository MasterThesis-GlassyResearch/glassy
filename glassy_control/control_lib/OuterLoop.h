#ifndef _OuterLoop_
#define _OuterLoop_

#include <vector>
#include <eigen3/Eigen/Core>
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <cmath>


class OuterLoop
{
private:
    /* data */
    std::vector<float> references;
public:
    OuterLoop(/* args */){
        references.push_back(0.0);
        references.push_back(0.0);
    };
    ~OuterLoop(){};
    virtual void computeOutput(Eigen::Vector2d pose_ref, Eigen::Vector2d pose,Eigen::Vector2d p_deriv,Eigen::Vector2d p_2nd_deriv, float speed, float duration){
        // since nothing runs here, to avoid warnings 
        (void) pose_ref;
        (void) pose;
        (void) p_deriv;
        (void) p_2nd_deriv;
        (void) speed;
        (void) duration;
    };
};


#endif