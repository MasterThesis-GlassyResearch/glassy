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
    std::vector<float> computeOutput(){return references;};
};


#endif