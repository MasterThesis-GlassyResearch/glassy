#ifndef _PidController_
#define _PidController_

#include <cmath>
#include <stdlib.h>
#include <iostream>
#include <chrono>
#include <ctime> 
#include <string>
#include <iostream>

// TODO: Clean up and test

class PidController
{

protected:

    // pid controller gains
    float p_gain = 0.05;
    float i_gain = 0.0;
    float d_gain = 0.0;

    // error and output boundaries
    float max_integral_val = 5;
    float min_integral_val = -5; 

    // integral value
    float integral = 0.0;
    float prev_error= 0.0;

    // maybe will be used
    float prev_time = 0.0;

    bool is_disabled = false;



private:
    float clipping(float value, float max, float min);

public:
    PidController();
    PidController(float kp, float ki, float kd);
    PidController(float kp, float ki, float kd, float max_integral_err);
    ~PidController();

    void reset_integral();
    void set_gains(float kp, float ki, float kd);
    float computePIDOutput(float current_val, float ref_val, float duration, int debug);

    void enable(bool yes);
};


#endif