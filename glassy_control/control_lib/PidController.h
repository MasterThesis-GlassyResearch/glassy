#ifndef _PidController_
#define _PidController_

#include <cmath>
#include <stdlib.h>
#include <iostream>
#include <chrono>
#include <ctime> 
#include <string>
#include <iostream>



/**
 * @brief This class implements a PID controller
 */
class PidController
{

protected:
    bool is_disabled = false;

private:
    float clipping(float value, float max, float min);

public:
    PidController();
    PidController(float kp, float ki, float kd);
    PidController(float kp, float ki, float kd, float max_integral_err);
    PidController(float *gains);
    ~PidController();

    void reset_integral();
    void set_gains(float kp, float ki, float kd);
    void set_gains(float *gains);
    void set_integral_max_min(float max, float min);
    void full_reset();
    float computePIDOutput(float current_val, float ref_val, float duration, int debug = 0);

    void enable(bool yes);
    

    // terms needed for implementation
    float Pterm;
    float Iterm;
    float Dterm;
    float output;
    float integral = 0.0;
    float error;
    float prev_error= NAN;

    // pid controller gains
    float p_gain = 0.05;
    float i_gain = 0.0;
    float d_gain = 0.0;

    // error and output boundaries
    float max_integral_val = 5;
    float min_integral_val = -5; 

};


#endif