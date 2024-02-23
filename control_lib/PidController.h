#ifndef _PidController_
#define _PidController_

#include <cmath>

// TODO: FINiSH - NOT FINISHED, STARTED BUT DIDNT COMPLETE IT ---
//------------------------------------------------

class PidController
{

protected:

    // pid controller gains
    float p_gain;
    float i_gain;
    float d_gain;

    // error and output boundaries
    float max_integral_val;
    float max_out;
    float min_integral_val; 
    float min_out;

    // integral value
    float integral;
    float prev_error;

    // maybe will be used
    float prev_time;

    bool is_disabled;

    void reset_integral();

    float computePIDOutput(float current_val, float ref_val, float duration, int debug=false);

private:
    float clipping(float value, float max, float min);

public:
    PidController();
    PidController(float kp, float ki, float kd);
    PidController(float kp, float ki, float kd, float max_err, float max_out);
    ~PidController();

    void enable(bool yes);
};


#endif