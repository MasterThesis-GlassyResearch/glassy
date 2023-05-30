#include <PidController.h>

/*-------------------------
    Public Methods
--------------------------*/

PidController::PidController():  p_gain(0), i_gain(0), d_gain(0)
{
}

PidController::PidController(float kp, float ki, float kd): p_gain(kp), i_gain(ki), d_gain(kd)
{
}

PidController::PidController(float kp, float ki, float kd, float max_err, float max_out): 
p_gain(kp), i_gain(ki), d_gain(kd), max_err(max_err), min_err(-max_err), max_out(max_out), min_out(-max_out)
{
}

PidController::~PidController()
{
}

void PidController::enable(bool yes){
    this->is_disabled = !yes;
};

/*-------------------------
    Protected Methods
--------------------------*/

void PidController::reset_integral(){
    this->integral = 0.0;
}

float PidController::computeOutput(float current_val, float ref_val, float duration, float debug = false){


    if(duration<0.05 || duration>0.2 || this->is_disabled){
        return 0.0;
    }


    float error = ref_val-current_val;

    // take care of error saturation
    error = this->clipping(error, this->max_err, this->min_err);

    // compute proportional action
    float Pterm = error*this->p_gain;
    float Iterm = error*this->p_gain;
    float Dterm = (error-this->prev_error)/duration;

    float output = Pterm+Iterm+Dterm;

    output = this->clipping(output, max_out, min_out);

    return output;

}

/*-------------------------
    Private Methods
--------------------------*/

float PidController::clipping(float val, float max, float min){
    if(val>max) return max;
    else if(val<min) return min;
    return val;
}