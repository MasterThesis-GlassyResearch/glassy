#include "./PidController.h"


/*-------------------------
    Public Methods
--------------------------*/

PidController::PidController():  p_gain(0), i_gain(0), d_gain(0)
{
}

PidController::PidController(float kp, float ki, float kd): p_gain(kp), i_gain(ki), d_gain(kd)
{
}

PidController::PidController(float kp, float ki, float kd, float max_err): 
p_gain(kp), i_gain(ki), d_gain(kd), max_integral_val(max_err), min_integral_val(-max_err)
{
}

PidController::~PidController()
{
}

void PidController::enable(bool yes){
    this->is_disabled = !yes;
    if(!this->is_disabled){
        this->reset_integral();
    }
};

void PidController::set_gains(float kp, float ki, float kd){
    this->p_gain = kp;
    this->i_gain = ki;
    this->d_gain = kd;
}

/*-------------------------
    Protected Methods
--------------------------*/

void PidController::reset_integral(){
    this->integral = 0.0;
}


void PidController::full_reset(){
    this->reset_integral();
    this->prev_error= NAN;
}

void PidController::set_integral_max_min(float max, float min){
    this->max_integral_val=max;
    this->min_integral_val=min;
}



float PidController::computePIDOutput(float current_val, float ref_val, float duration, int debug){


    if(duration>0.2 || this->is_disabled){
        return 0.0;
    }


    // calculate error
    float error = ref_val-current_val;


    // compute proportional action
    float Pterm = error*this->p_gain;

    // compute integral action
    if(!std::isnan(this->prev_error)){
        this->integral = this->clipping(this->integral+duration*error, this->max_integral_val, this->min_integral_val);
    }
    float Iterm = this->integral*this->i_gain;

    // compute derivative action
    float Dterm = 0.0;
    if(!std::isnan(this->prev_error)){
        Dterm = ((error-this->prev_error)/duration)*this->d_gain;
    }

    // print to console if debugging
    if(debug){
        std::cout<<"************** PID debug ************"<< std::endl;
        std::cout<<"P = " << Pterm<< std::endl;
        std::cout<<"I = " << Iterm<< std::endl;
        std::cout<<"D = " << Dterm<< std::endl;
        std::cout<<"Integral = "<< integral<< std::endl;
        std::cout<<"Duration = "<< duration<< std::endl;
        std::cout<< "Error = "<< error<< std::endl;
        std::cout<<"Prev. Error "<< prev_error<< std::endl;
        std::cout<<"*************************************"<< std::endl;
    }

    // store current error (used to calculate D term)
    this->prev_error=error;


    // calculate output
    float output = Pterm+Iterm+Dterm;
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