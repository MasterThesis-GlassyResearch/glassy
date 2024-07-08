/*
// Author: João Lehodey (DSOR/ISR/IST), joao.lehodey@tecnico.ulisboa.pt
*/

#include "./PidController.h"


/*-------------------------
    Public Methods
--------------------------*/

/**
 * @brief Constructor without arguments (all gains = 0.0)
 */
PidController::PidController():  p_gain(0), i_gain(0), d_gain(0)
{
}

/**
 * @brief Constructor with gains
 * @arg kp Proportional gain
 * @arg ki Integral gain
 * @arg kd Derivative gain
 */
PidController::PidController(float kp, float ki, float kd): p_gain(kp), i_gain(ki), d_gain(kd)
{
}

/**
 * @brief Constructor with gains
 * @arg [kp, ki, kd] pid gains
 */
PidController::PidController(float *gains): p_gain(gains[0]), i_gain(gains[1]), d_gain(gains[2])
{
}


/**
 * @brief Constructor with gains and saturation value for integral term
 * @arg kp Proportional gain
 * @arg ki Integral gain
 * @arg kd Derivative gain
 */
PidController::PidController(float kp, float ki, float kd, float integral_saturation): 
p_gain(kp), i_gain(ki), d_gain(kd), max_integral_val(integral_saturation), min_integral_val(-integral_saturation)
{
}

/**
 * @brief Destructor
 */
PidController::~PidController()
{
}

/**
 * @brief Enable or disable the controller
 * @arg yes True to enable, false to disable
 */
void PidController::enable(bool yes){
    this->is_disabled = !yes;
    if(!this->is_disabled){
        this->full_reset();
    }
};

/**
 * @brief Set the gains of the controller
 * @arg kp Proportional gain
 * @arg ki Integral gain
 * @arg kd Derivative gain
 */
void PidController::set_gains(float kp, float ki, float kd){
    this->p_gain = kp;
    this->i_gain = ki;
    this->d_gain = kd;
}

/**
 * @brief Set the gains of the controller
 * @arg [kp, ki, kd] pid gains
 */
void PidController::set_gains(float *gains){
    this->p_gain = gains[0];
    this->i_gain = gains[1];
    this->d_gain = gains[2];
}

/*-------------------------
    Protected Methods
--------------------------*/

/**
 * @brief Reset the integral term
 */
void PidController::reset_integral(){
    this->integral = 0.0;
}

/**
 * @brief Reset the integral term and the previous error
 */
void PidController::full_reset(){
    this->reset_integral();
    this->prev_error= NAN;
}

/**
 * @brief Set the maximum and minimum values for the integral term
 * @arg max Maximum value
 * @arg min Minimum value
 */
void PidController::set_integral_max_min(float max, float min){
    this->max_integral_val=max;
    this->min_integral_val=min;
}


/**
 * @brief Compute the output of the PID controller
 * @arg current_val Current value of the system
 * @arg ref_val Reference value
 * @arg duration Time since the last call
 * @arg debug If true, print debug information
 * @return Output of the controller
 */
float PidController::computePIDOutput(float current_val, float ref_val, float duration, int debug){
    // check if the controller is disabled
    if(duration>0.2 || this->is_disabled){
        return 0.0;
    }

    // calculate error
    this->error = ref_val-current_val;


    // compute proportional action
    this->Pterm = error*this->p_gain;

    // compute integral action
    // if(!std::isnan(this->prev_error)){
    // }

    this->integral = this->integral+duration*error;
    this->Iterm = this->integral*this->i_gain;

    // compute derivative action
    if(!std::isnan(this->prev_error)){
        this->Dterm = ((error-this->prev_error)/duration)*this->d_gain;
    } else {
        this->Dterm = 0.0;
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
    this->output = Pterm+Iterm+Dterm;
    return output;
}

/*-------------------------
    Private Methods
--------------------------*/

/**
 * @brief Clip a value between a maximum and minimum value
 * @arg val Value to clip
 * @arg max Maximum value
 * @arg min Minimum value
 * @return Clipped value
 */
float PidController::clipping(float val, float max, float min){
    if(val>max) return max;
    else if(val<min) return min;
    return val;
}