#include "glassy_utils/GlassyDynamics.h"

Eigen::Vector2d getActuatorsFromDesiredAccelerations(float surge_force, float yaw_rate_force, glassy_msgs::msg::State::SharedPtr state){

        float surge = state->v_body[0];
        float sway = state->v_body[1];
        float yawRate = state->yaw_rate;


        // define the parameters of the dynamical model :)
        float surgeParams[10] = { 1.1965, -0.6218, -0.0216, 16.4,  0.0976,  0.5056, 335.2551, -0.1154, -0.0025, -0.2088};
        float yawRateParams[7] = { 2.1225, -0.8592, -0.0963,  2.2910, -5.5, -1.9001,  0.0123};
        //float angle_params[2] = { 0.1000, -0.6986};
        float angle_factor_deg = 60;
        float epsilon_cnst = 10e-12;
        (void)  epsilon_cnst;

        float cancel_surge = surgeParams[0]*sway*yawRate + surgeParams[1]*surge + surgeParams[2]*surge*surge+ abs(yawRate)*surgeParams[7]*surge + abs(yawRate)*surgeParams[8]*surge*surge;
        float cancel_yaw = yawRateParams[0]*sway*surge + yawRateParams[1]*yawRate + yawRateParams[2]*yawRate*abs(yawRate) + surge*yawRate*yawRateParams[5] + surge*surge*yawRateParams[6];


        float surge_force_new = surge_force - cancel_surge;
        float yaw_rate_force_new = yaw_rate_force - cancel_yaw;

        // float Eff = this->surgeParams[3]*(1-exp(-(this->surgeParams[4]* this->surge*this->surge + this->surgeParams[5])/(abs(this->yawRate)+this->epsilon_cnst)));
        float Eff = surgeParams[3];

        if(std::isnan(Eff)){
            Eff = surgeParams[3];
        }

        float thrust_val = (surge_force_new/Eff);

        float sin_rudder_angle;
        if(surge>10e-10){
            sin_rudder_angle = yaw_rate_force_new/(yawRateParams[3]*surge*surge + yawRateParams[4]*thrust_val);
            // std::cout<< "Sin rudder degrees" <<sin_rudder_angle <<std::endl;
        } else{
            sin_rudder_angle = 0.0;
        }

        sin_rudder_angle = std::min(std::max(sin_rudder_angle,-1.f), 1.f);


        float rudder_angle_degrees = asin(sin_rudder_angle)*180/M_PI;


        float rudder_val = (rudder_angle_degrees)/angle_factor_deg;

        Eigen::Vector2d actuator_values(thrust_val, rudder_val);

        return actuator_values;
}

Eigen::Vector2d getActuatorsFromDesiredAccelerations(float surge_force, float yaw_rate_force, glassy_msgs::msg::State::SharedPtr state, float cancel_dynamics_surge, float cancel_dynamics_yaw_rate){

        float surge = state->v_body[0];


        // at the moment only Eff is needed
        float surgeParams[10] = { 1.1965, -0.6218, -0.0216, 16.4,  0.0976,  0.5056, 335.2551, -0.1154, -0.0025, -0.2088};
        float yawRateParams[7] = { 2.1225, -0.8592, -0.0963,  2.2910, -5.5, -1.9001,  0.0123};
        //float angle_params[2] = { 0.1000, -0.6986};
        float angle_factor_deg = 60;
        float epsilon_cnst = 10e-12;
        (void)  epsilon_cnst;



        float surge_force_new = surge_force- cancel_dynamics_surge;
        float yaw_rate_force_new = yaw_rate_force - cancel_dynamics_yaw_rate;

        // float Eff = this->surgeParams[3]*(1-exp(-(this->surgeParams[4]* this->surge*this->surge + this->surgeParams[5])/(abs(this->yawRate)+this->epsilon_cnst)));
        float Eff = surgeParams[3];

        if(std::isnan(Eff)){
            Eff = surgeParams[3];
        }

        float thrust_val = (surge_force_new/Eff);

        float sin_rudder_angle;
        if(surge>10e-10){
            sin_rudder_angle = yaw_rate_force_new/(yawRateParams[3]*surge*surge + yawRateParams[4]*thrust_val);
            // std::cout<< "Sin rudder degrees" <<sin_rudder_angle <<std::endl;
        } else{
            sin_rudder_angle = 0.0;
        }

        sin_rudder_angle = std::min(std::max(sin_rudder_angle,-1.f), 1.f);


        float rudder_angle_degrees = asin(sin_rudder_angle)*180/M_PI;


        float rudder_val = (rudder_angle_degrees)/angle_factor_deg;

        Eigen::Vector2d actuator_values(thrust_val, rudder_val);

        return actuator_values;
}