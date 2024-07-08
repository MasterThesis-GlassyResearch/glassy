#include "PIDControlNode.h"

/**
 * @brief Construct a new PIDControlNode::PIDControlNode object
 * 
 * @param node 
 */
PIDControlNode::PIDControlNode(std::shared_ptr<rclcpp::Node> node): pid_glassy_node(node)
{
    std::cout<<"Creating PID Controller Node...\n";
    this->prev_time = this->pid_glassy_node->get_clock()->now();
    this->ctrlType = SURGE_YAW;
}



/*-------------------------------------
         Timer CallBack
-------------------------------------*/

/**
 * @brief Calculate the control output and publish to the actuators
 */
void PIDControlNode::direct_actuator_publish(){
    rclcpp::Time current_time = this->pid_glassy_node->get_clock()->now();

    rclcpp::Duration duration = current_time - this->prev_time;

    this->prev_time = current_time;
    


    // Start by taking care of the surge componnent:
    float pidValSurge = this->surgePIDCtrl.computePIDOutput(this->surge, this->surge_ref, duration.nanoseconds()/1e9, false);

    // Now take care of YAW or YAWRATE
    float pidValYaw = 0.0;
    if(this->ctrlType == SURGE_YAW){

        // take care of ensuring shortest way to desired yaw
        if(this->yaw_ref-this->yaw>M_PI){
            this->yaw +=2*M_PI;
        } else if(this->yaw_ref-this->yaw<-M_PI){
            this->yaw -=2*M_PI;
        }

        pidValYaw = this->yawPIDCtrl.computePIDOutput(this->yaw, this->yaw_ref, duration.nanoseconds()/1e9, false);
    } 
    else if(this->ctrlType == SURGE_YAWRATE){
        pidValYaw = this->yawRatePIDCtrl.computePIDOutput(this->yawRate, this->yawRate_ref, duration.nanoseconds()/1e9, false);
    } else{
        pidValYaw = 0.0;
    }

    /* --------------------------
        ADD THE 'CANCELLING PART'
    ----------------------------*/
    float surge_force;
    float yaw_force;
    if(this->cancel_dynamics){
        // calculate surge canceling part:
        float cancel_surge = this->surgeParams[0]*this->sway*this->yawRate + this->surgeParams[1]*this->surge + this->surgeParams[2]*this->surge*this->surge+ abs(this->yawRate)*this->surgeParams[7]*this->surge + abs(this->yawRate)*this->surgeParams[8]*this->surge*this->surge;
        float cancel_yaw = this->yawRateParams[0]*this->sway*this->surge + this->yawRateParams[1]*this->yawRate + this->yawRateParams[2]*this->yawRate*abs(this->yawRate) + this->surge*this->yawRate*this->yawRateParams[5] + this->surge*this->surge*this->yawRateParams[6];

        

        surge_force = pidValSurge - cancel_surge;
        yaw_force = pidValYaw - cancel_yaw;
    } 
    else{
        surge_force = pidValSurge;
        yaw_force = pidValYaw;
    }



    // float Eff = this->surgeParams[3]*(1-exp(-(this->surgeParams[4]* this->surge*this->surge + this->surgeParams[5])/(abs(this->yawRate)+this->epsilon_cnst)));
    float Eff = this->surgeParams[3];

    if(std::isnan(Eff)){
        Eff = this->surgeParams[3];
    }

    float thrust_val = (surge_force/Eff);

    float sin_rudder_angle;
    if(this->surge>10e-10){
        sin_rudder_angle = yaw_force/(this->yawRateParams[3]*this->surge*this->surge + this->yawRateParams[4]*thrust_val);
        // std::cout<< "Sin rudder degrees" <<sin_rudder_angle <<std::endl;
    } else{
        sin_rudder_angle = 0.0;
    }

    sin_rudder_angle = std::min(std::max(sin_rudder_angle,-1.f), 1.f);


    float rudder_angle_degrees = asin(sin_rudder_angle)*180/M_PI;


    float rudder_val = (rudder_angle_degrees)/angle_factor_deg;

    /* --------------------------
        Publish to the actuators
    ----------------------------*/

    this->direct_actuator_msg.thrust = std::min(std::max( thrust_val, 0.f), 1.f);
    this->direct_actuator_msg.rudder = std::min(std::max( rudder_val, -1.f), 1.f);

    this->direct_actuator_msg.header.stamp = current_time;
    this->actuator_publisher->publish(this->direct_actuator_msg);
}   


/*-------------------------------------
         Subscription CallBacks 
-------------------------------------*/

/**
 * @brief Callback function for the reference subscription
 *
 * @param msg 
 */
void PIDControlNode::referrence_subscription_callback(const glassy_msgs::msg::InnerLoopReferences::SharedPtr msg){
    // Set the correct references to track...
    this->surge_ref = msg->surge_ref;
    this->yaw_ref = msg->yaw_ref;
    this->yawRate_ref = msg->yaw_rate_ref;
}

/**
 * @brief Callback function for the mission info subscription
 *
 * @param msg 
 */
void PIDControlNode::mission_info_subscription_callback(const glassy_msgs::msg::MissionInfo::SharedPtr msg){
    if(this->is_active && this->mission_type != msg->mission_mode){
        std::cout<<"DE"<<std::endl;

        this->deactivate();
        this->mission_type = msg->mission_mode;
    } else{
        // if(!this->is_active && std::find(MissionTypesInnerLoop.begin(), MissionTypesInnerLoop.end(), msg->mission_mode) != MissionTypesInnerLoop.end()){
        if(!this->is_active && msg->mission_mode == MissionInfo::PATH_FOLLOWING){
        

            std::cout<<"ACTIVATING"<<std::endl;
            this->activate();
            this->mission_type = msg->mission_mode;

        }
    }
}


/**
 * @brief Callback function for the state subscription
 *
 * @param msg 
 */
void PIDControlNode::state_subscription_callback(const glassy_msgs::msg::State::SharedPtr msg){
    // linear velocities 2D
    this->surge = msg->v_body[0];
    this->sway = msg->v_body[1];

    // orientation and angular velocitie 2D
    this->yaw = msg->yaw;
    this->yawRate = msg->yaw_rate;
}


/*-------------------------------------
         Service CallBacks 
-------------------------------------*/

/**
 * @brief update the gains of the PID controller used for surge control using a service
 * 
 * @param request 
 * @param response 
 */
void PIDControlNode::update_gains_surge_callback(const std::shared_ptr<glassy_msgs::srv::PidGains::Request> request, std::shared_ptr<glassy_msgs::srv::PidGains::Response> response){
    (void) response;
    this->surgePIDCtrl.set_gains(request->kp, request->ki, request->kd);
    this->surgePIDCtrl.full_reset();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Surge PID gains set to: kp = %f, ki = %f, kd = %f", request->kp, request->ki, request->kd);
}

/**
 * @brief update the gains of the PID controller used for yaw control using a service
 * 
 * @param request 
 * @param response 
 */
void PIDControlNode::update_gains_yaw_callback(const std::shared_ptr<glassy_msgs::srv::PidGains::Request> request, std::shared_ptr<glassy_msgs::srv::PidGains::Response> response){
    (void) response;
    this->yawPIDCtrl.set_gains(request->kp, request->ki, request->kd);
    this->yawPIDCtrl.full_reset();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Yaw PID gains set to: kp = %f, ki = %f, kd = %f", request->kp, request->ki, request->kd);
}

/**
 * @brief update the gains of the PID controller used for yaw rate control using a service
 * 
 * @param request 
 * @param response 
 */
void PIDControlNode::update_gains_yawRate_callback(const std::shared_ptr<glassy_msgs::srv::PidGains::Request> request, std::shared_ptr<glassy_msgs::srv::PidGains::Response> response){
    (void) response;
    this->yawRatePIDCtrl.set_gains(request->kp, request->ki, request->kd);
    this->yawRatePIDCtrl.full_reset();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Yaw Rate PID gains set to: kp = %f, ki = %f, kd = %f", request->kp, request->ki, request->kd);
}


/**
 * @brief Update the integral limits of the PID controller used for surge control
 * 
 * @param request 
 * @param response 
 */
void PIDControlNode::update_integral_limits_surge_callback(const std::shared_ptr<glassy_msgs::srv::SetLimits::Request> request, std::shared_ptr<glassy_msgs::srv::SetLimits::Response> response){
    (void) response;
    this->surgePIDCtrl.set_integral_max_min(request->max_value, request->min_value);
}

/**
 * @brief Update the integral limits of the PID controller used for yaw control
 * 
 * @param request 
 * @param response 
 */
void PIDControlNode::update_integral_limits_yaw_callback(const std::shared_ptr<glassy_msgs::srv::SetLimits::Request> request, std::shared_ptr<glassy_msgs::srv::SetLimits::Response> response){
    (void) response;
    this->yawPIDCtrl.set_integral_max_min(request->max_value, request->min_value);
}

/**
 * @brief Update the integral limits of the PID controller used for yaw rate control
 * 
 * @param request 
 * @param response 
 */
void PIDControlNode::update_integral_limits_yawRate_callback(const std::shared_ptr<glassy_msgs::srv::SetLimits::Request> request, std::shared_ptr<glassy_msgs::srv::SetLimits::Response> response){
    (void) response;
    this->yawRatePIDCtrl.set_integral_max_min(request->max_value, request->min_value);
}



/*-----------------------------------
    Activate and deactivate logic
------------------------------------*/

/**
 * @brief Activate the inner loops
 */
void PIDControlNode::activate(){
    this->yawPIDCtrl.full_reset();
    this->yawRatePIDCtrl.full_reset();
    this->surgePIDCtrl.full_reset();

    std::cout<< "STARTED INNER LOOP SUCCESSFULLY"<< std::endl;

    this->is_active=true;
}

/**
 * @brief Deactivate the inner loops
 */
void PIDControlNode::deactivate(){
    this->is_active=false;
    // this->yawPIDCtrl.full_reset();
    // this->yawRatePIDCtrl.full_reset();
    // this->surgePIDCtrl.full_reset();
}


/**
 * @brief Initialize the PID controller node
 */
void PIDControlNode::init(){


    

    /* -----------------------------
        Parameter Initialization
    -------------------------------*/

    // Declare all of the parameters 
    this->pid_glassy_node->declare_parameter("rates.inner_loop", 20.0);
    this->pid_glassy_node->declare_parameter("pid_gains.surge", std::vector<double>({1.0, 0.1, 0.0}));
    this->pid_glassy_node->declare_parameter("pid_gains.yaw", std::vector<double>({1.0, 0.1, 0.0}));
    this->pid_glassy_node->declare_parameter("pid_gains.yaw_rate", std::vector<double>({1.0, 0.1, 0.0}));

    // Initialize all the parameters
    this->update_rate = this->pid_glassy_node->get_parameter("rates.inner_loop").as_double();
    std::vector<double> gains_surge = this->pid_glassy_node->get_parameter("pid_gains.surge").as_double_array();
    std::vector<double> gains_yaw = this->pid_glassy_node->get_parameter("pid_gains.yaw").as_double_array();
    std::vector<double> gains_yaw_rate = this->pid_glassy_node->get_parameter("pid_gains.yaw_rate").as_double_array();



    const auto node_graph_interface = this->pid_glassy_node->get_node_graph_interface();

    RCLCPP_INFO(this->pid_glassy_node->get_logger(), "surge gains: %.2f, %.2f, %.2f", gains_surge[0], gains_surge[1], gains_surge[2]);
    RCLCPP_INFO(this->pid_glassy_node->get_logger(), "yaw gains: %.2f, %.2f, %.2f", gains_yaw[0], gains_yaw[1], gains_yaw[2]);
    RCLCPP_INFO(this->pid_glassy_node->get_logger(), "yaw_rate gains: %.2f, %.2f, %.2f", gains_yaw_rate[0], gains_yaw_rate[1], gains_yaw_rate[2]);
    RCLCPP_INFO(this->pid_glassy_node->get_logger(), "update rate: %f", this->update_rate);

    /* -----------------------------
        Variable Initialization
    -------------------------------*/

    // this->thrust_gain = this->pid_glassy_node->get_parameter("thrust_gain").as_double();

    this->surgePIDCtrl.set_gains(gains_surge[0], gains_surge[1], gains_surge[2]);
    this->yawPIDCtrl.set_gains(gains_yaw[0], gains_yaw[1], gains_yaw[2]);



    
    /* -----------------------------
        ROS2 Service Initialization
    -------------------------------*/


    // subscribe to the reference topic
    this->ref_subscription = this->pid_glassy_node->create_subscription<glassy_msgs::msg::InnerLoopReferences>("/glassy/innerloop_refs", 1, std::bind(&PIDControlNode::referrence_subscription_callback, this, _1));

    // subscribe to the joystick topic
    this->state_subscription = this->pid_glassy_node->create_subscription<glassy_msgs::msg::State>("/glassy/state", 1, std::bind(&PIDControlNode::state_subscription_callback, this, _1));

    //subscribe to the mission info
    this->mission_info_subscription = this->pid_glassy_node->create_subscription<glassy_msgs::msg::MissionInfo>("/glassy/mission_status", 1, std::bind(&PIDControlNode::mission_info_subscription_callback, this, _1));

    // initialize publisher
    this->actuator_publisher = this->pid_glassy_node->create_publisher<glassy_msgs::msg::Actuators>("/glassy/actuators", 1);

    this->timer = this->pid_glassy_node->create_wall_timer(1.0s/this->update_rate, std::bind(&PIDControlNode::direct_actuator_publish, this));

    /*------------------------------
                 SERVICES
     -----------------------------*/

    // set gains services
    this->change_gains_surge = this->pid_glassy_node->create_service<glassy_msgs::srv::PidGains>("pid_gains_surge", std::bind(&PIDControlNode::update_gains_surge_callback, this, _1, _2));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Set GAINS SURGE service Ready...");
    this->change_gains_yaw = this->pid_glassy_node->create_service<glassy_msgs::srv::PidGains>("pid_gains_yaw", std::bind(&PIDControlNode::update_gains_yaw_callback, this, _1, _2));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Set GAINS YAW service Ready...");
    this->change_gains_yawRate = this->pid_glassy_node->create_service<glassy_msgs::srv::PidGains>("pid_gains_yaw_rate", std::bind(&PIDControlNode::update_gains_yawRate_callback, this, _1, _2));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Set GAINS YAW-RATE service Ready...");

    // set integral limits services
    this->change_integral_limits_surge = this->pid_glassy_node->create_service<glassy_msgs::srv::SetLimits>("pid_integral_limits_surge", std::bind(&PIDControlNode::update_integral_limits_surge_callback, this, _1, _2));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Update SURGE integral limits service Ready...");
    this->change_integral_limits_yaw = this->pid_glassy_node->create_service<glassy_msgs::srv::SetLimits>("pid_integral_limits_yaw", std::bind(&PIDControlNode::update_integral_limits_yaw_callback, this, _1, _2));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Update YAW integral limits service Ready...");

    this->change_integral_limits_yawRate = this->pid_glassy_node->create_service<glassy_msgs::srv::SetLimits>("pid_integral_limits_yaw_rate", std::bind(&PIDControlNode::update_integral_limits_yawRate_callback, this, _1, _2));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Update YAW-RATE integral limits service Ready...");
   
   
    // Log after everything is initialized
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Inner Loops ready...");
}
