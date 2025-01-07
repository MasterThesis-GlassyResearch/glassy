#include "./JLthesis.h"
#include <rclcpp/rclcpp.hpp>


JLthesis::JLthesis(std::shared_ptr<rclcpp::Node> nd, rclcpp::Publisher<glassy_msgs::msg::Actuators>::SharedPtr actuator_publisher, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gamma_pub){

    // get parameters from the node
    nd->declare_parameter("JLIntegrated_params.k1", 1.0);
    nd->declare_parameter("JLIntegrated_params.k2", 1.0);
    nd->declare_parameter("JLIntegrated_params.delta", -0.5);
    nd->declare_parameter("JLIntegrated_params.k_gamma", 1.0);
    nd->declare_parameter("JLIntegrated_params.kp_u", 1.0);
    nd->declare_parameter("JLIntegrated_params.kp_r", 1.0);

    nd->declare_parameter("JLIntegrated_params.ki_u", 1.0);
    nd->declare_parameter("JLIntegrated_params.ki_r", 1.0);

    nd->declare_parameter("JLIntegrated_params.ku_param_update",  std::vector<double>({0.01, 0.01, 0.01, 0.01, 0.01}));
    nd->declare_parameter("JLIntegrated_params.kr_param_update",  std::vector<double>({0.01, 0.01, 0.01, 0.01, 0.01}));

    nd->declare_parameter("JLIntegrated_params.trajtracking",  true);

    nd->declare_parameter("JLIntegrated_params.integral_u_max", 1.0);
    nd->declare_parameter("JLIntegrated_params.integral_r_max", 1.0);

    nd->declare_parameter("JLIntegrated_params.kd_u", 0.1);
    nd->declare_parameter("JLIntegrated_params.kd_r", 0.1);

    nd->declare_parameter("JLIntegrated_params.backstepping_term", 1.0);

    nd->declare_parameter("JLIntegrated_params.reset_r_integral", false);

    reset_r_integral_ = nd->get_parameter("JLIntegrated_params.reset_r_integral").as_bool();


    k1_ = nd->get_parameter("JLIntegrated_params.k1").as_double();
    k2_ = nd->get_parameter("JLIntegrated_params.k2").as_double();


    kd_u_ = nd->get_parameter("JLIntegrated_params.kd_u").as_double();
    kd_r_ = nd->get_parameter("JLIntegrated_params.kd_r").as_double();

    k_gamma_ = nd->get_parameter("JLIntegrated_params.k_gamma").as_double();
    float delta_ = nd->get_parameter("JLIntegrated_params.delta").as_double();

    backstepping_const_ = nd->get_parameter("JLIntegrated_params.backstepping_term").as_double();

    kp_u_ = nd->get_parameter("JLIntegrated_params.kp_u").as_double();
    kp_r_ = nd->get_parameter("JLIntegrated_params.kp_r").as_double();

    ki_u_ = nd->get_parameter("JLIntegrated_params.ki_u").as_double();
    ki_r_ = nd->get_parameter("JLIntegrated_params.ki_r").as_double();

    integral_u_max_ = nd->get_parameter("JLIntegrated_params.integral_u_max").as_double();
    integral_r_max_ = nd->get_parameter("JLIntegrated_params.integral_r_max").as_double();

    traj_tracking_ = nd->get_parameter("JLIntegrated_params.trajtracking").as_bool();

    std::vector<double> ku_param_update = nd->get_parameter("JLIntegrated_params.ku_param_update").as_double_array();
    std::vector<double> kr_param_update = nd->get_parameter("JLIntegrated_params.kr_param_update").as_double_array();
    

    // rclcpp info all the parameters for debugging purposes
    RCLCPP_INFO(nd->get_logger(), "USING JL Integrated CONTROLLER");
    RCLCPP_INFO(nd->get_logger(), "JLIntegrated_params.k1: %f", k1_);
    RCLCPP_INFO(nd->get_logger(), "JLIntegrated_params.k2: %f", k2_);
    RCLCPP_INFO(nd->get_logger(), "JLIntegrated_params.delta: %f", delta_);
    RCLCPP_INFO(nd->get_logger(), "JLIntegrated_params.k_gamma: %f", k_gamma_);
    RCLCPP_INFO(nd->get_logger(), "JLIntegrated_params.kp_u: %f", kp_u_);
    RCLCPP_INFO(nd->get_logger(), "JLIntegrated_params.kp_r: %f", kp_r_);
    RCLCPP_INFO(nd->get_logger(), "JLIntegrated_params.ki_u: %f", ki_u_);
    RCLCPP_INFO(nd->get_logger(), "JLIntegrated_params.ki_r: %f", ki_r_);

    RCLCPP_INFO(nd->get_logger(), "JLIntegrated_params.backstepping_term: %f", backstepping_const_);


    RCLCPP_INFO(nd->get_logger(), "JLIntegrated_params.integral_u_max: %f", integral_u_max_);
    RCLCPP_INFO(nd->get_logger(), "JLIntegrated_params.integral_r_max: %f", integral_r_max_);



    debug_publisher = nd->create_publisher<glassy_msgs::msg::PathFollowingDebug>("path_following_debug", 1); 




    /* Insert the values into the corresponding matrices and vectors*/
    delta_mat_<< 1, 0,
                0, -delta_;

    delta_vec_<< delta_, 0.0;

    delta_mat_inv_<< 1, 0,
                    0, -1/delta_;

    K_mat_<< k1_, 0,
            0, k2_;

    Kp_<< kp_u_, 0,
            0, kp_r_;

    Ki_<< ki_u_, 0,
            0, ki_r_;

    //TODO: check this derivative term and if it helps
    Kd_<< kd_u_, 0,
          0, kd_r_;


    K_drag_est_ = Eigen::MatrixXd(9, 9);


    // put the values of the parameters into the matrix diagonal first u then r
    K_drag_est_.diagonal() << ku_param_update[0], ku_param_update[1], ku_param_update[2], ku_param_update[3], kr_param_update[0], kr_param_update[1], kr_param_update[2], kr_param_update[3], kr_param_update[4];

    std::cout<<"K_drag_est_ equals "<<K_drag_est_<<std::endl;
    params_estimate = Eigen::VectorXd(9);
    params_estimate << surgeParamsDrag[0], surgeParamsDrag[1], surgeParamsDrag[2], surgeParamsDrag[3], yawRateParamsDrag[0], yawRateParamsDrag[1], yawRateParamsDrag[2], yawRateParamsDrag[3], yawRateParamsDrag[4];

                

    node_ptr_ = nd;

    this->ref_publisher = actuator_publisher;
    this->gamma_publisher = gamma_pub;

    this->gamma_msg_.data = 0.0;

    this->references.push_back(0.0);
    this->references.push_back(0.0);


    gamma_ = 0.0;
    gamma_dot_ = 0.0;
    gamma_dot_dot_ = 0.0;

    integral_vec_<< 0.0, 0.0;
}



/**
 * @brief compute the output of the controller
 * 
 * @param state the current state of the system
 * @param pose_ref the reference pose
 * @param p_deriv the derivative of the reference pose
 * @param p_2nd_deriv the second derivative of the reference pose
 * @param speed the speed of the vehicle
 * @param duration the duration of the control
 */
void JLthesis::computeOutput(glassy_msgs::msg::State::SharedPtr state, Eigen::Vector2d pose_ref,Eigen::Vector2d p_deriv,Eigen::Vector2d p_2nd_deriv, float speed, float duration){
    // for now ignore all parameters
    (void) p_2nd_deriv;


    // check if any of the necessary values is nan
    if(std::isnan(pose_ref(0)) || std::isnan(pose_ref(1)) || std::isnan(p_deriv(0)) || std::isnan(p_deriv(1))){
        std::cout<<"Nan values in the pose_ref or p_deriv"<<std::endl;
        gamma_msg_.data=0.0;
        this->gamma_publisher->publish(gamma_msg_);
        prev_time_ = node_ptr_->get_clock()->now().nanoseconds();
        return;
    }

    if(duration > 0.5){
        std::cout<<"Duration is too large"<<std::endl;
        gamma_msg_.data=0.0;
        this->gamma_publisher->publish(gamma_msg_);
        prev_time_ = node_ptr_->get_clock()->now().nanoseconds();
        return;
    } else if(duration < 0.00001){
        std::cout<<"Duration is too small"<<std::endl;
        return;
    }



    if(p_deriv.norm() < 0.0001){
        std::cout<<"p_deriv is too small"<<std::endl;
        this->gamma_publisher->publish(gamma_msg_);
        prev_time_ = node_ptr_->get_clock()->now().nanoseconds();
        return;
    }
    float vd = speed/p_deriv.norm();
    
    if(vd>40.0){
        std::cout<<"vd is too large"<<std::endl;
        // print some variables
        std::cout<<"vd: "<<vd<<std::endl;
        std::cout<<"speed: "<<speed<<std::endl;
        std::cout<<"p_deriv: "<<p_deriv(0)<<" "<<p_deriv(1)<<std::endl;
        std::cout<<"norm p_deriv: "<<p_deriv.norm()<<std::endl;
        return;
    }

    float dt = duration;



    Eigen::Matrix2d rot_I_to_B;
    rot_I_to_B<< cos(state->yaw), sin(state->yaw),
                -sin(state->yaw), cos(state->yaw);


    
    /* Get the pose in a vector and the error in body coordinates (delta already included)*/
    Eigen::Vector2d pose(state->p_ned[0], state->p_ned[1]);
    Eigen::Vector2d p_err = rot_I_to_B*(pose- pose_ref) - delta_vec_;

    /* Get the tanh of the error, this is used as saturation, not necessarily needed*/
    Eigen::Vector2d tanh_pos_err(tanh(p_err(0)), tanh(p_err(1)));
    
    /* Get the references for the path following controller (see thesis Vanni)*/
    Eigen::Vector2d refs = delta_mat_inv_*(-K_mat_*tanh_pos_err - Eigen::Vector2d(0.0, state->v_body[1]) + rot_I_to_B*p_deriv*vd);


    /* Get the gamma_dot_dot_ value, this is the acceleration of the virtual target*/
    float gamma_d_err = gamma_dot_ - vd;

    float gamma_d_err_deriv = 0.0;
    if(!first_time_){
        gamma_d_err_deriv = (gamma_d_err-gamma_d_err_prev_)/dt;
    }
    float k_gamma_deriv = 0.0;
    gamma_dot_dot_ = -k_gamma_*gamma_d_err + p_err.transpose()*rot_I_to_B*p_deriv - k_gamma_deriv * gamma_d_err_deriv;

    float vd_dot = 0.0;
    if(!first_time_){
        vd_dot = (vd - prev_vd_)/dt;
        gamma_dot_dot_ = gamma_dot_dot_ + vd_dot;
    }

    // limit gamma_dot_dot_
    if(gamma_dot_dot_ > 0.5){
        gamma_dot_dot_ = 0.5;
    } else if(gamma_dot_dot_ < -0.5){
        gamma_dot_dot_ = -0.5;
    }
    /* Check whether or not to track the trajectory, or to use gamma_dot_dot_ designated from the path following approach */
    if(traj_tracking_ || first_time_ || changed_segment_){
        gamma_dot_ = vd;

        if(reset_r_integral_){
            integral_vec_(1) = 0.0;
        }
    } else{
        gamma_dot_ = gamma_dot_ + gamma_dot_dot_*dt;
        if(gamma_dot_ < 0.0){
            gamma_dot_ = 0.0;
        }
        changed_segment_ = false;
    }

    /*Update gamma, take care of case when gamma<0*/
    gamma_ = gamma_ + gamma_dot_*dt;
    if(gamma_ < 0.0){
        gamma_ = 0.0;
        gamma_dot_ = 0.0;
        gamma_dot_dot_ = 0.0;
    }

    /* Print stuff, for debugging purposes */
    // std::cout<<"refs: "<<refs(0)<<" "<<refs(1)<<std::endl;
    // std::cout<<"pose_ref: "<<pose_ref(0)<<" "<<pose_ref(1)<<std::endl;
    // std::cout<<"pose: "<<pose(0)<<" "<<pose(1)<<std::endl;
    // std::cout<<"yaw: "<<state->yaw<<std::endl;
    // std::cout<<"gamma: "<<gamma_<<std::endl;
    // std::cout<<"gamma_dot: "<<gamma_dot_<<std::endl;
    // std::cout<<"gamma_dot_dot: "<<gamma_dot_dot_<<std::endl;
    // std::cout<<"gamma_d_err: "<<gamma_d_err<<std::endl;
    // std::cout<<"time dt: "<<dt<<std::endl;

    /* Get the 'ideal inputs'*/ 
    float u_star = refs(0);
    float r_star = refs(1);




    /* calculate the derivative of the ideal inpts */ 
    float u_dot_star = (u_star - prev_u_star_)/dt;
    float r_dot_star = (r_star - prev_r_star_)/dt;



    /*Get the state info*/
    float u = state->v_body[0];
    float r = state->yaw_rate;
    float v = state->v_body[1];

    /*calculate the difference between the state and the ideal state*/
    float error_u = u - u_star;
    float error_r = r - r_star;

    /* Insert the information into vectors for faster operations*/
    Eigen::Vector2d tracking_err(error_u, error_r);
    Eigen::Vector2d ref_vec(u_dot_star, r_dot_star);
    Eigen::Vector2d ref_star_dot(u_dot_star, r_dot_star);
    Eigen::Vector2d tracking_err_deriv(0.0, 0.0);

    /* Calculate the derivative of the error*/
    if(first_time_){
        first_time_ = false;
    }else{
        tracking_err_deriv = (tracking_err - prev_tracking_err_)/dt;
    }


    // correct the previous implementation of integrals, because it was wrong
    integral_vec_(0) = integral_vec_(0) + tracking_err(0)*dt;
    if(integral_vec_(0) > integral_u_max_){
        integral_vec_(0) = integral_u_max_;
    } else if(integral_vec_(0) < -integral_u_max_){
        integral_vec_(0) = -integral_u_max_;
    }

    integral_vec_(1) = integral_vec_(1) + tracking_err(1)*dt;
    if(integral_vec_(1) > integral_r_max_){
        integral_vec_(1) = integral_r_max_;
    } else if(integral_vec_(1) < -integral_r_max_){
        integral_vec_(1) = -integral_r_max_;
    }
    



    // calculate the desired acceleration in surge and desired angular acceleration in yaw rate

    /*
        MAKE SURE TO CORRECT AND CHECK TAHAT THIS IS CHANGED SUCH THAT THE DERIVATIVE ACTION IS CORRECT
        \\TODO
    */
   // Eigen::Vector2d desired_accelerations = ref_star_dot - (Eigen::Matrix2d::Identity()+Kd_)*(tracking_err/tracking_err.norm())*(p_err.transpose()*delta_mat_*tracking_err )- Kp_*tracking_err - Ki_*(Eigen::Matrix2d::Identity()+Kd_).inverse().transpose()*integral_vec_ - Kd_*tracking_err_deriv ; 

    // //test but should be equal to this:
    Eigen::Vector2d desired_accelerations = ref_star_dot - backstepping_const_*(Eigen::Matrix2d::Identity()+Kd_).inverse()*delta_mat_.transpose()*p_err - Kp_*tracking_err - Ki_*(Eigen::Matrix2d::Identity()+Kd_).inverse().transpose()*integral_vec_ - Kd_*tracking_err_deriv; 


    // generate the time varying drag dynamics matrix
    DragDynamicsMatrix = Eigen::MatrixXd(2 , 9);
    DragDynamicsMatrix << r*v, u, u*u, u*abs(r),  0, 0, 0, 0, 0,
                            0, 0, 0, 0 ,v*u, r, r*abs(r), u*r, u*u;

    /* Update the parameters of the drag dynamics*/
    params_estimate = params_estimate + K_drag_est_*( (Eigen::Matrix2d::Identity()+Kd_).inverse() *  DragDynamicsMatrix).transpose()*tracking_err * dt;






    // calculate the cancelation terms
    float cancel_u_dot = DragDynamicsMatrix.row(0)*params_estimate;
    float cancel_r_dot = DragDynamicsMatrix.row(1)*params_estimate;


    // get the actuator values
    // Eigen::Vector2d actuator_values = getActuatorsFromDesiredAccelerations(desired_accelerations(0), desired_accelerations(1), state);
    Eigen::Vector2d actuator_values = getActuatorsFromDesiredAccelerations(desired_accelerations(0), desired_accelerations(1), state, cancel_u_dot, cancel_r_dot);


    // update the previous values
    prev_u_star_ = u_star;
    prev_r_star_ = r_star;


    // previous tracking error
    prev_tracking_err_ = tracking_err;

    // update previous vd
    prev_vd_ = vd;

    // update the previous gamma_speed
    prev_gamma_speed_ = gamma_dot_*p_deriv.norm();

    /*Update the actuator msg  and gamma msg fields and publish*/
    actuator_msg.thrust = actuator_values(0);
    actuator_msg.rudder = actuator_values(1);
    actuator_msg.header.stamp = node_ptr_->get_clock()->now();
    gamma_msg_.data = gamma_;
    this->ref_publisher->publish(actuator_msg);
    this->gamma_publisher->publish(gamma_msg_);


    std::vector<double> surgeParamsDrag_estimated = {params_estimate(0), params_estimate(1), params_estimate(2), params_estimate(3), integral_vec_(0)};
    std::vector<double> yawRateParamsDrag_estimated = {params_estimate(4), params_estimate(5), params_estimate(6), params_estimate(7), params_estimate(8), integral_vec_(1)};

    /*fill in the debug msg*/
    debug_msg.header.stamp = node_ptr_->get_clock()->now();
    debug_msg.surge_drag_param_estimates = surgeParamsDrag_estimated;
    debug_msg.yawrate_drag_param_estimates = yawRateParamsDrag_estimated;
    debug_msg.u_star = u_star;
    debug_msg.r_star = r_star;

    debug_msg.u = u;
    debug_msg.r = r;
    debug_msg.u_err = error_u;
    debug_msg.r_err = error_r;
    debug_msg.p_err_x_body = p_err(0);
    debug_msg.p_err_y_body = p_err(1);
    debug_msg.gamma_dot = gamma_dot_;
    debug_msg.gamma_dot_dot = gamma_dot_dot_;
    debug_msg.gamma_dot_err = gamma_d_err;


    // change so its first row
    debug_msg.u_deriv_contrib = -Kd_.row(0)*tracking_err_deriv;
    debug_msg.u_integral_contrib = -Ki_.row(0)*integral_vec_;
    debug_msg.u_proportional_contrib = -Kp_.row(0)*tracking_err;

    debug_msg.u_star_deriv_contrib = ref_star_dot(0);
    debug_msg.r_star_deriv_contrib = ref_star_dot(1);
    
    debug_msg.r_deriv_contrib = -Kd_.row(1)*tracking_err_deriv;
    debug_msg.r_integral_contrib = -Ki_.row(1)*integral_vec_;
    debug_msg.r_proportional_contrib = -Kp_.row(1)*tracking_err;
    /* publish debug msg*/
    debug_publisher->publish(debug_msg);

}

/**
 * @brief reset all the values of the controller
 */
void JLthesis::reset(){
    std::cout<<"Resetting the JLthesis"<<std::endl;
    gamma_ = 0.0;
    gamma_dot_ = 0.0;
    is_on_ = false;
    first_time_ = true;
    integral_vec_<< 0.0, 0.0;
    if(params_estimate.rows() == 0 || params_estimate.cols() == 0){
        return;
    }
    params_estimate << surgeParamsDrag[0], surgeParamsDrag[1], surgeParamsDrag[2], surgeParamsDrag[3],  yawRateParamsDrag[0], yawRateParamsDrag[1], yawRateParamsDrag[2], yawRateParamsDrag[3], yawRateParamsDrag[4];

}