#include "./VanniIntegrated.h"
#include <rclcpp/rclcpp.hpp>


VanniIntegrated::VanniIntegrated(std::shared_ptr<rclcpp::Node> nd, rclcpp::Publisher<glassy_msgs::msg::Actuators>::SharedPtr actuator_publisher, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gamma_pub){

    // get parameters from the node
    nd->declare_parameter("Vanni_gains.k1", 1.0);
    nd->declare_parameter("Vanni_gains.k2", 1.0);
    nd->declare_parameter("Vanni_gains.gamma", 0.0);

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


//   delta: -1.0
//   kx: 0.5
//   ky: 0.5
//   kz: 0.5
//   k_pos: 2.0
//   k_currents: 0.2
//   rd: [0.0, 0.0, 1.0]
//   d: [-5.0, 5.0, 0.0]

// in this case, speed will be vd
void VanniIntegrated::computeOutput(glassy_msgs::msg::State::SharedPtr state, Eigen::Vector2d pose_ref,Eigen::Vector2d p_deriv,Eigen::Vector2d p_2nd_deriv, float speed, float duration){
    // for now ignore all parameters
    (void) p_2nd_deriv;
    (void) speed;



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
    }


    float desired_const_speed = 1;

    std::cout<<"p_deriv: "<<p_deriv(0)<<" "<<p_deriv(1)<<std::endl;
    if(p_deriv.norm() < 0.000000001){
        std::cout<<"p_deriv is too small"<<std::endl;
        this->gamma_publisher->publish(gamma_msg_);
        prev_time_ = node_ptr_->get_clock()->now().nanoseconds();
        return;
    }
    float vd = desired_const_speed/p_deriv.norm();
    // for testing 
    k1_ = 2.0;
    k2_ = 2.0;

    float dt = duration;

    Eigen::Matrix2d rot_I_to_B;
    rot_I_to_B<< cos(state->yaw), sin(state->yaw),
                -sin(state->yaw), cos(state->yaw);


    


    float delta = -1.0;
    Eigen::Matrix2d delta_mat;
    delta_mat<< 1, 0,
                0, -delta;

    Eigen::Matrix2d delta_mat_inv;
    delta_mat_inv<< 1, 0,
                    0, -1/delta;

    Eigen::Vector2d delta_vec( delta, 0.0);

    Eigen::Matrix2d K_mat;
    K_mat<< k1_, 0,
            0, k2_;

    // get the error in body coordinates
    Eigen::Vector2d pose(state->p_ned[0], state->p_ned[1]);
    Eigen::Vector2d p_err = rot_I_to_B*(pose- pose_ref) - delta_vec;

    // float vd = speed;
    Eigen::Vector2d tanh_pos_err(tanh(p_err(0)), tanh(p_err(1)));
    
    Eigen::Vector2d refs = delta_mat_inv*(-K_mat*tanh_pos_err- Eigen::Vector2d(0.0, state->v_body[1]) + rot_I_to_B*p_deriv*vd);


    float k_gamma = 1.0;



    float gamma_d_err = gamma_dot_ - vd;
    
    gamma_dot_dot_ = -k_gamma*gamma_d_err + p_err.transpose()*rot_I_to_B*p_deriv;


    if(gamma_dot_ < vd){
        gamma_dot_dot_ = 0.005;
    }
    else{
        gamma_dot_dot_ = 0.0;
    }
    gamma_dot_dot_ = 0.0;
    gamma_dot_ = vd;

    // update gamma values
    // gamma_dot_ = gamma_dot_ + gamma_dot_dot_*dt;
    gamma_ = gamma_ + gamma_dot_*dt;
    if(gamma_ < 0.0){
        gamma_ = 0.0;
        gamma_dot_ = 0.0;
        gamma_dot_dot_ = 0.0;
    }

    std::cout<<"refs: "<<refs(0)<<" "<<refs(1)<<std::endl;
    std::cout<<"pose_ref: "<<pose_ref(0)<<" "<<pose_ref(1)<<std::endl;
    std::cout<<"pose: "<<pose(0)<<" "<<pose(1)<<std::endl;
    std::cout<<"yaw: "<<state->yaw<<std::endl;
    std::cout<<"gamma: "<<gamma_<<std::endl;
    std::cout<<"gamma_dot: "<<gamma_dot_<<std::endl;
    std::cout<<"gamma_dot_dot: "<<gamma_dot_dot_<<std::endl;
    std::cout<<"gamma_d_err: "<<gamma_d_err<<std::endl;
    std::cout<<"time dt: "<<dt<<std::endl;

    // get the ideal inputs, and then use it to compute the real input 
    float u_star = refs(0);
    float r_star = refs(1);


    // calculate the derivative of the ideal inpts 
    float u_dot_star = (u_star - prev_u_star_)/dt;
    float r_dot_star = (r_star - prev_r_star_)/dt;


    // get the velocities in body frame
    float u = state->v_body[0];

    // get the rates
    float r = state->yaw_rate;

    float error_u = u - u_star;
    float error_r = r - r_star;


    Eigen::Vector2d error_vec(error_u, error_r);
    Eigen::Vector2d ref_vec(u_dot_star, r_dot_star);
    Eigen::Vector2d ref_star_dot(u_dot_star, r_dot_star);



    float kp_u_ = 5;
    float kp_r_ = 5;
    Eigen::Matrix2d proportional_mat;
    proportional_mat<< kp_u_, 0,
                        0, kp_r_;

    Eigen::Matrix2d integral_mat;
    integral_mat<< 0.1, 0.0,
                    0.0, 0.1;
    
    integral_vec_ = integral_vec_ + error_vec*dt;


    Eigen::Vector2d desired_accelerations = ref_star_dot - (error_vec/error_vec.norm())*(-p_err.transpose()*delta_mat*error_vec )- proportional_mat*error_vec - integral_mat*integral_vec_; 


    // print the desired accelerations for debbugging 
    // RCLCCP_IN


    // get the actuator values
    Eigen::Vector2d actuator_values = getActuatorsFromDesiredAccelerations(desired_accelerations(0), desired_accelerations(1), state);


    // update the previous values
    prev_u_star_ = u_star;
    prev_r_star_ = r_star;

    actuator_msg.thrust = actuator_values(0);
    actuator_msg.rudder = actuator_values(1);

    actuator_msg.header.stamp = node_ptr_->get_clock()->now();


    gamma_msg_.data = gamma_;

    this->ref_publisher->publish(actuator_msg);
    this->gamma_publisher->publish(gamma_msg_);


}

void VanniIntegrated::reset(){
    std::cout<<"Resetting the VanniIntegrated"<<std::endl;
    gamma_ = 0.0;
    is_on_ = false;
}