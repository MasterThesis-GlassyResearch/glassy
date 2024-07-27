#include "./VanniOuterLoop.h"
#include <rclcpp/rclcpp.hpp>


VanniOuterLoop::VanniOuterLoop(std::shared_ptr<rclcpp::Node> nd, rclcpp::Publisher<glassy_msgs::msg::InnerLoopReferences>::SharedPtr inner_loop_ref_pub, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gamma_pub){

    // get parameters from the node
    nd->declare_parameter("Vanni_gains.k1", 1.0);
    nd->declare_parameter("Vanni_gains.k2", 1.0);
    nd->declare_parameter("Vanni_gains.gamma", 0.0);

    clock = nd->get_clock();

    this->ref_publisher = inner_loop_ref_pub;
    this->gamma_publisher = gamma_pub;
    this->references.push_back(0.0);
    this->references.push_back(0.0);
}

void VanniOuterLoop::computeOutput(glassy_msgs::msg::State::SharedPtr state, Eigen::Vector2d pose_ref,Eigen::Vector2d p_deriv,Eigen::Vector2d p_2nd_deriv, float speed, float duration){
    // for now ignore all parameters
    (void) pose_ref;
    (void) state;
    (void) p_deriv;
    (void) p_2nd_deriv;
    (void) speed;
    (void) duration;


    // get current time:
    float current_time = clock->now().seconds();
    float dt = current_time - prev_time;

    if(dt > 0.1){
        // do something        
    }

    
    // get the 






    // update the previous time
    prev_time = current_time;
    // lets implement this thing
}