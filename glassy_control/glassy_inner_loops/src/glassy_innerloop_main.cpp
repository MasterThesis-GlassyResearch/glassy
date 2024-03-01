#include <cstdio>
#include <stdlib.h>
#include "PID_ctrl_node/PIDControlNode.h"
// #include "rclcpp/rclcpp.hpp"


int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

    // initialize rclcpp
    rclcpp::init(argc, argv);

    //create joystick node
    std::cout << "starting inner loop node....\n";
    std::shared_ptr<rclcpp::Node> nh = rclcpp::Node::make_shared("glassy_pid");

    std::shared_ptr<PIDControlNode> pid_com = std::make_shared<PIDControlNode>(nh);

    pid_com->init();

    rclcpp::spin(nh);
    rclcpp::shutdown();
    
  return 0;
}
