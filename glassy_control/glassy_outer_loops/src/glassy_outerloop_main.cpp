#include <cstdio>
#include <stdlib.h>
#include "rclcpp/rclcpp.hpp"
#include "PathFollowingNode/PathFollowingNode.h"


int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

    // initialize rclcpp
    rclcpp::init(argc, argv);

    //create joystick node
    std::cout << "starting inner loop node....\n";
    std::shared_ptr<rclcpp::Node> nh = rclcpp::Node::make_shared("glassy_PF");

    std::shared_ptr<PathFollowingNode> pathfollowing_com = std::make_shared<PathFollowingNode>(nh);

    pathfollowing_com->init();

    rclcpp::spin(nh);
    rclcpp::shutdown();
    
  return 0;
}
