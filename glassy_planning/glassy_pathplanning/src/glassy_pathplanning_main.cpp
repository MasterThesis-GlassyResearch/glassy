#include <cstdio>
#include <stdlib.h>
#include "rclcpp/rclcpp.hpp"
#include "PathManagementNode/PathManagementNode.h"


int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

    // initialize rclcpp
    rclcpp::init(argc, argv);

    //create path planning
    std::cout << "starting path planning node....\n";
    std::shared_ptr<rclcpp::Node> nh = rclcpp::Node::make_shared("glassy_pathplanning");

    std::shared_ptr<PathManagementNode> path_man_com = std::make_shared<PathManagementNode>(nh);

    rclcpp::spin(nh);
    rclcpp::shutdown();

  return 0;
}
