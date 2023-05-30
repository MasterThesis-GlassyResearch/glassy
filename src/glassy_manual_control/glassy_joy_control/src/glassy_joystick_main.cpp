#include <stdio.h>
#include <iostream>
#include <JoyControlNode.h>



int main(int argc, char *argv[]){
    std::cout<<"hello\n";

    // initialize rclcpp
    rclcpp::init(argc, argv);

    //create joystick node
    std::cout << "starting joystick node....\n";
    std::shared_ptr<rclcpp::Node> nh = rclcpp::Node::make_shared("glassy_joy_node");

    // initialize an object of both MavlinkNode and RosNode 
    std::shared_ptr<JoyControlNode> joy_com = std::make_shared<JoyControlNode>(nh);

    joy_com->init();

    rclcpp::spin(nh);
    rclcpp::shutdown();
}