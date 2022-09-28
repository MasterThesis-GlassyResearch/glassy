#include <iostream>
#include <MavsdkNode.h>
#include <RosNode.h>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <chrono>
#include <thread>
#include <future>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace mavsdk;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;



int main(int argc, char **argv)
{

    // initialize rclcpp
    rclcpp::init(argc, argv);

    //create ros node to be used 
    std::cout << "starting main....\n";
    std::shared_ptr<rclcpp::Node> nh = rclcpp::Node::make_shared("ros_communication_node");

    // initialize an object of both MavlinkNode and RosNode 
    std::shared_ptr<RosNode> ros_com = std::make_shared<RosNode>(nh);
    std::shared_ptr<MavsdkNode> mav_com = std::make_shared<MavsdkNode>();

    // point teach object to each other to be able to call each others methods
    mav_com->ros_node = ros_com;
    ros_com->mav_node = mav_com;

    // initialize publishers in ros node and suscribers in mav node
    ros_com->init();
    mav_com->init();


    // spin node 
    rclcpp::spin(nh);
    rclcpp::shutdown();

}
