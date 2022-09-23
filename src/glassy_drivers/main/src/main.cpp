// #include <mavsdk_node/mavsdk_node.h>
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







int main(int argc, char** argv)
{

//  rclcpp::init(argc, argv);
 
 
 std::cout<< "starting main....\n";
 MavsdkNode mav_com;
 RosNode ros_com;


 
 mav_com.ros_node = &ros_com;
 ros_com.mav_node = &mav_com;


 rclcpp::init(argc, argv);

 mav_com.init();




 
//  ros_com.mav_node->print();

// sleep_for(seconds(20));

 
//  ros_com.init();

// rclcpp::spin(ros_com.ros_node);
// rclcpp::shutdown();   //  -
 
}
