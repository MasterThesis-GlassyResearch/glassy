#ifndef _RosNode_
#define _RosNode_

#include <MavsdkNode.h>
// #include <RosNode.h>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <future>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MavsdkNode;
class RosNode
{
private:

    
public:
    RosNode();
    ~RosNode(){ };
    void init();

    MavsdkNode* mav_node;   //consider adding to private variables, using then public function to set it 
    std::shared_ptr<rclcpp::Node> ros_node;   //consider adding to private variables, using then public function to set it 
    
};

#endif