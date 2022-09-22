#ifndef _MavsdkNode_
#define _MavsdkNode_

// #include <MavsdkNode.h>
#include <RosNode.h>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <future>



class RosNode;

class MavsdkNode
{
private:
    void usage_info(const std::string& bin_name);
    std::shared_ptr<mavsdk::System> get_system(mavsdk::Mavsdk& mavsdk);
    void arm_disarm(int mode); // mode = 1 -> arms while mode = 0 disarms

    
public:
    MavsdkNode();
    ~MavsdkNode(){ };

    std::shared_ptr<mavsdk::System> system;
    void init(std::string port = "udp://:14540",bool fowarding = false); //make port a dynamic entry
    void print(); //------------ used for testing purposes
    RosNode* ros_node;




};

#endif