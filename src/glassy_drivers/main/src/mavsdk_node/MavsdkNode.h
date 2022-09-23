#ifndef _MavsdkNode_
#define _MavsdkNode_

#include <RosNode.h>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <future>



class RosNode;

class MavsdkNode
{
private:
    // pribvate methods
    void usage_info(const std::string& bin_name);
    void subscribe_info();


    //private variables
    std::shared_ptr<mavsdk::System> get_system(mavsdk::Mavsdk& mavsdk);
    std::shared_ptr<mavsdk::Telemetry> telemetry;
    std::shared_ptr<mavsdk::Action> action;
    std::shared_ptr<mavsdk::Offboard> offboard;
    

    
public:

    //constructor and destructor
    MavsdkNode();
    ~MavsdkNode(){ };

    //public variables
    std::shared_ptr<mavsdk::System> system;
    RosNode* ros_node;

    //public methods
    void init(std::string port = "udp://:14540",bool fowarding = false); //make port a dynamic entry
    void print(); //------------ used for testing purposes
    void arm_disarm(int mode); // mode = 1 -> arms while mode = 0 disarms





};

#endif