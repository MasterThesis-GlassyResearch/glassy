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

class MavsdkNode;
class RosNode
{
private:

    
public:
    RosNode();
    ~RosNode(){ };

    MavsdkNode* mav_node;

};

#endif