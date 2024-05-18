/*
Developers: João Lehodey - joao.lehodey@tecnico.ulisboa.pt - DSOR/ISR team (Instituto Superior Tecnico) 
*/

#ifndef _MissionTypesInnerLoop_
#define _MissionTypesInnerLoop_

#include <glassy_msgs/msg/mission_info.hpp>
#include <vector>
#include <string>
using namespace glassy_msgs::msg;

const std::vector<uint8_t> MissionTypesInnerLoop = {
    MissionInfo::PATH_FOLLOWING,
    MissionInfo::TRAJECTORY_TRACKING
};


#endif