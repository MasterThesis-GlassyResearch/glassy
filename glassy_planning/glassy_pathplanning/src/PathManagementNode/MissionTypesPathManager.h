/*
Developers: João Lehodey - joao.lehodey@tecnico.ulisboa.pt - DSOR/ISR team (Instituto Superior Tecnico) 
*/

#ifndef _MissionTypesPathManager_
#define _MissionTypesPathManager_

#include <glassy_msgs/msg/mission_info.hpp>
#include <vector>
#include <string>
using namespace glassy_msgs::msg;

const std::vector<uint8_t> MissionTypesPathManager = {
    MissionInfo::PATH_FOLLOWING
};


#endif