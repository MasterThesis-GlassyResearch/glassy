/*
Developers: João Lehodey - joao.lehodey@tecnico.ulisboa.pt - DSOR/ISR team (Instituto Superior Tecnico) 
*/

#ifndef _MissionTypes_
#define _MissionTypes_

#include <glassy_msgs/msg/mission_info.hpp>
#include <vector>
#include <string>
using namespace glassy_msgs::msg;

const std::vector<uint8_t> MissionTypes = {
    MissionInfo::MISSION_OFF, 
    MissionInfo::PATH_FOLLOWING,
    MissionInfo::TRAJECTORY_TRACKING,
    MissionInfo::OPEN_LOOP
};

const std::vector<std::string> MissionNames = {
    "MISSION_OFF",
    "PATH_FOLLOWING",
    "TRAJECTORY_TRACKING",
    "OPEN_LOOP"
};

#endif