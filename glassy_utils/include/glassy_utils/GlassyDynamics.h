/*
Developers: João Lehodey - joao.lehodey@tecnico.ulisboa.pt - DSOR/ISR team (Instituto Superior Tecnico) 
*/

#ifndef _GlassyDynamics_
#define _GlassyDynamics_

#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include "glassy_msgs/msg/state.hpp"



Eigen::Vector2d getActuatorsFromDesiredAccelerations(float surge_force, float yaw_rate_force, glassy_msgs::msg::State::SharedPtr state);
#endif