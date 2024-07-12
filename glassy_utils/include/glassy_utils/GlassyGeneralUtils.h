/*
Developers: João Lehodey - joao.lehodey@tecnico.ulisboa.pt - DSOR/ISR team (Instituto Superior Tecnico) 
*/

#ifndef _GlassyGeneralUtils_
#define _GlassyGeneralUtils_

#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <iostream>


// Angle conversions
template <typename T>
T deg2rad(T degrees);

template <typename T>
T rad2deg(T radians);

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

float wrapToPi(float x);


float wrapToTwoPi(float x);

float clip_val(float, float, float);


Eigen::Vector3d quat_to_euler_ZYX(Eigen::Quaterniond q);

#endif