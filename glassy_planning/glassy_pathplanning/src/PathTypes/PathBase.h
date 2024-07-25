#ifndef _PathBase_
#define _PathBase_

#include "eigen3/Eigen/Core"
#include <iostream>
#include <stdlib.h>
#include <glassy_utils/GlassyGeneralUtils.h>


class PathBase
{
protected:
    /**
     * @brief data about the path
     */
    Eigen::Vector2d initial_point;
    Eigen::Vector2d final_point;
    Eigen::Vector2d path_dot;
    Eigen::Vector2d path_dot_dot;



public:
    bool is_active=false;
    float curvature;
    void activate(){
        this->is_active = true;
    };
    void deactivate(){
        this->is_active = false;
    };

    PathBase(/* args */){};

    /**
     * @brief Get the closest point on the path to a given point
     * 
     */
    virtual Eigen::Vector2d getClosestPoint(Eigen::Vector2d point) {(void) point; return(Eigen::Vector2d(0.0, 0.0));};


    /**
     * @brief Get the point corresponding to a given gamma (parameterization of the path)
     * 
     * @param gamma
     */
    virtual Eigen::Vector2d getPoint(float gamma){
        (void) gamma;
        return(Eigen::Vector2d(0.0, 0.0));};

    /**
     * @brief Get the derivative of the path at a given point
     *  
     * @param gamma
     */
    virtual Eigen::Vector2d getPathDerivative(float gamma){
        (void) gamma;
        return(Eigen::Vector2d(0.0, 0.0));
    };

    /**
     * @brief Get the second derivative of the path at a given point
     * 
     * @param gamma
     */
    virtual Eigen::Vector2d getPathSecondDerivative(float gamma){
        (void) gamma;
        return(Eigen::Vector2d(0.0, 0.0));
    };


    /**
     * @brief Get the tangent heading of the path at a given point
     * 
     * @param point
     */
    virtual float getTangHeading(Eigen::Vector2d point){
        (void) point;
        return(0.0);};

    /** 
     * @brief Get the curvature of the path at a given point
     * 
     * @param gamma
     */
    virtual float getCurvature(float gamma){(void) gamma; return this->curvature;};


    ~PathBase(){};





};




#endif