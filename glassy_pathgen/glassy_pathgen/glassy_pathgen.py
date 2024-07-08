#!/usr/bin/env python

import rclpy
from rclpy.node import Node

import px4_msgs.msg as px4_msgs
import std_srvs.srv as std_srvs

import glassy_msgs.msg as glassy_msgs
import glassy_dubins
import matplotlib.pyplot as plt



# allowed libraries
import numpy as np


class GlassyPathGen(Node):
    def __init__(self):
        """
        Class
        """
        super().__init__('glassy_openloop', 
            allow_undeclared_parameters=True, 
            automatically_declare_parameters_from_overrides=True)

        # create publishers for torque and thrust setpoints
        self.actuators_publisher_ = self.create_publisher(glassy_msgs.Actuators, 'glassy/actuators', 1)

        # create subscriber for the mission status
        self.mission_status_subscriber_ = self.create_subscription(glassy_msgs.MissionInfo, 'glassy/mission_status', self.mission_status_subscription_callback, 1)

        # get the parameters from the files
        self.rate = self.get_parameter('glassy_pathgen.rate').get_parameter_value().double_value
        self.min_radius = self.get_parameter('glassy_pathgen.min_radius').get_parameter_value().double_value
        self.path_gen_method = self.get_parameter('glassy_pathgen.method').get_parameter_value().double_value
        self.generate_once = self.get_parameter('glassy_pathgen.method').get_parameter_value().bool_value

        self.rate = 1
        
        # Log parameters:
        self.get_logger().info("Path gen rate: {}".format(self.rate))
        self.get_logger().info("Min Radius: {}".format(self.min_radius))

        self.is_active=False

        self.timer = self.create_timer(1/self.rate, self.PathGeneration)

        self.goal_size = 1
        self.goals = [np.array([0, 0, 0]), np.array([4, 0, np.pi]), np.array([10, 10, 0]), np.array([0, 10, -np.pi/2]), np.array([0, 0, 0])]
        self.goals = [np.array([0, 0, 0]), np.array([4, 0, np.pi]), np.array([10, 10, 0]), np.array([0, 10, -np.pi/2]), np.array([0, 0, 0])]
        self.goals = [np.array([0, 0, 0]), np.array([4, 0, np.pi]), np.array([10, 10, 0]), np.array([0, 10, -np.pi/2]), np.array([0, 0, 0])]

        
        # if(self.path_gen_method=='dubins'):
        # self.pathgen_dubins = glassy_dubins.DubinsGenerator(1)
        # self.pathgen_dubins = glassy_dubins.DubinsGenerator(2)
        # self.pathgen_dubins = glassy_dubins.DubinsGenerator(3)
        self.pathgen_dubins = glassy_dubins.DubinsGenerator(4)
        # self.pathgen_dubins = glassy_dubins.DubinsGenerator(5)



    def plotPathThroughGoals(self):
        for barrier in self.goals:
            post1 = barrier[:2]+ self.goal_size/2*np.array([np.cos(barrier[2]+np.pi/2), np.sin(barrier[2]+np.pi/2)])
            post2 = barrier[:2]+ self.goal_size/2*np.array([np.cos(barrier[2]-np.pi/2), np.sin(barrier[2]-np.pi/2)])
            plt.plot([post1[0], post2[0]], [post1[1], post2[1]], 'o--', color= 'darkorange')
            plt.legend({"gates"})

        self.pathgen_dubins.DubinsInterpolator(waypoints=self.goals)
        self.pathgen_dubins.full_path_plot()
        





    def mission_status_subscription_callback(self, msg):
        """
        Checks whether the mission is active or not.
        """
        if self.is_active:
            if msg.mission_mode != glassy_msgs.MissionInfo.PATH_FOLLOWING:
                self.timer.cancel()
                self.is_active = False
        else:
            if msg.mission_mode == glassy_msgs.MissionInfo.PATH_FOLLOWING:
                self.timer.reset()
                self.is_active = True


    def PathGeneration(self):
        """
        Generate the path and publish the message
        """
        # if not self.is_active:
        #     return


        pass


def main(args=None):
    rclpy.init(args=args)


    test = GlassyPathGen()

    # test.get_logger().info('Glassy Path Gen started')
    # rclpy.spin(test)

    # # Destroy the node explicitly
    # # (optional - otherwise it will be done automatically
    # # when the garbage collector destroys the node object)
    # test.destroy_node()
    # rclpy.shutdown()
    test.plotPathThroughGoals()
    plt.show()

if __name__ == '__main__':
    main()