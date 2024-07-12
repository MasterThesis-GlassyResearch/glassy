#!/usr/bin/env python

import rclpy
from rclpy.node import Node

import px4_msgs.msg as px4_msgs
import std_srvs.srv as std_srvs

import glassy_msgs.msg as glassy_msgs
import glassy_pathgen.glassy_dubins as glassy_dubins
import matplotlib.pyplot as plt
import itertools


# allowed libraries
import numpy as np


class GlassyPathGen(Node):
    def __init__(self):
        """
        Class
        """
        super().__init__('glassy_pathgen', 
            allow_undeclared_parameters=True, 
            automatically_declare_parameters_from_overrides=True)

        # create publishers for torque and thrust setpoints
        self.path_info_publisher = self.create_publisher(glassy_msgs.PathInfo, 'glassy/path_info', 1)

        # create subscriber for the mission status
        self.mission_status_subscriber_ = self.create_subscription(glassy_msgs.MissionInfo, 'glassy/mission_status', self.mission_status_subscription_callback, 1)
        self.state_subscriber_ = self.create_subscription(glassy_msgs.State, 'glassy/state', self.state_callback, 1)


        # get the parameters from the files
        self.rate = self.get_parameter('glassy_pathgen.path_defs.rate').get_parameter_value().double_value
        self.min_radius = self.get_parameter('glassy_pathgen.path_defs.r_min').get_parameter_value().double_value
        self.path_gen_method = self.get_parameter('glassy_pathgen.path_defs.path_type').get_parameter_value().string_value
        self.online_gen = self.get_parameter('glassy_pathgen.path_defs.online_gen').get_parameter_value().bool_value
        self.circuit = np.array(self.get_parameter('glassy_pathgen.waypoints').get_parameter_value().double_array_value).reshape(-1, 3)

        print(self.circuit)

        
        # Log parameters:
        self.get_logger().info("Path gen rate: {}".format(self.rate))
        self.get_logger().info("Min Radius: {}".format(self.min_radius))
        self.get_logger().info("Path Type: {}".format(self.path_gen_method))
        self.get_logger().info("Online Generation: {}".format(self.online_gen))
        self.get_logger().info("Circuit waypoints: {}".format(self.circuit))

        self.is_active=True
        self.number_gates_in_advance = 2

        self.current_gate = 0

        self.gate_size = 1
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        
        self.pathgen_dubins = glassy_dubins.DubinsGenerator(self.min_radius)

        # msgs to be published:
        self.path_msg = glassy_msgs.PathInfo() 
        self.path_msg.path_recalculated = False

        self.needs_to_be_calculated = True


        #----------- Start timers
        self.timer = self.create_timer(1/self.rate, self.PathGeneration)
        # self.timer.cancel()




    def plotPathThroughGoals(self):
        for barrier in self.goals:
            post1 = barrier[:2]+ self.gate_size/2*np.array([np.cos(barrier[2]+np.pi/2), np.sin(barrier[2]+np.pi/2)])
            post2 = barrier[:2]+ self.gate_size/2*np.array([np.cos(barrier[2]-np.pi/2), np.sin(barrier[2]-np.pi/2)])
            plt.plot([post1[0], post2[0]], [post1[1], post2[1]], 'o--', color= 'darkorange')
            plt.legend({"gates"})

        if self.path_gen_method == 'dubins':
            self.pathgen_dubins.DubinsInterpolator(waypoints=self.goals)
            self.pathgen_dubins.full_path_plot()
        




    def state_callback(self, msg):
        """
        Receive the state of the vehicle and check which gates have been crossed
        """
        prev_x = self.x
        prev_y = self.y
        self.x = msg.p_ned[0]
        self.y = msg.p_ned[1]
        self.yaw = msg.yaw


        if self.check_if_waypoint_crossed(np.array([prev_x, prev_y]), np.array([self.x, self.y])):
            if self.current_gate<len(self.circuit)-1:
                self.current_gate=self.current_gate+1
            else:
                self.current_gate=0

        

    def check_if_waypoint_crossed(self, p1, p2):
        #first check orientation of gate compared to p2-p1

        correct_way = False
        p2_min_p1 = p2-p1

        if(np.inner(p2_min_p1, np.array([np.cos(self.circuit[self.current_gate][2]), np.sin(self.circuit[self.current_gate][2])]))>0):
            correct_way=True
        else:
            return False

        heading_g1_g2 = self.circuit[self.current_gate][2]-np.pi/2

        A = np.array([[self.gate_size * np.cos(heading_g1_g2), -p2_min_p1[0]], [self.gate_size * np.sin(heading_g1_g2), -p2_min_p1[1]]])
        b = p1-self.circuit[self.current_gate][:2]+ self.gate_size/2*np.array([np.cos(self.circuit[self.current_gate][2]+np.pi/2), np.sin(self.circuit[self.current_gate][2]+np.pi/2)])
        if(correct_way):
            t = np.linalg.solve(A, b)
        if(min(t)>=0 and max(t)<=1):
            return True

        return False



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

        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        if self.path_gen_method == "dubins":
            if(not self.online_gen and self.needs_to_be_calculated== True):
                self.pathgen_dubins.DubinsInterpolator(np.vstack([np.array([self.x, self.y, self.yaw]), self.circuit]))
                self.path_msg.path_recalculated = True
                self.needs_to_be_calculated = False
            elif(self.online_gen):
                self.pathgen_dubins.DubinsInterpolator(np.vstack([np.array([self.x, self.y, self.yaw]), np.take(self.circuit, range(self.current_gate*3, self.current_gate*3+3*self.number_gates_in_advance), mode='wrap').reshape(2, 3)]))
                self.get_logger().info('wrapping')
                self.path_msg.path_recalculated = True

            self.path_msg.path_segments = self.pathgen_dubins.full_path_type
            self.path_msg.path_segment_info=list(np.concatenate(self.pathgen_dubins.full_path_info))
        else:
            self.path_msg.path_recalculated = False

        self.path_info_publisher.publish(msg=self.path_msg)



def main(args=None):
    print('starting')
    rclpy.init(args=args)

    test = GlassyPathGen()

    test.get_logger().info('Glassy Path Gen started')
    rclpy.spin(test)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    test.destroy_node()
    rclpy.shutdown()
    # test.plotPathThroughGoals()
    # plt.show()

if __name__ == '__main__':
    main()