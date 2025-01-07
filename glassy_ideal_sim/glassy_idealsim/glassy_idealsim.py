
import rclpy
from rclpy.node import Node

import px4_msgs.msg as px4_msgs
import std_srvs.srv as std_srvs

import glassy_msgs.msg as glassy_msgs


# allowed libraries
import numpy as np


# This class will be used to simulate the system in the ideal case, replacinf either the real system or the gazebo simulation.
# It will be used to test the control algorithms in the ideal case, before testing them in the real system.
# It will publish the state, and subscribe to the control commands. It also publishes the mission status

class GlassyIdealSim(Node):
    def __init__(self):
        """
        Class
        """
        super().__init__('glassy_openloop', 
            allow_undeclared_parameters=False,
            automatically_declare_parameters_from_overrides=False)
        # create publishers for mission stattus and state
        self.mission_status_publisher_ = self.create_publisher(glassy_msgs.MissionInfo, 'glassy/mission_status', 1)
        self.state_publisher_ = self.create_publisher(glassy_msgs.State, 'glassy/state', 1)


        # define a subscriber 
        self.actuator_subscription_ = self.create_subscription(glassy_msgs.Actuators, 'glassy/actuators', self.actuator_subscription_callback, 1)

        # get the parameters from the files (sensor noises)
        '''    glassy_ideal_sim:
      rate: 100
      noise:
        x_noise: 0.0
        y_noise: 0.0
        yaw_noise: 0.0
        u_noise: 0.0
        v_noise: 0.0
        r_noise: 0.0'''

        # declare all the parameters
        self.declare_parameter('glassy_ideal_sim.rate', 50.0)
        self.declare_parameter('glassy_ideal_sim.noise.x_noise', 0.0)
        self.declare_parameter('glassy_ideal_sim.noise.y_noise', 0.0)
        self.declare_parameter('glassy_ideal_sim.noise.yaw_noise', 0.0)
        self.declare_parameter('glassy_ideal_sim.noise.u_noise', 0.0)
        self.declare_parameter('glassy_ideal_sim.noise.v_noise', 0.0)
        self.declare_parameter('glassy_ideal_sim.noise.r_noise', 0.0)


        self.rate = self.get_parameter('glassy_ideal_sim.rate').get_parameter_value().double_value
        self.x_noise = self.get_parameter('glassy_ideal_sim.noise.x_noise').get_parameter_value().double_value
        self.y_noise = self.get_parameter('glassy_ideal_sim.noise.y_noise').get_parameter_value().double_value
        self.yaw_noise = self.get_parameter('glassy_ideal_sim.noise.yaw_noise').get_parameter_value().double_value
        self.u_noise = self.get_parameter('glassy_ideal_sim.noise.u_noise').get_parameter_value().double_value
        self.v_noise = self.get_parameter('glassy_ideal_sim.noise.v_noise').get_parameter_value().double_value
        self.r_noise = self.get_parameter('glassy_ideal_sim.noise.r_noise').get_parameter_value().double_value



        # Log parameters:
        self.get_logger().info('Rate: {}'.format(self.rate))
        self.get_logger().info('X noise: {}'.format(self.x_noise))
        self.get_logger().info('Y noise: {}'.format(self.y_noise))
        self.get_logger().info('Yaw noise: {}'.format(self.yaw_noise))
        self.get_logger().info('U noise: {}'.format(self.u_noise))
        self.get_logger().info('V noise: {}'.format(self.v_noise))
        self.get_logger().info('R noise: {}'.format(self.r_noise))



        self.msg_state = glassy_msgs.State()
        self.mission_status_msg = glassy_msgs.MissionInfo()
        self.mission_status_msg.mission_mode = glassy_msgs.MissionInfo.PATH_FOLLOWING


        # create timer
        self.timer_control_ = self.create_timer(1.0/self.rate, self.updateState)

        # create timer for mission status
        self.timer_mission_status_ = self.create_timer(1.0/10.0, self.mission_status_timer_callback)

        # initialize thrust and rudder
        self.thrust = 0.0
        self.rudder = 0.0    

        # initialize the state
        self.u = 0.0
        self.v = 0.0
        self.r = 0.0

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        



    def actuator_subscription_callback(self, msg):
        """
        Takes the control commands and stores them in the class variables. Limits them between reasonable values.
        """
        self.thrust = msg.thrust
        self.rudder = msg.rudder

        # limit the values
        if self.thrust > 10.0:
            self.thrust = 10.0
        elif self.thrust < 0.0:
            self.thrust = 0.0
        
        if self.rudder > 1.0:
            self.rudder = 1.0
        elif self.rudder < -1.0:
            self.rudder = -1.0


    def mission_status_timer_callback(self):
        """
        Checks whether the mission is active or not.
        """
        # self.mission_status_msg.header.stamp = self.get_clock().now().to_msg()
        self.mission_status_publisher_.publish(msg=self.mission_status_msg)



    def updateState(self):
        """
        Implement the controller for the challenge here.
        (it will run at 30Hz)
        """
        Et = 16.4
        
        a = [1.1965, -0.6218, -0.0216, 16.4, 0.0976, 0.5056, -0.1154, -0.0025]
        b = [-0.1885, -4.4450, -0.1937]
        c = [2.1225, -0.8592, -0.0963, 2.2910, -5.5000, -1.9001, 0.0123]
        angle_params = [60.0 * np.pi/180.0] # degrees


        # calculate the surge, sway and yaw rate accelerations
        # u = a1vr + a2u + a3u2 + ET · uT + a7u|r| + a8u2|r|
        # ˙v = b1ru + b2v + b3v|v| + b4u · sin(δrud) + b5u2 · sin(δrud)
        # ˙r = c1vu + c2r + c3r|r| + c4u2 · sin(δrud) + c5 · sin(δrud) · uT + c6ur + c7u2
        u_dot = a[0]*self.r*self.v + a[1]*self.u + a[2]*self.u**2 + Et*self.thrust + a[6]*self.u*np.abs(self.r) + a[7]*self.u**2*np.abs(self.r)
        v_dot = b[0]*self.r*self.u + b[1]*self.v + b[2]*self.v*np.abs(self.v) 
        r_dot = c[0]*self.v*self.u + c[1]*self.r + c[2]*self.r*np.abs(self.r) + c[3]*self.u**2*np.sin(self.rudder*angle_params[0]) + c[4]*np.sin(self.rudder*angle_params[0])*self.thrust + c[5]*self.u*self.r + c[6]*self.u**2

        # get accelerations in inertial frame
        x_dot = self.u*np.cos(self.yaw) - self.v*np.sin(self.yaw)
        y_dot = self.u*np.sin(self.yaw) + self.v*np.cos(self.yaw)
        yaw_dot = self.r

        # update the state
        self.u = self.u + u_dot * 1.0/self.rate
        self.v = self.v + v_dot * 1.0/self.rate
        self.r = self.r + r_dot * 1.0/self.rate

        self.x = self.x + x_dot * 1.0/self.rate
        self.y = self.y + y_dot * 1.0/self.rate
        self.yaw = self.yaw + yaw_dot * 1.0/self.rate

        # add noise to the state
        self.u += np.random.normal(0, self.u_noise)
        self.v += np.random.normal(0, self.v_noise)
        self.r += np.random.normal(0, self.r_noise)
        self.x += np.random.normal(0, self.x_noise)
        self.y += np.random.normal(0, self.y_noise)
        self.yaw += np.random.normal(0, self.yaw_noise)

        # fill state msg
        self.msg_state.header.stamp = self.get_clock().now().to_msg()
        self.msg_state.p_ned[0] = self.x
        self.msg_state.p_ned[1] = self.y
        self.msg_state.p_ned[2] = 0.0
        self.msg_state.yaw = self.yaw
        self.msg_state.v_body[0] = self.u
        self.msg_state.v_body[1] = self.v
        self.msg_state.v_body[2] = 0.0
        self.msg_state.yaw_rate = self.r

        # publish the state
        self.state_publisher_.publish(msg=self.msg_state)


    

def main(args=None):
    rclpy.init(args=args)

    test = GlassyIdealSim()

    test.get_logger().info('Glassy OpenLoop node started')
    rclpy.spin(test)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    test.destroy_node()
    rclpy.shutdown()