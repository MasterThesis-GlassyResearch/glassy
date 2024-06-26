import rclpy
from rclpy.node import Node

import px4_msgs.msg as px4_msgs
import std_srvs.srv as std_srvs

import glassy_msgs.msg as glassy_msgs


# allowed libraries
import numpy as np


class GlassyOpenLoop(Node):
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
        self.thrust_value = self.get_parameter('glassy_openloop_control.actuators.thrust').get_parameter_value().double_value
        self.rudder_value = self.get_parameter('glassy_openloop_control.actuators.rudder').get_parameter_value().double_value
        self.rudder_sin = self.get_parameter('glassy_openloop_control.type.rudder_sinusoidal_wave').get_parameter_value().bool_value

        # Log parameters:
        self.get_logger().info("Thrust: {}".format(self.thrust_value))
        self.get_logger().info("Rudder: {}".format(self.rudder_value))
        self.get_logger().info("Sinusoidal rudder: {}".format(self.rudder_sin))

        

        # create timer
        self.timer_control_ = self.create_timer(1.0/30.0, self.openLoopCommands)
        self.timer_control_.cancel()
        self.is_active=False
    
        self.time_of_last_mission_status_msg_received = None

        self.list_item = 0

        




    def mission_status_subscription_callback(self, msg):
        """
        Checks whether the mission is active or not.
        """
        if self.is_active:
            if msg.mission_mode != glassy_msgs.MissionInfo.OPEN_LOOP:
                self.timer_control_.cancel()
                self.is_active = False
        else:
            if msg.mission_mode == glassy_msgs.MissionInfo.OPEN_LOOP:
                self.timer_control_.reset()
                self.is_active = True


    def openLoopCommands(self):
        """
        Implement the controller for the challenge here.
        (it will run at 30Hz)
        """
        motor = self.motor_value
        rudder = self.rudder_value

        # Fill (rudder should be between [-1,1])
        # currently, the rudder is a sinusoidal function of time, just to test that everything is working
        if(rudder_sin):
            rudder = self.rudder_value*np.sin(self.get_clock().now().nanoseconds/1000000000)


        self.publish_actuators(motor, rudder)



    #
    def publish_actuators(self, motor_value, rudder_value):
        """
        Takes the motor and rudder values. Clips and publishes them.
        """
        #clip values
        motor_value = np.clip(motor_value, 0.0, 1.0)
        rudder_value = np.clip(rudder_value, -1.0, 1.0)

        msg_actuators = glassy_msgs.Actuators()

        msg_actuators.header.stamp = self.get_clock().now().to_msg()
        msg_actuators.thrust = motor_value
        msg_actuators.rudder = rudder_value


        self.actuators_publisher_.publish(msg=msg_actuators)

    

def main(args=None):
    rclpy.init(args=args)

    test = GlassyOpenLoop()

    test.get_logger().info('Glassy OpenLoop node started')
    rclpy.spin(test)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    test.destroy_node()
    rclpy.shutdown()