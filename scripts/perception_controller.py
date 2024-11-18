#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from asl_tb3_lib.control import BaseController
from asl_tb3_lib.math_utils import wrap_angle
from asl_tb3_msgs.msg import TurtleBotControl, TurtleBotState

# import the message type to use
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist


class PerceptionController(BaseController):
    def __init__(self) -> None:
				# initialize base class (must happen before everything else)
        super().__init__("PerceptionController")
        # self.kp = 2.0
        self.prev_time = self.get_clock().now().nanoseconds / 1e9
        # self.prev_detector_time = self.get_clock().now().nanoseconds / 1e9
        self.prev_active = False
        self.enable_detector = True
        # Declares a parameter named 'my_int' and sets it to a default value of 7
        self.declare_parameter("kp", 2.0)
        # Set a parameter named 'my_float' to 7.0
        self.declare_parameter("active", True)
        
        self.detector_sub = self.create_subscription(Bool, "/detector_bool", self.detector_callback, 10)


				

		# 		# create publisher with: self.create_publisher(<msg type>, <topic>, <qos>)
        # self.hb_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        
        # # create a timer with: self.create_timer(<second>, <callback>)
        # self.hb_timer = self.create_timer(0.2, self.hb_callback)

        # self.motor_sub = self.create_subscription(Bool, "/kill", self.kill_callback, 10)
         
    @property
    def kp(self) -> float:
        """ Get real-time parameter value of maximum velocity

        Returns:
            float: latest parameter value of maximum velocity
        """
        return self.get_parameter("kp").value
    @property
    def active(self) -> bool:
        """ Get real-time parameter value of maximum velocity

        Returns:
            float: latest parameter value of maximum velocity
        """
        return self.get_parameter("active").value
    # def hb_callback(self) -> None:
				
    #     # construct PerceptionController message
    #     msg = Twist()
        
    #     msg.linear.x = 12.0
    #     msg.linear.y = 8.0
    #     msg.linear.z = 2001.0
        
    #     msg.angular.x = 11.0
    #     msg.angular.y = 14.0
    #     msg.angular.z = 1996.0
        

    #     # publish PerceptionController counter
    #     self.hb_pub.publish(msg)
    
    def compute_control(self
                            ) -> TurtleBotControl:
        """ Compute control given current robot state and goal state

        Returns:
            TurtleBotControl: control command
        """
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        if self.get_parameter("active").value:
            w = 0.5
        else:  
            w = 0.0
        
        if current_time > self.prev_time + 5 and current_time < self.prev_time + 7:
            self.get_logger().info(str(self.get_parameter("active").value))
            w = 0.5
            self.set_parameters([rclpy.Parameter("active", value=True)])
            self.enable_detector = False
        elif current_time > self.prev_time + 7:
            self.prev_time = current_time
            self.set_parameters([rclpy.Parameter("active", value=True)])
            self.enable_detector = True
            
        # if current_time > self.prev_time + 5:
        #     if not self.prev_active:
        #         self.prev_detector_time = current_time
        #     self.set_parameters([rclpy.Parameter("active", value=(not self.active))])
            
        #     self.prev_time = current_time
        #     if self.prev_active:
        #         self.enable_detector = True
        
        # if current_time > self.prev_detector_time + 1:
        #     self.enable_detector = True
        
        # if self.active is not self.prev_active:
        #     self.prev_time = current_time
        #     self.prev_active = self.active
                
        output = TurtleBotControl()
        output.omega = w

        return output

    def detector_callback(self, msg: Bool) -> None:
        """
        Sensor health callback triggered by subscription
        """
        if msg.data and self.enable_detector:
            self.enable_detector = False
            self.prev_time = self.get_clock().now().nanoseconds / 1e9
            self.set_parameters([rclpy.Parameter("active", value=False)])
        
        

if __name__ == "__main__":
    rclpy.init()        # initialize ROS2 context (must run before any other rclpy call)
    node = PerceptionController()  # instantiate the PerceptionController node
    rclpy.spin(node)    # Use ROS2 built-in schedular for executing the node
    rclpy.shutdown()    # cleanly shutdown ROS2 context