#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from asl_tb3_lib.control import BaseHeadingController
from asl_tb3_lib.math_utils import wrap_angle
from asl_tb3_msgs.msg import TurtleBotControl, TurtleBotState

# import the message type to use
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist


class HeadingController(BaseHeadingController):
    def __init__(self) -> None:
				# initialize base class (must happen before everything else)
        super().__init__("HeadingController")
        self.kp = 2.0
				

		# 		# create publisher with: self.create_publisher(<msg type>, <topic>, <qos>)
        # self.hb_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        
        # # create a timer with: self.create_timer(<second>, <callback>)
        # self.hb_timer = self.create_timer(0.2, self.hb_callback)

        # self.motor_sub = self.create_subscription(Bool, "/kill", self.kill_callback, 10)
         
    # def hb_callback(self) -> None:
				
    #     # construct HeadingController message
    #     msg = Twist()
        
    #     msg.linear.x = 12.0
    #     msg.linear.y = 8.0
    #     msg.linear.z = 2001.0
        
    #     msg.angular.x = 11.0
    #     msg.angular.y = 14.0
    #     msg.angular.z = 1996.0
        

    #     # publish HeadingController counter
    #     self.hb_pub.publish(msg)
    
    def compute_control_with_goal(self,
        state: TurtleBotState,
        goal: TurtleBotState
    ) -> TurtleBotControl:
        """ Compute control given current robot state and goal state

        Args:
            state (TurtleBotState): current robot state
            goal (TurtleBotState): current goal state

        Returns:
            TurtleBotControl: control command
        """
        err = goal._theta - state._theta
        w = self.kp * err
        output = TurtleBotControl()
        output.omega = w
        return output
        pass

        
    def kill_callback(self, msg: Bool) -> None:
        
        if msg.data:
            self.get_logger().fatal("HeadingController stopped")
            self.hb_timer.cancel()
            msg2 = Twist()
            self.hb_pub.publish(msg2)
        



if __name__ == "__main__":
    rclpy.init()        # initialize ROS2 context (must run before any other rclpy call)
    node = HeadingController()  # instantiate the HeadingController node
    rclpy.spin(node)    # Use ROS2 built-in schedular for executing the node
    rclpy.shutdown()    # cleanly shutdown ROS2 context