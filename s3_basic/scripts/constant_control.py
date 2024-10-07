#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# import the message type to use
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist


class Heartbeat(Node):
    def __init__(self) -> None:
				# initialize base class (must happen before everything else)
        super().__init__("heartbeat")
				

				# create publisher with: self.create_publisher(<msg type>, <topic>, <qos>)
        self.hb_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        
        # create a timer with: self.create_timer(<second>, <callback>)
        self.hb_timer = self.create_timer(0.2, self.hb_callback)

        self.motor_sub = self.create_subscription(Bool, "/kill", self.kill_callback, 10)
         
    def hb_callback(self) -> None:
				
        # construct heartbeat message
        msg = Twist()
        
        msg.linear.x = 2.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        
        

        # publish heartbeat counter
        self.hb_pub.publish(msg)
        
    def kill_callback(self, msg: Bool) -> None:
        
        if msg.data:
            self.get_logger().fatal("Heartbeat stopped")
            self.hb_timer.cancel()
            msg2 = Twist()
            self.hb_pub.publish(msg2)
        



if __name__ == "__main__":
    rclpy.init()        # initialize ROS2 context (must run before any other rclpy call)
    node = Heartbeat()  # instantiate the heartbeat node
    rclpy.spin(node)    # Use ROS2 built-in schedular for executing the node
    rclpy.shutdown()    # cleanly shutdown ROS2 context