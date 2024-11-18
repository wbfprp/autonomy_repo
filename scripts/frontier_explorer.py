#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from asl_tb3_msgs.msg import TurtleBotState, TurtleBotControl
from asl_tb3_lib.grids import snap_to_grid, StochOccupancyGrid2D
from scipy.signal import convolve2d
import numpy as np



class FrontierExploration(Node):
    def __init__(self):
        super().__init__('frontier_explorer')


        # Subscribers
        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self.map_callback, 10)
        self.state_sub = self.create_subscription(TurtleBotState, "/state", self.state_callback, 10)
        self.nav_success_sub = self.create_subscription(Bool, "/nav_success", self.nav_success_callback, 10)

        # Publishers
        self.goal_pub = self.create_publisher(TurtleBotState, "/cmd_nav", 10)

        # Variables
        self.occupancy = None
        self.state = None
        self.frontiers = []
        self.nav_success = True
        self.is_planned = False
        self.plan = None

    def map_callback(self, msg: OccupancyGrid) -> None:
        # create occupancy map
        self.occupancy = StochOccupancyGrid2D(
            resolution=msg.info.resolution,
            size_xy=np.array([msg.info.width, msg.info.height]),
            origin_xy=np.array([msg.info.origin.position.x, msg.info.origin.position.y]),
            window_size=7,
            probs=msg.data,
        )
        # self.get_logger().info("Map received!")
        
    def state_callback(self, msg):
        # Update state
        self.state = msg

    def nav_success_callback(self, msg):
        self.nav_success = msg.data
        
        # if self.nav_success:
        self.frontiers = self.explore(self.occupancy)
        # if not self.is_planned:
        self.explore_next_frontier()
        self.get_logger().info('Navigation succeeded. Exploring next frontier.')
        self.is_planned = False

    def explore(self, occupancy):
        # Copied and pasted from HW4 - P2
        window_size = 13
        probs = occupancy.probs
        kernel = np.ones((window_size, window_size))

        mask_unknown = (probs == -1)
        mask_occupied = (probs >= 0.5) & (probs != -1)
        mask_free = (probs < 0.5) & (probs != -1)

        total_cells = convolve2d(np.ones_like(probs), kernel, mode='same', boundary='fill')
        unknown_cnt = convolve2d(mask_unknown.astype(int), kernel, mode='same', boundary='fill')
        percent_unknown = unknown_cnt / total_cells
        free_cnt = convolve2d(mask_free.astype(int), kernel, mode='same', boundary='fill')
        percent_free = free_cnt / total_cells
        occupied_cnt = convolve2d(mask_occupied.astype(int), kernel, mode='same', boundary='fill')

        frontier_cells = (percent_unknown >= 0.2) & (occupied_cnt == 0) & (percent_free >= 0.3)
        frontier_idx = np.argwhere(frontier_cells)
        frontier_states = occupancy.grid2state(frontier_idx[:, [1, 0]])
        
        if len(frontier_idx) == 0:
            self.get_logger().warn("In explore(): no frontier")

        return frontier_states

    def explore_next_frontier(self):
        # move robot to the closest frontier
        self.get_logger().info("e_n_f called")
        if (self.frontiers is not None) and (self.state is not None):
            self.get_logger().info("inside")
            
            distances = np.linalg.norm(self.frontiers - np.array([self.state.x, self.state.y]), axis=1)

            next_frontier = self.frontiers[np.argmin(distances)]
            goal = TurtleBotState()
            goal.x = next_frontier[0]
            goal.y = next_frontier[1]
            # goal.theta = np.arctan2(goal.y - self.state.y, goal.x - self.state.x)
            goal.theta = 0.0
            self.goal_pub.publish(goal)
            self.is_planned = True
        else:
            self.get_logger().info("No frontiers found")
            rclpy.shutdown()



def main(args=None):
    rclpy.init(args=args)
    node = FrontierExploration()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
