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

        # Parameters
        # self.map_topic = self.declare_parameter('map_topic', '/map').value
        # self.state_topic = self.declare_parameter('state_topic', '/state').value
        # self.nav_success_topic = self.declare_parameter('nav_success_topic', '/nav_success').value
        # self.to = self.declare_parameter('to', 600.0).value
        
        # self.planned_path_pub = self.create_publisher(Path, "/planned_path", 10)
        # self.smoothed_path_pub = self.create_publisher(Path, "/smoothed_path", 10)


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
        # self.to = 60.0

        # Timer to check exploration timeout
        # self.timer = self.create_timer(1.0, self.exploration_timer_callback)
        # self.start_time = self.get_clock().now()


    def snap_to_grid(state, resolution):
        return resolution * np.round(state / resolution)

    def map_callback(self, msg: OccupancyGrid) -> None:
        # create occupancy map
        self.occupancy = StochOccupancyGrid2D(
            resolution=msg.info.resolution,
            size_xy=np.array([msg.info.width, msg.info.height]),
            origin_xy=np.array([msg.info.origin.position.x, msg.info.origin.position.y]),
            window_size=9,
            probs=msg.data,
        )
        # self.get_logger().info("Map received!")
        self.frontiers = self.explore(self.occupancy)
        if not self.is_planned:
            self.explore_next_frontier()

        # Recalculate frontiers based on the updated map
        

        # # If there are no valid frontiers, log the status
        # if not self.frontiers:
        #     self.get_logger().info('No frontiers available for exploration.')
        #     return

        #ERROR
        # # Trigger replanning if needed
        # if self.is_planned and not all([self.occupancy.is_free(state) for state in self.plan.path[1:]]):
        #     self.is_planned = False
        #     self.get_logger().info('Replanning due to new map update.')
        #     self.explore_next_frontier()

    def state_callback(self, msg):
        # Update state
        self.state = msg

    def nav_success_callback(self, msg):
        self.nav_success = msg.data
        if self.nav_success:
            self.get_logger().info('Navigation succeeded. Exploring next frontier.')
            self.is_planned = False
            self.explore_next_frontier()

    def explore(self, occupancy):
        # Copied and pasted from HW4 - P2
        window_size = occupancy.window_size
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
        if (self.frontiers is not None) and (self.state is not None):
            
            distances = np.linalg.norm(self.frontiers - np.array([self.state.x, self.state.y]), axis=1)

            next_frontier = self.frontiers[np.argmin(distances)]
            goal = TurtleBotState()
            goal.x = next_frontier[0]
            goal.y = next_frontier[1]
            goal.theta = np.arctan2(goal.y - self.state.y, goal.x - self.state.x)
            self.goal_pub.publish(goal)
            self.is_planned = True
        else:
            self.get_logger().info("No frontiers found")

    # def exploration_timer_callback(self):
    #     elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
    #     if elapsed_time > self.to:
    #         self.get_logger().info('EXPLORATION TIMEOUT!!')
    #         self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExploration()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
