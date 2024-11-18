#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from asl_tb3_lib.navigation import BaseNavigator, TrajectoryPlan  # Import TrajectoryPlan here
from asl_tb3_lib.math_utils import wrap_angle
from asl_tb3_lib.grids import StochOccupancyGrid2D
from asl_tb3_msgs.msg import TurtleBotControl, TurtleBotState
import numpy as np
from scipy.interpolate import splev, splrep

class NavigatorNode(BaseNavigator):
    def __init__(self):
        super().__init__(node_name="navigator")
        
        self.kp = 2.0

        self.kpx = 2.0
        self.kdx = 2.0
        self.kpy = 2.0
        self.kdy = 2.0

        self.V_PREV_THRES = 0.0001

    def compute_heading_control(self, state: TurtleBotState, goal: TurtleBotState) -> TurtleBotControl:
        """ Compute heading control based on the goal state. """
        control = TurtleBotControl()

        heading_error = wrap_angle(goal.theta - state.theta)
        
        control.omega = self.kp * heading_error

        return control

    def compute_trajectory_tracking_control(self, state: TurtleBotState, plan: TrajectoryPlan, t: float) -> TurtleBotControl:
        """ Compute control target using a trajectory tracking controller. """
        control = TurtleBotControl()
        dt = t - self.t_prev
        
        x_d = splev(t, plan.path_x_spline, der=0)
        xd_d = splev(t, plan.path_x_spline, der=1)
        xdd_d = splev(t, plan.path_x_spline, der=2)
        
        y_d = splev(t, plan.path_y_spline, der=0)
        yd_d = splev(t, plan.path_y_spline, der=1)
        ydd_d = splev(t, plan.path_y_spline, der=2)

        u1 = xdd_d + self.kpx * (x_d - state.x) + self.kdx * (xd_d - self.V_prev * np.cos(state.theta))
        u2 = ydd_d + self.kpy * (y_d - state.y) + self.kdy * (yd_d - self.V_prev * np.sin(state.theta))

        V_prev = max(self.V_prev, self.V_PREV_THRES)
        J = np.array([[np.cos(state.theta), -V_prev * np.sin(state.theta)],
                       [np.sin(state.theta), V_prev * np.cos(state.theta)]])
        J_inv = np.linalg.inv(J)
        a, om = J_inv @ np.array([u1, u2])
        V = V_prev + a * dt

        self.t_prev = t
        self.V_prev = V
        self.om_prev = om

        control.v = V
        control.omega = om
        return control

    def compute_trajectory_plan(self, state: TurtleBotState, goal: TurtleBotState, occupancy: StochOccupancyGrid2D, resolution: float, horizon: float):
        """ Compute a trajectory plan using A* and cubic spline fitting. """
        a_star = AStar(
            statespace_lo=(state.x - horizon, state.y - horizon),
            statespace_hi=(state.x + horizon, state.y + horizon),
            x_init=(state.x, state.y),
            x_goal=(goal.x, goal.y),
            occupancy=occupancy,
            resolution=resolution
        )

        v_desired = 0.15
        spline_alpha = 0.05

        if not a_star.solve() or len(a_star.path) < 4:
            return None

        self.V_prev = 0.0
        self.om_prev = 0.0
        self.t_prev = 0.0

        path = np.array(a_star.path)

        distances = np.zeros(len(path) - 1)
        for i in range(len(path) - 1):
            distances[i] = np.linalg.norm(np.array(path[i]) - np.array(path[i+1]))
        ts = np.zeros(len(path))
        for i in range(1, len(path)):
            ts[i] = ts[i-1] + (distances[i - 1] / v_desired)

        # Fit cubic splines to the x and y coordinates with respect to ts
        path_x_spline = splrep(ts, path[:, 0], s=spline_alpha)
        path_y_spline = splrep(ts, path[:, 1], s=spline_alpha)
        ###### YOUR CODE END HERE ######
        
        return TrajectoryPlan(
            path=path,
            path_x_spline=path_x_spline,
            path_y_spline=path_y_spline,
            duration=ts[-1],
        )

class AStar(object):
    """Represents a motion planning problem to be solved using A*"""

    def __init__(self, statespace_lo, statespace_hi, x_init, x_goal, occupancy, resolution=1):
        self.statespace_lo = statespace_lo
        self.statespace_hi = statespace_hi
        self.occupancy = occupancy
        self.resolution = resolution 
        self.x_offset = x_init                     
        self.x_init = self.snap_to_grid(x_init)
        self.x_goal = self.snap_to_grid(x_goal) 

        self.closed_set = set()
        self.open_set = set() 

        self.est_cost_through = {} 
        self.cost_to_arrive = {} 
        self.came_from = {}   

        self.open_set.add(self.x_init)
        self.cost_to_arrive[self.x_init] = 0
        self.est_cost_through[self.x_init] = self.distance(self.x_init,self.x_goal)

        self.path = None    

    def is_free(self, x):
        """
        Checks if a give state x is free, meaning it is inside the bounds of the map and
        is not inside any obstacle.
        Inputs:
            x: state tuple
        Output:
            Boolean True/False
        Hint: self.occupancy is a DetOccupancyGrid2D object, take a look at its methods for what might be
              useful here
        """
        ########## Code starts here ##########
        x = np.asarray(x)
    
        # If the state is out of bounds, return False immediately
        if np.any(x < self.statespace_lo) or np.any(x > self.statespace_hi):
            return False

        # If the state is in an obstacle, return False
        if not self.occupancy.is_free(x):
            return False

        # If both conditions pass, the state is free
        return True
        ########## Code ends here ##########

    def distance(self, x1, x2):
        """
        Computes the Euclidean distance between two states.
        Inputs:
            x1: First state tuple
            x2: Second state tuple
        Output:
            Float Euclidean distance

        HINT: This should take one line. Tuples can be converted to numpy arrays using np.array().
        """
        ########## Code starts here ##########
        return np.linalg.norm(np.array(x1) - np.array(x2))
        ########## Code ends here ##########

    def snap_to_grid(self, x):
        """ Returns the closest point on a discrete state grid
        Input:
            x: tuple state
        Output:
            A tuple that represents the closest point to x on the discrete state grid
        """
        return (
            self.resolution * round((x[0] - self.x_offset[0]) / self.resolution) + self.x_offset[0],
            self.resolution * round((x[1] - self.x_offset[1]) / self.resolution) + self.x_offset[1],
        )

    def get_neighbors(self, x):
        """
        Gets the FREE neighbor states of a given state x. Assumes a motion model
        where we can move up, down, left, right, or along the diagonals by an
        amount equal to self.resolution.
        Input:
            x: tuple state
        Ouput:
            List of neighbors that are free, as a list of TUPLES

        HINTS: Use self.is_free to check whether a given state is indeed free.
               Use self.snap_to_grid (see above) to ensure that the neighbors
               you compute are actually on the discrete grid, i.e., if you were
               to compute neighbors by adding/subtracting self.resolution from x,
               numerical errors could creep in over the course of many additions
               and cause grid point equality checks to fail. To remedy this, you
               should make sure that every neighbor is snapped to the grid as it
               is computed.
        """
        neighbors = []
        ########## Code starts here ##########
        delta = self.resolution
        movements = [[0, delta], [0, -delta], [delta, 0], [-delta, 0], [delta, delta], [delta, -delta], [-delta, -delta], [-delta, delta]]
        for movement in movements:
            possible_neighbor = (x[0] + movement[0], x[1] + movement[1])
            snapped_neighbor = self.snap_to_grid(possible_neighbor)
            if self.is_free(snapped_neighbor):
                neighbors.append(snapped_neighbor)
        ########## Code ends here ##########
        return neighbors

    def find_best_est_cost_through(self):
        """
        Gets the state in open_set that has the lowest est_cost_through
        Output: A tuple, the state found in open_set that has the lowest est_cost_through
        """
        return min(self.open_set, key=lambda x: self.est_cost_through[x])

    def reconstruct_path(self):
        """
        Use the came_from map to reconstruct a path from the initial location to
        the goal location
        Output:
            A list of tuples, which is a list of the states that go from start to goal
        """
        path = [self.x_goal]
        current = path[-1]
        while current != self.x_init:
            path.append(self.came_from[current])
            current = path[-1]
        return list(reversed(path))

    # def plot_path(self, fig_num=0, show_init_label=True):
    #     """Plots the path found in self.path and the obstacles"""
    #     if not self.path:
    #         return

    #     self.occupancy.plot(fig_num)

    #     solution_path = np.asarray(self.path)
    #     plt.plot(solution_path[:,0],solution_path[:,1], color="green", linewidth=2, label="A* solution path", zorder=10)
    #     plt.scatter([self.x_init[0], self.x_goal[0]], [self.x_init[1], self.x_goal[1]], color="green", s=30, zorder=10)
    #     if show_init_label:
    #         plt.annotate(r"$x_{init}$", np.array(self.x_init) + np.array([.2, .2]), fontsize=16)
    #     plt.annotate(r"$x_{goal}$", np.array(self.x_goal) + np.array([.2, .2]), fontsize=16)
    #     plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.03), fancybox=True, ncol=3)

    #     plt.axis([0, self.occupancy.width, 0, self.occupancy.height])

    # def plot_tree(self, point_size=15):
    #     plot_line_segments([(x, self.came_from[x]) for x in self.open_set if x != self.x_init], linewidth=1, color="blue", alpha=0.2)
    #     plot_line_segments([(x, self.came_from[x]) for x in self.closed_set if x != self.x_init], linewidth=1, color="blue", alpha=0.2)
    #     px = [x[0] for x in self.open_set | self.closed_set if x != self.x_init and x != self.x_goal]
    #     py = [x[1] for x in self.open_set | self.closed_set if x != self.x_init and x != self.x_goal]
    #     plt.scatter(px, py, color="blue", s=point_size, zorder=10, alpha=0.2)

    def solve(self):
        """
        Solves the planning problem using the A* search algorithm. It places
        the solution as a list of tuples (each representing a state) that go
        from self.x_init to self.x_goal inside the variable self.path
        Input:
            None
        Output:
            Boolean, True if a solution from x_init to x_goal was found

        HINTS:  We're representing the open and closed sets using python's built-in
                set() class. This allows easily adding and removing items using
                .add(item) and .remove(item) respectively, as well as checking for
                set membership efficiently using the syntax "if item in set".
        """
        ########## Code starts here ##########
        while self.open_set:
            current = self.find_best_est_cost_through()
            if current == self.x_goal:
                self.path = self.reconstruct_path()
                return True
            self.open_set.remove(current)
            self.closed_set.add(current)
            for neighbor in self.get_neighbors(current):
                if neighbor in self.closed_set:
                    continue
                tentative_cost_to_arrive = self.cost_to_arrive[current] + self.distance(current, neighbor)
                if neighbor not in self.open_set:
                    self.open_set.add(neighbor)
                elif tentative_cost_to_arrive > self.cost_to_arrive.get(neighbor, float('inf')):
                    continue
                self.came_from[neighbor] = current
                self.cost_to_arrive[neighbor] = tentative_cost_to_arrive
                self.est_cost_through[neighbor] = tentative_cost_to_arrive + self.distance(neighbor, self.x_goal)
        return False
        ########## Code ends here ##########

def main():
    rclpy.init()
    node = NavigatorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()




