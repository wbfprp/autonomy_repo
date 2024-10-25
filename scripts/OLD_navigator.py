#!/usr/bin/env python3

import numpy as np
from scipy.interpolate import splev, splrep
from asl_tb3_lib.navigation import BaseNavigator
from asl_tb3_lib.control import BaseController

from asl_tb3_lib.math_utils import wrap_angle
from asl_tb3_lib.grids import StochOccupancyGrid2D
from asl_tb3_msgs.msg import TurtleBotState, TurtleBotControl
from asl_tb3_lib.navigation import TrajectoryPlan
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import rclpy

# class AStar(object):
#     """Motion planning problem to be solved using A*"""

#     def __init__(self, statespace_lo, statespace_hi, x_init, x_goal, occupancy, resolution=1):
#         self.statespace_lo = statespace_lo         
#         self.statespace_hi = statespace_hi         
#         self.occupancy = occupancy                
#         self.resolution = resolution               
#         self.x_offset = x_init                     
#         self.x_init = self.snap_to_grid(x_init)    
#         self.x_goal = self.snap_to_grid(x_goal)    

#         self.closed_set = set()    
#         self.open_set = set()      

#         self.est_cost_through = {}  
#         self.cost_to_arrive = {}   
#         self.came_from = {}         

#         self.open_set.add(self.x_init)
#         self.cost_to_arrive[self.x_init] = 0
#         self.est_cost_through[self.x_init] = self.distance(self.x_init,self.x_goal)

#         self.path = None        

#     def is_free(self, x):
#         for i in range(0, len(x)):
#             if x[i] > self.statespace_hi[i] or x[i] < self.statespace_lo[i]:
#                 return False
#         return self.occupancy.is_free(x)

#     def distance(self, x1, x2):
#         return np.linalg.norm(np.array(x2) - np.array(x1))

#     def snap_to_grid(self, x):
#         return (
#             self.resolution * round((x[0] - self.x_offset[0]) / self.resolution) + self.x_offset[0],
#             self.resolution * round((x[1] - self.x_offset[1]) / self.resolution) + self.x_offset[1],
#         )

#     def get_neighbors(self, x):
#         neighbors = []
#         if not self.is_free(x):
#             return neighbors
#         dirs = np.array([[-1, -1], [-1, 0], [-1, 1], [0, -1], [0, 1], [1, -1], [1, 0], [1, 1]])
#         xnparr = np.array(x)
#         for i in range(0, len(dirs)):
#             if self.is_free(tuple(xnparr + dirs[i])):
#                 neighbors.append(tuple(xnparr + dirs[i]))
#         return neighbors

#     def find_best_est_cost_through(self):
#         return min(self.open_set, key=lambda x: self.est_cost_through[x])

#     def reconstruct_path(self):
#         path = [self.x_goal]
#         current = path[-1]
#         while current != self.x_init:
#             path.append(self.came_from[current])
#             current = path[-1]
#         return list(reversed(path))

#     def solve(self):
#         self.open_set.add(self.x_init)
#         self.cost_to_arrive[self.x_offset] = 0
#         self.est_cost_through[self.x_init] = self.distance(self.x_init, self.x_goal)

#         while len(self.open_set) > 0:
#             x_current = self.find_best_est_cost_through()
            
#             if x_current == self.x_goal:
#                 self.path = self.reconstruct_path()
#                 return True
            
#             self.open_set.remove(x_current)
#             self.closed_set.add(x_current)

#             for neighbor in self.get_neighbors(x_current):
#                 if neighbor in self.closed_set:
#                     continue  

#                 tentative_cost_to_arrive = self.cost_to_arrive[x_current] + self.distance(x_current, neighbor)
                
#                 if neighbor not in self.open_set:
#                     self.open_set.add(neighbor)  
#                 elif tentative_cost_to_arrive > self.cost_to_arrive.get(neighbor, float('inf')):
#                     continue  

#                 self.came_from[neighbor] = x_current
#                 self.cost_to_arrive[neighbor] = tentative_cost_to_arrive
#                 self.est_cost_through[neighbor] = tentative_cost_to_arrive + self.distance(neighbor, self.x_goal)

#         return False
class AStar(object):
    """Represents a motion planning problem to be solved using A*"""

    def __init__(self, statespace_lo, statespace_hi, x_init, x_goal, occupancy, resolution=1):
        self.statespace_lo = statespace_lo         # state space lower bound (e.g., [-5, -5])
        self.statespace_hi = statespace_hi         # state space upper bound (e.g., [5, 5])
        self.occupancy = occupancy                 # occupancy grid (a DetOccupancyGrid2D object)
        self.resolution = resolution               # resolution of the discretization of state space (cell/m)
        self.x_offset = x_init                     
        self.x_init = self.snap_to_grid(x_init)    # initial state
        self.x_goal = self.snap_to_grid(x_goal)    # goal state

        self.closed_set = set()    # the set containing the states that have been visited
        self.open_set = set()      # the set containing the states that are condidate for future expension

        self.est_cost_through = {}  # dictionary of the estimated cost from start to goal passing through state (often called f score)
        self.cost_to_arrive = {}    # dictionary of the cost-to-arrive at state from start (often called g score)
        self.came_from = {}         # dictionary keeping track of each state's parent to reconstruct the path

        self.open_set.add(self.x_init)
        self.cost_to_arrive[self.x_init] = 0
        self.est_cost_through[self.x_init] = self.distance(self.x_init,self.x_goal)

        self.path = None        # the final path as a list of states

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
        for i in range(0, len(x)):
            if x[i] > self.statespace_hi[i] or x[i] < self.statespace_lo[i]:
                return False
        #print("jojo")
        return self.occupancy.is_free(np.array(x))
        raise NotImplementedError("is_free not implemented")
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
        return np.linalg.norm(np.array(x2) - np.array(x1))
        raise NotImplementedError("distance not implemented")
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
        if not self.is_free(x):
            return neighbors
        dirs = np.array([[-1, -1], [-1, 0], [-1, 1], [0, -1], [0, 1], [1, -1], [1, 0], [1, 1]])
        xnparr = np.array(x)
        for i in range(0, len(dirs)):
            if self.is_free(tuple(xnparr + dirs[i])):
                neighbors.append(tuple(xnparr + dirs[i]))
        return neighbors
        raise NotImplementedError("get_neighbors not implemented")
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
        self.open_set.add(self.x_init)
        self.cost_to_arrive[self.x_offset] = 0
        self.est_cost_through[self.x_init] = self.distance(self.x_init, self.x_goal)

        while len(self.open_set) > 0:
            #print("in the loop")
            
            x_current = self.find_best_est_cost_through()
            
            if x_current == self.x_goal:
                self.path = self.reconstruct_path()
                return True
            #print(x_current)
            self.open_set.remove(x_current)
            self.closed_set.add(x_current)

            for neighbor in self.get_neighbors(x_current):
                #print("in the loop")
                if neighbor in self.closed_set:
                    continue  

                tentative_cost_to_arrive = self.cost_to_arrive[x_current] + self.distance(x_current, neighbor)
                
                if neighbor not in self.open_set:
                    self.open_set.add(neighbor)  
                    #print("in the loop3")
                elif tentative_cost_to_arrive > self.cost_to_arrive.get(neighbor, float('inf')):
                    continue  

                
                self.came_from[neighbor] = x_current
                self.cost_to_arrive[neighbor] = tentative_cost_to_arrive
                self.est_cost_through[neighbor] = tentative_cost_to_arrive + self.distance(neighbor, self.x_goal)
            #print("in the loop2")
        # If we exit the loop without finding the goal, return False
        return False
        raise NotImplementedError("solve not implemented")
        ########## Code ends here ##########

class DetOccupancyGrid2D(object):
    """
    A 2D state space grid with a set of rectangular obstacles. The grid is
    fully deterministic
    """
    def __init__(self, width, height, obstacles):
        self.width = width
        self.height = height
        self.obstacles = obstacles

    def is_free(self, x):
        """Verifies that point is not inside any obstacles by some margin"""
        for obs in self.obstacles:
            if x[0] >= obs[0][0] - self.width * .01 and \
               x[0] <= obs[1][0] + self.width * .01 and \
               x[1] >= obs[0][1] - self.height * .01 and \
               x[1] <= obs[1][1] + self.height * .01:
                return False
        return True



class Navigator(BaseNavigator):
    def __init__(self):
        super().__init__()
        self.Kp = 1.0
        self.Kpx = 1.0
        self.Kdx = 0.1
        self.Kpy = 1.0
        self.Kdy = 0.1
        self.V_PREV_THRES = 0.001
        self.reset()
    
    def reset(self) -> None:
        self.V_prev = 0.
        self.om_prev = 0.
        self.t_prev = 0.

    def compute_heading_control(self, 
                                state: TurtleBotState, 
                                goal: TurtleBotState
                                )-> TurtleBotControl:
        
        
        err = wrap_angle(goal.theta - state.theta)
        omega = self.Kp * err
        
        turtle_control = TurtleBotControl()
        turtle_control.omega = omega

        return turtle_control

    def compute_trajectory_tracking_control(self, 
                                            state: TurtleBotState, 
                                            plan: TrajectoryPlan, 
                                            t: float) -> TurtleBotControl:
        
        
        x_d, xd_d, xdd_d = splev(t, plan.path_x_spline)
        y_d, yd_d, ydd_d = splev(t, plan.path_y_spline)

        dt = t - self.t_prev
        th = state.theta
        cth = np.cos(th)
        sth = np.sin(th)

        V_prev_safe = max(self.V_prev, self.V_PREV_THRES)
        J = [[cth, - V_prev_safe * th], [sth, V_prev_safe * cth]]

        u1 = xdd_d + self.Kpx * (x_d - state.x) + self.Kdx * (xd_d - self.V_prev * cth)
        u2 = ydd_d + self.Kpy * (y_d - state.y) + self.Kdy * (yd_d - self.V_prev * sth)
        

        uvec = np.array([u1, u2])
        v_dot, om = np.linalg.solve(J, uvec)

        V = self.V_prev + dt * v_dot
        
        
        self.t_prev = t
        self.V_prev = V
        self.om_prev = om

        turtle_control = TurtleBotControl()
        turtle_control.v = V
        turtle_control.omega = om

        return turtle_control

    def compute_trajectory_plan(self, 
                                state: TurtleBotState, 
                                goal: TurtleBotState, 
                                occupancy: StochOccupancyGrid2D, 
                                resolution: float, horizon:float):
        
        
        # Create A* problem and solve it
        # astar_problem = AStar(
        #     statespace_lo= (-horizon, -horizon),
        #     statespace_hi= (horizon, horizon),
        #     x_init=(state.x, state.y),
        #     x_goal=(goal.x, goal.y),
        #     occupancy=occupancy,
        #     resolution=resolution
        # )

        astar_problem = AStar(
            statespace_lo=(state.x - horizon, state.y - horizon),
            statespace_hi=(state.x + horizon, state.y + horizon),
            x_init=(state.x, state.y),
            x_goal=(goal.x, goal.y),
            occupancy=occupancy,
            resolution=resolution
        )
        
        # self.get_logger().info(str(horizon))
        # self.get_logger().info("\n\n\n\nREACHPOINT -1\n\n\n\n")
        # success = astar_problem.solve()

        # self.get_logger().info("\n\n\n\nREACHPOINT 0\n\n\n\n")

        success = astar_problem.solve()

        v_desired = 0.15
        spline_alpha = 0.05
        
        
        if not success or len(astar_problem.path) < 4:
            return None

        self.reset()
        
        # self.get_logger().info("\n\n\n\nREACHPOINT 1\n\n\n\n")
        # total_distance = sum(
        # np.linalg.norm(np.array(astar_problem.path[i + 1]) - np.array(astar_problem.path[i]))
        # for i in range(len(astar_problem.path) - 1)
        # )
        # total_time = total_distance / self.V_prev
        for i in range(len(astar_problem.path) - 1):
            distance = np.linalg.norm(np.array(astar_problem.path[i + 1]) - np.array(astar_problem.path[i]))
        ts = np.zeros(len(astar_problem.path))
        for i in range(1, len(astar_problem.path)):
            ts[i] = ts[i-1] + (distance[i-1] / v_desired)
        
        
        
        # self.get_logger().info("\n\n\n\nREACHPOINT 3\n\n\n\n")
        # x_vals = [p[0] for p in astar_problem.path]
        # y_vals = [p[1] for p in astar_problem.path]

        # self.get_logger().info("\n\n\n\nREACHPOINT 4\n\n\n\n")
        # spline_x = self.create_spline(x_vals)
        # spline_y = self.create_spline(y_vals)

        # self.get_logger().info("\n\n\n\nREACHPOINT 5\n\n\n\n")
        # self.get_logger().info(str(spline_x))
        spline_x = splrep(ts, astar_problem.path[:, 0], s=spline_alpha)
        spline_y = splrep(ts, astar_problem.path[:, 1], s=spline_alpha)
        

        traj_plan = TrajectoryPlan(
            path=np.array(astar_problem.path), 
            path_x_spline=spline_x, 
            path_y_spline=spline_y, 
            duration=len(astar_problem.path)
        )

        return traj_plan

    def create_spline(self, values):
        t = np.linspace(0, 1, len(values))
        tck = splrep(t, values, s=0)
        
        return tck


if __name__ == '__main__':
    rclpy.init()
    node = Navigator()

    node.get_logger().info('NODE MAIN FUNCTION')
    rclpy.spin(node)
    rclpy.shutdown()