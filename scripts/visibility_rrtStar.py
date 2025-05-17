import math
import time
import copy
import numpy as np
import os

from visibility_rrt.utils import env, plotting, utils
from visibility_rrt.utils.node import Node
from visibility_rrt.LQR_CBF_planning import LQR_CBF_Planner
from cbf_qp_tracking import UnicyclePathFollower

# Flag to control animation display
SHOW_ANIMATION = False

class VisibilityRRTStar:
    def __init__(self, x_start, x_goal, max_sampled_node_dist=10, max_rewiring_node_dist=10,
                 goal_sample_rate=0.1, rewiring_radius=20, iter_max=1000, solve_QP=False,
                 visibility=True, collision_cbf=True,
                 show_animation=False, path_saved=None):
        # Initialize parameters for RRT* algorithm
        self.x_start = Node(x_start)
        self.x_goal = Node(x_goal)
        self.max_sampled_node_dist = max_sampled_node_dist
        self.max_rewiring_node_dist = max_rewiring_node_dist
        self.goal_sample_rate = goal_sample_rate
        self.rewiring_radius = rewiring_radius # [m]
        self.iter_max = iter_max
        self.solve_QP = solve_QP
        self.show_animation = show_animation
        self.path_saved = path_saved

        # Tuning parameters
        self.sample_delta = 0.5 # Pad the environment boundary
        self.goal_len = 1 

        # Initialization
        self.vertex = [self.x_start] # Store all nodes in the RRT tree
        self.path = [] # Final result of RRT algorithm

        # General setup
        self.env = env.Env()
        self.x_range = self.env.x_range # x range of the environment
        self.y_range = self.env.y_range # y range of the environment
        self.plotting = plotting.Plotting(x_start, x_goal)
        utils_ = utils.Utils() # Utility functions, e.g., collision checking
        self.is_collision = utils_.is_collision
        lqr_cbf_planner = LQR_CBF_Planner(visibility=visibility, collision_cbf=collision_cbf)
        self.lqr_cbf_planning = lqr_cbf_planner.lqr_cbf_planning
        self.LQR_Gain = dict() # Dictionary to store LQR gains

    def planning(self):
        # Main planning function for RRT*
        print("===================================")
        print("============  RRT*   ==============")
        print("Start generating path.")
    
        start_time = time.time()

        compute_node_time = 0
        rewire_time = 0

        for k in range(self.iter_max):
            compute_node_start_time = time.time()

            # Generate a random node and find the nearest neighbor
            node_rand = self.generate_random_node(self.goal_sample_rate)
            node_nearest = self.nearest_neighbor(self.vertex, node_rand)
            node_new = self.LQR_steer(node_nearest, node_rand)

            compute_node_end_time = time.time()

            if self.show_animation:
                # Visualization during iterations
                if k % 100 == 0:
                    print('rrtStar sampling iterations: ', k)
                    print('rrtStar 1000 iterations sampling time: ', time.time() - start_time)
                    start_time = time.time()

                if k % 500 == 0:
                    print('rrtStar sampling iterations: ', k)
                    self.plotting.animation_online(self.vertex, "rrtStar", True)

            rewire_start_time = time.time()

            # Check if the new node is feasible and safe
            if node_new and not self.is_collision(node_nearest, node_new):
                # Find neighbors and add the new node to the tree
                neighbor_index = self.find_near_neighbor(node_new) 
                self.vertex.append(node_new)

                # Perform rewiring
                if neighbor_index:
                    self.LQR_choose_parent(node_new, neighbor_index)
                    self.rewire(node_new, neighbor_index)
            rewire_end_time = time.time()
        
            compute_node_time += compute_node_end_time - compute_node_start_time
            rewire_time += rewire_end_time - rewire_start_time

        # Search for the goal parent node
        index = self.search_goal_parent()

        if index is None:
            print('No path found!')
            return None, compute_node_time, rewire_time

        # Extract the path to the goal
        self.path = self.extract_path(node_end=self.vertex[index])
        self.path = np.array(self.path, dtype=np.float64)
        print("path: ", self.path)

        # Visualization of the final path
        if self.show_animation:
            self.plotting.animation(self.vertex, self.path, "rrt*, N = " + str(self.iter_max))

        print("====  Path planning finished  =====")
        print("===================================\n")

        print("Compute node time: ", compute_node_time)
        print("Rewire time: ", rewire_time)

        return self.path, compute_node_time, rewire_time

    def generate_random_node(self, goal_sample_rate=0.1):
        # Generate a random node within the environment
        delta = self.sample_delta

        if np.random.random() > goal_sample_rate: # Random sampling
            return Node((np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                        np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))

        return copy.deepcopy(self.x_goal) # Bias towards the goal
    
    @staticmethod
    def nearest_neighbor(node_list, n):
        # Find the nearest neighbor to a given node
        return node_list[int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y)
                                        for nd in node_list]))]

    def LQR_steer(self, node_start, node_goal, clip_max_dist=True):
        """
            - Steer from node_start to node_goal using LQR CBF planning
            - Only return a new node if the path has at least one safe path segment
            clip_max_dist: if True, clip the distance of node_goal to be at most self.max_sampled_node_dist
        """
        dist, theta = self.get_distance_and_angle(node_start, node_goal)
        if clip_max_dist: 
            dist = min(self.max_sampled_node_dist, dist)
        node_goal.x = node_start.x + dist * math.cos(theta)
        node_goal.y = node_start.y + dist * math.sin(theta)
        node_goal.yaw = theta

        # Perform LQR-CBF planning to generate a feasible trajectory
        rtraj, _, _, = self.lqr_cbf_planning(node_start, node_goal, self.LQR_Gain, solve_QP=self.solve_QP, show_animation=False)
        rx, ry, ryaw = rtraj
        if len(rx) == 1:
            return None
        px, py, traj_cost = self.sample_path(rx, ry)

        # Create a new node based on the trajectory
        node_new = Node((rx[-1], ry[-1], ryaw[-1]))
        node_new.parent = node_start

        # Calculate cost in terms of trajectory length
        node_new.cost = node_start.cost + sum(abs(c) for c in traj_cost)
        return node_new

    def find_near_neighbor(self, node_new):
        """
            - Find neighbors for rewiring
        """
        n = len(self.vertex) + 1
        r = min(self.max_rewiring_node_dist, self.rewiring_radius * math.sqrt((math.log(n) / n)))

        # Find nodes within the rewiring radius
        dist_table = [math.hypot(nd.x - node_new.x, nd.y - node_new.y) for nd in self.vertex]
        dist_table_index = [ind for ind in range(len(dist_table)) if dist_table[ind] <= r and
                            not self.is_collision(node_new, self.vertex[ind])]
        return dist_table_index
    
    def sample_path(self, rx, ry, step=0.2):
        # Smooth the path by interpolating between points
        px, py, traj_costs = [], [], []

        for i in range(len(rx) - 1):
            for t in np.arange(0.0, 1.0, step):
                px.append(t * rx[i+1] + (1.0 - t) * rx[i])
                py.append(t * ry[i+1] + (1.0 - t) * ry[i])

        # Calculate trajectory costs
        dx, dy = np.diff(px), np.diff(py)
        traj_costs = [math.sqrt(idx ** 2 + idy ** 2) for (idx, idy) in zip(dx, dy)]
        return px, py, traj_costs

    def cal_LQR_new_cost(self, node_start, node_goal):
        # Calculate the cost of a new trajectory using LQR-CBF planning
        rtraj, _, can_reach = self.lqr_cbf_planning(node_start, node_goal, self.LQR_Gain, show_animation=False, solve_QP=self.solve_QP)
        rx, ry, ryaw = rtraj
        px, py, traj_cost = self.sample_path(rx, ry)
        if rx is None:
            return float('inf'), False
        

        return node_start.cost + sum(abs(c) for c in traj_cost), can_reach

    def LQR_choose_parent(self, node_new, neighbor_index):
        """
            - Choose the best parent for node_new before rewiring
        """
        cost = []
        for i in neighbor_index:

            # Check if the neighbor node can reach node_new
            _, _, can_reach = self.lqr_cbf_planning(self.vertex[i], node_new, self.LQR_Gain, show_animation=False, solve_QP=self.solve_QP)

            if can_reach and not self.is_collision(self.vertex[i], node_new):  # Collision check
                update_cost, _ = self.cal_LQR_new_cost(self.vertex[i], node_new)
                cost.append(update_cost)
            else:
                cost.append(float('inf'))
        min_cost = min(cost)

        if min_cost == float('inf'):
            print('There is no good path.(min_cost is inf)')
            return None

        # Update the parent of node_new to the node with the minimum cost
        cost_min_index = neighbor_index[int(np.argmin(cost))]
        node_new.parent = self.vertex[cost_min_index] 
        node_new.parent.childrenNodeInds.add(len(self.vertex)-1) # Add the index of node_new to the children of its parent
        

    def rewire(self, node_new, neighbor_index):
        # Rewire the tree to improve path costs
        for i in neighbor_index:
            node_neighbor = self.vertex[i]

            # Check collision and LQR reachability
            if not self.is_collision(node_new, node_neighbor):
                new_cost, can_rach = self.cal_LQR_new_cost(node_new, node_neighbor)

                if can_rach and node_neighbor.cost > new_cost:
                    node_neighbor.parent = node_new
                    node_neighbor.cost = new_cost
                    self.updateCosts(node_neighbor)

    def updateCosts(self,node):
        # Update the costs of all child nodes recursively
        for ich in node.childrenNodeInds: 
            self.vertex[ich].cost = self.cal_LQR_new_cost(node,self.vertex[ich])[0] # Update cost
            self.updateCosts(self.vertex[ich])
            

    def search_goal_parent(self):
        # Search for the parent node closest to the goal
        dist_list = [math.hypot(n.x - self.x_goal.x, n.y - self.x_goal.y) for n in self.vertex]
        node_index = [i for i in range(len(dist_list)) if dist_list[i] <= self.goal_len]

        if not node_index:
            return None

        if len(node_index) > 0:
            cost_list = [dist_list[i] + self.vertex[i].cost for i in node_index
                         if not self.is_collision(self.vertex[i], self.x_goal)]
            return node_index[int(np.argmin(cost_list))]

        return len(self.vertex) - 1

    def extract_path(self, node_end):
        # Extract the path from the start to the goal
        path = [[self.x_goal.x, self.x_goal.y, self.x_goal.yaw]]
        node = node_end

        while node.parent is not None:
            path.append([node.x, node.y, node.yaw])
            node = node.parent
        path.append([node.x, node.y, node.yaw])

        path.reverse()

        return path

    @staticmethod
    def get_distance_and_angle(node_start, node_end):
        # Calculate the distance and angle between two nodes
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx, dy), math.atan2(dy, dx)

    @staticmethod
    def save_traj_npy(traj, path_saved):
        # Save the trajectory to a .npy file
        if path_saved  is None:
            cwd = os.getcwd()
            path_saved = ''
        print("Saving state trajectory...")
        np.save(path_saved , traj)

if __name__ == '__main__':
    SHOW_ANIMATION = False

    # Set up the environment type and start/goal positions
    env_type = env.type

    if env_type == 1:
        x_start = (2.0, 2.0, 0)  # Starting node (x, y, yaw)
        x_goal = (25.0, 3.0)  # Goal node
    elif env_type == 2:
        x_start = (1.5, 2.0, 0)  # Starting node (x, y, yaw)
        x_goal = (3.5, 3.5)  # Goal node

    # Initialize the RRT* planner
    lqr_rrt_star = VisibilityRRTStar(x_start=x_start, x_goal=x_goal,
                              max_sampled_node_dist=0.5,
                              max_rewiring_node_dist=1,
                              goal_sample_rate=0.1,
                              rewiring_radius=0.5,  
                              iter_max=1000,
                              solve_QP=True,
                              visibility=True,
                              collision_cbf=False,
                              show_animation=SHOW_ANIMATION)
    waypoints, _ , _ = lqr_rrt_star.planning()

    # Initialize the path follower
    x_init = waypoints[0]
    path_follower = UnicyclePathFollower(
        'DynamicUnicycle2D',
        x_init,
        waypoints,
        show_animation=True,
        plotting=lqr_rrt_star.plotting,
        env=lqr_rrt_star.env
    )

    # Run the path follower
    unexpected_beh = path_follower.run(save_animation=True)
