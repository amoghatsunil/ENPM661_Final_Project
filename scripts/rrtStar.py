import numpy as np
import matplotlib.pyplot as plt
import random
import math
from scipy.interpolate import CubicSpline
from visibility_rrt.LQR_CBF_planning import LQR_CBF_Planner
import datetime
class Node:
    def __init__(self, x, y, yaw=0.0):
        self.x    = x
        self.y    = y
        self.yaw  = yaw        # ← new
        self.parent = None
        self.cost   = 0.0
class RRTStar:
    def __init__(self, start, goal, map_size, step_size=0.9, goal_sample_rate=0.3, max_iter=1000, search_radius=0.5):
        self.start = Node(*start) 
        self.goal = Node(*goal)
        self.map_size = map_size
        self.obstacle_list =[
                        (2.75, 2.5, 0.25),
                        (3.25, 2.5, 0.25),
                        (3.75, 2.5, 0.25),
                        (4.25, 2.5, 0.25),
                        (4.75, 2.5, 0.25),
                        (2.0, 3.5, 0.25)
                        ]
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.search_radius = search_radius
        self.nodes = [self.start]
        self.planner  = LQR_CBF_Planner(visibility=True, collision_cbf=True)
        self.solve_QP = True
        self.LQR_gain = {}

    def plan(self):
        for i in range(self.max_iter):
            rnd_node = self.get_random_node()
            nearest_node = self.get_nearest_node(self.nodes, rnd_node)
            new_node = self.steer(nearest_node, rnd_node)
            
            if not self.check_collision(new_node):
                near_nodes = self.find_near_nodes(new_node)
                new_node = self.choose_parent(new_node, near_nodes)
                self.nodes.append(new_node)
                self.rewire(new_node, near_nodes)
            
            if self.distance(new_node, self.goal) <= self.step_size:
                self.goal.parent = new_node
                self.goal.cost = new_node.cost + self.distance(new_node, self.goal)
                self.nodes.append(self.goal)
                return self.extract_path()
        return None

    def get_random_node(self):
        if random.random() < self.goal_sample_rate:
            return self.goal
        else:
            return Node(random.uniform(0, self.map_size[0]), random.uniform(0, self.map_size[1]))

    def get_nearest_node(self, nodes, rnd_node):
        return min(nodes, key=lambda node: self.distance(node, rnd_node))

    def smooth_path(self, path, iterations=100):

        if not path:
            return path

        path = path.copy()
        for _ in range(iterations):
            if len(path) <= 2:
                break
            i = random.randint(0, len(path) - 2)
            j = random.randint(i + 1, len(path) - 1)
            if j - i <= 1:
                continue
            if self.is_collision_free(path[i], path[j]):
                path = path[:i+1] + path[j:]
        
        # After smoothing, check for collisions on the entire path
        smoothed_path = []
        for i in range(len(path) - 1):
            if self.is_collision_free(path[i], path[i+1]):
                smoothed_path.append(path[i])
        
        # Ensure the last point (goal) is added even if it doesn't cause a collision
        smoothed_path.append(path[-1])
        return smoothed_path
    
    def is_collision_free(self, p1, p2):
            for (ox, oy, r) in self.obstacle_list:
                for t in np.linspace(0, 1, num=20):
                    x = p1[0] + t * (p2[0] - p1[0])
                    y = p1[1] + t * (p2[1] - p1[1])
                    if math.hypot(ox - x, oy - y) <= r + 0.3:
                        return False
            return True

    def cubic_spline_smooth(self, path, num_points=1000):
        
        # Extract x and y coordinates of the path
        x_points, y_points = zip(*path)

        # Use cubic spline interpolation
        cs_x = CubicSpline(range(len(x_points)), x_points, bc_type='natural')
        cs_y = CubicSpline(range(len(y_points)), y_points, bc_type='natural')

        # Generate a smooth path with num_points
        smooth_x = cs_x(np.linspace(0, len(x_points)-1, num_points))
        smooth_y = cs_y(np.linspace(0, len(y_points)-1, num_points))

        # Return the smoothed path
        smooth_path = list(zip(smooth_x, smooth_y))
        smoothed_path = []
        for i in range(len(smooth_path) - 1):
            if self.is_collision_free(smooth_path[i], smooth_path[i+1]):
                smoothed_path.append(smooth_path[i])
        
        # Ensure the last point (goal) is added even if it doesn't cause a collision
        smoothed_path.append(smooth_path[-1])
        return smoothed_path
    
    def steer(self, from_node, to_node):
        dist = self.distance(from_node, to_node)
        if dist < self.step_size:
            return to_node
        theta = math.atan2(to_node.y - from_node.y, to_node.x - from_node.x)
        new_node = Node(from_node.x + self.step_size * math.cos(theta),
                        from_node.y + self.step_size * math.sin(theta))
        new_node.parent = from_node
        new_node.cost = from_node.cost + self.step_size
        return new_node

    # def steer(self, from_node, to_node):
    #     start = Node(from_node.x, from_node.y, from_node.yaw)
    #     goal  = Node(to_node.x,   to_node.y, yaw=None)
    #     traj, error, found = self.planner.lqr_cbf_planning(
    #                               start_node=start,
    #                               goal_node=goal,
    #                               LQR_gain=self.LQR_gain,
    #                               solve_QP=self.solve_QP,
    #                               show_animation=False
    #                           )
    #     if not found:
    #         return None
    #     rx, ry, ryaw = traj
    #     dist_acc = 0.0
    #     for i in range(1, len(rx)):
    #         dist_acc += math.hypot(rx[i]-rx[i-1], ry[i]-ry[i-1])
    #         if dist_acc >= self.step_size:
    #             new = Node(rx[i], ry[i], yaw=ryaw[i])
    #             new.parent = from_node
    #             new.cost   = from_node.cost + dist_acc
    #             return new
    #     # fallback if trajectory < step_size
        
    #     new = Node(rx[-1], ry[-1], yaw=ryaw[-1])
    #     new.parent = from_node
    #     new.cost   = from_node.cost + dist_acc
    #     return new
    
    def check_collision(self, node):
        for (ox, oy, radius) in self.obstacle_list:
            if math.hypot(ox - node.x, oy - node.y) <= radius + 0.1:
                return True
        return False
    def check_collision(self, node):
        if node.parent is None:
            return False
        return not self.is_collision_free((node.parent.x, node.parent.y), (node.x, node.y))

    def find_near_nodes(self, new_node):
        n = len(self.nodes)
        r = min(self.search_radius * math.sqrt((math.log(n) / n)), self.step_size * 10)
        return [node for node in self.nodes if self.distance(node, new_node) <= r]

    def choose_parent(self, new_node, near_nodes):
        if not near_nodes:
            return new_node
        costs = [node.cost + self.distance(node, new_node) for node in near_nodes]
        min_index = int(np.argmin(costs))
        new_node.cost = costs[min_index]
        new_node.parent = near_nodes[min_index]
        return new_node

    def rewire(self, new_node, near_nodes):
        for node in near_nodes:
            new_cost = new_node.cost + self.distance(new_node, node) 
            if new_cost < node.cost and self.is_collision_free((new_node.x, new_node.y), (node.x, node.y)):
                node.parent = new_node
                node.cost = new_cost

    def extract_path(self):
        path = []
        node = self.goal
        while node.parent is not None:
            path.append((node.x, node.y))
            node = node.parent
        path.append((self.start.x, self.start.y))
        return path[::-1]

    def distance(self, n1, n2):
        return math.hypot(n1.x - n2.x, n1.y - n2.y)
    
    def plot(self, rrt_star, path):
        fig, ax = plt.subplots()
        for node in rrt_star.nodes:
            if node.parent:
                ax.plot([node.x, node.parent.x], [node.y, node.parent.y], "-g", linewidth=0.5)
        for (ox, oy, r) in self.obstacle_list:
            circle = plt.Circle((ox, oy), r, color='r')
            ax.add_patch(circle)
        if path:
            px, py = zip(*path)
            ax.plot(px, py, '-b', linewidth=2)
        ax.plot(rrt_star.start.x, rrt_star.start.y, "bs")
        ax.plot(rrt_star.goal.x, rrt_star.goal.y, "gs")
        ax.set_xlim(0, rrt_star.map_size[0])
        ax.set_ylim(0, rrt_star.map_size[1])
        ax.set_aspect('equal')
        plt.grid(True)
        plt.show()
        plt.savefig(f"rrt_star_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.png")
        plt.close(fig)