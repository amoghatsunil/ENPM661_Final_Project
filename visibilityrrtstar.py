import numpy as np
import matplotlib.pyplot as plt
import random
import math
from scipy.interpolate import CubicSpline

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0.0

class RRTStar:
    def __init__(self, start, goal, map_size, step_size=0.2, goal_sample_rate=0.1, max_iter=2000, search_radius=0.5):
        self.start = Node(*start)
        self.goal = Node(*goal)
        self.map_size = map_size
        self.obstacle_list = [
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

    def plan(self):
        for i in range(self.max_iter):
            rnd_node = self.get_random_node()
            nearest_node = self.get_nearest_node(self.nodes, rnd_node)
            new_node = self.lqr_steer(nearest_node, rnd_node)
            if new_node is None:
                continue

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

    def lqr_steer(self, from_node, to_node, T=1.0, dt=0.1):
        x = np.array([from_node.x, from_node.y, 0.0])
        path = [Node(x[0], x[1])]
        path[0].cost = from_node.cost

        for _ in np.arange(0, T, dt):
            error = np.array([to_node.x - x[0], to_node.y - x[1]])
            angle_to_goal = math.atan2(error[1], error[0])
            angle_diff = angle_to_goal - x[2]
            angle_diff = (angle_diff + np.pi) % (2 * np.pi) - np.pi

            v = 0.5
            omega = 2.0 * angle_diff

            x[0] += v * math.cos(x[2]) * dt
            x[1] += v * math.sin(x[2]) * dt
            x[2] += omega * dt

            new_node = Node(x[0], x[1])
            new_node.cost = path[-1].cost + dt * v
            new_node.parent = path[-1]

            if self.check_collision(new_node) or not self.is_visible((path[-1].x, path[-1].y), (new_node.x, new_node.y)):
                return None
            path.append(new_node)

        final_node = path[-1]
        final_node.parent = from_node
        final_node.cost = from_node.cost + self.distance(from_node, final_node)
        return final_node

    def is_visible(self, p1, p2):
        # Placeholder: Replace with actual visibility checking logic
        return self.is_collision_free(p1, p2)

    def is_collision_free(self, p1, p2):
        for (ox, oy, r) in self.obstacle_list:
            for t in np.linspace(0, 1, num=20):
                x = p1[0] + t * (p2[0] - p1[0])
                y = p1[1] + t * (p2[1] - p1[1])
                if math.hypot(ox - x, oy - y) <= r + 0.1:
                    return False
        return True

    def check_collision(self, node):
        for (ox, oy, radius) in self.obstacle_list:
            if math.hypot(ox - node.x, oy - node.y) <= radius + 0.1:
                return True
        return False

    def find_near_nodes(self, new_node):
        n = len(self.nodes)
        r = min(self.search_radius * math.sqrt((math.log(n) / n)), self.step_size * 10)
        return [node for node in self.nodes if self.distance(node, new_node) <= r]

    def choose_parent(self, new_node, near_nodes):
        if not near_nodes:
            return new_node
        costs = []
        for node in near_nodes:
            if self.is_visible((node.x, node.y), (new_node.x, new_node.y)):
                costs.append(node.cost + self.distance(node, new_node))
            else:
                costs.append(float('inf'))
        min_index = int(np.argmin(costs))
        new_node.cost = costs[min_index]
        new_node.parent = near_nodes[min_index]
        return new_node

    def rewire(self, new_node, near_nodes):
        for node in near_nodes:
            new_cost = new_node.cost + self.distance(new_node, node)
            if new_cost < node.cost and self.is_visible((new_node.x, new_node.y), (node.x, node.y)):
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

# Example usage:
if __name__ == '__main__':
    start_x, start_y = map(float, input("Enter start coordinates (x y): ").split())
    goal_x, goal_y = map(float, input("Enter goal coordinates (x y): ").split())
    map_width, map_height = map(float, input("Enter map size (width height): ").split())
    planner = RRTStar(start=(start_x, start_y), goal=(goal_x, goal_y), map_size=(map_width, map_height))
    path = planner.plan()
    planner.plot(path)
