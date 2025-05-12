#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import cvxpy as cp
from rrtStar import RRTStar

class CBF_RRT_Star_Follower(Node):
    def __init__(self):
        super().__init__('cbf_rrt_star_follower')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.x = 0.0
        self.y = 0.0
        self.path = []  # Initially empty, will be set by get_goals()
        self.current_wp_index = 0
        self.obstacles = [
            (2.75, 2.5, 0.25),
            (3.25, 2.5, 0.25),
            (3.75, 2.5, 0.25),
            (4.25, 2.5, 0.25),
            (4.75, 2.5, 0.25),
            (2.0, 3.5, 0.25)
        ]
        self.safety_margin = 0.1
        self.max_speed = 0.2

    def get_goals(self):
        """Generate the path using RRT*."""
        rrt_star = RRTStar(start=(4.5, 1.5), goal=(4.5, 4.5), map_size=(5.0, 5.0))
        raw_path = rrt_star.plan()  # Generate raw path from RRT* planner

        if raw_path is None:
            self.get_logger().warn("RRT* could not find a path.")
            return []

        return raw_path

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def timer_callback(self):
        # Check if path is empty or if we've already reached the goal
        if not self.path:  # If path is empty, generate it
            self.path = self.get_goals()
            if not self.path:
                self.get_logger().warn("Path generation failed, stopping.")
                return  # Exit if path cannot be generated

        if self.current_wp_index >= len(self.path):  # If we're done with the path
            self.get_logger().info("Reached final goal.")
            return

        # Get the current goal
        goal_x_, goal_y_ = self.path[self.current_wp_index]
        goal_x = goal_y_
        goal_y = 5 - goal_x_
        to_goal = np.array([goal_x - self.x, goal_y - self.y])
        dist_to_goal = np.linalg.norm(to_goal)

        if dist_to_goal < 0.2:  # If close enough to the goal, move to the next one
            self.get_logger().info(f"Reached waypoint {self.current_wp_index + 1}. Moving to next waypoint.")
            self.current_wp_index += 1
            return

        # Desired velocity toward the goal
        v_des = self.max_speed * to_goal / dist_to_goal

        # Define optimization variable for velocity command
        v_cmd = cp.Variable(2)  # [vx, vy]
        constraints = []

        # CBF constraints for each obstacle
        for ox, oy, r in self.obstacles:
            dx = self.x - ox
            dy = self.y - oy
            dist = np.sqrt(dx**2 + dy**2)
            h = dist - (r + self.safety_margin)
            
            # Relax the CBF constraint to allow some margin
            if dist < (r + self.safety_margin): 
                continue  # Skip this constraint if too close

            grad_h = np.array([dx / dist, dy / dist])
            constraints.append(grad_h @ v_cmd + 0.8 * h >= 0)  # Relax the constraint a bit

        # Minimize deviation from desired velocity
        objective = cp.Minimize(cp.sum_squares(v_cmd - v_des))
        prob = cp.Problem(objective, constraints)

        try:
            prob.solve()
            v_safe = v_cmd.value
            if v_safe is None:
                v_safe = np.zeros(2)
                self.get_logger().warn("Optimization failed, stopping.")
        except:
            v_safe = np.zeros(2)
            self.get_logger().warn("CVXPY solver error, stopping.")

        # Send the computed velocity command
        twist = Twist()
        twist.linear.x = v_safe[0]
        twist.linear.y = v_safe[1]  # Can be ignored for differential drive
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

def main():
    rclpy.init()
    node = CBF_RRT_Star_Follower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
