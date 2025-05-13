#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
import time
from rrtStar import RRTStar


class GoToPoint(Node):
    def __init__(self):
        super().__init__('go_to_point')

        # Publisher for sending velocity commands to the robot
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber for LiDAR data
        self.lidar_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        # Distance threshold for obstacle detection
        self.obstacle_distance_threshold = 1.5
        self.obstacle_detected = False

        # Read goal positions from the RRT* planner
        self.goal_list = self.get_goals()
        self.goal_list = self.filter_path_by_distance(self.goal_list, d_thresh=0.2)

        # Current robot position
        self.current_x = 1.5
        self.current_y = 0.5
        self.current_theta = 0.0

        # Begin navigating to the waypoints
        self.go_to_waypoints()

    def lidar_callback(self, msg):
        """
        Callback function for LiDAR data.
        Detects obstacles within the defined threshold distance.
        """
        # Check for obstacles in the forward direction (front 30 degrees)
        min_distance = float('inf')
        for i in range(-15, 16):  # Forward 30 degrees (adjust as needed)
            distance = msg.ranges[i] if not math.isinf(msg.ranges[i]) else float('inf')
            min_distance = min(min_distance, distance)

        # If an obstacle is detected within the threshold distance, stop and replan
        if min_distance < self.obstacle_distance_threshold:
            self.get_logger().info(f"Obstacle detected at {min_distance:.2f} meters. Replanning...")
            self.obstacle_detected = True
            self.stop_robot()
            self.replan_path()
        else:
            self.obstacle_detected = False

    def go_to_waypoints(self):
        """
        Iterates through each point in the filtered goal list, rotates to face the goal, and moves straight toward it.
        """
        for goal_x, goal_y in self.goal_list:
            if self.obstacle_detected:
                self.get_logger().info("Obstacle detected. Pausing navigation.")
                break

            # Calculate angle and distance to the goal
            delta_x = goal_x - self.current_x
            delta_y = goal_y - self.current_y

            target_angle = math.atan2(delta_y, delta_x)
            angle_to_rotate = target_angle - self.current_theta
            angle_to_rotate = math.atan2(math.sin(angle_to_rotate), math.cos(angle_to_rotate))

            # Rotate to face the goal
            self.rotate(angle_to_rotate)

            # Move straight to the goal
            distance = math.hypot(delta_x, delta_y)
            self.move_straight(distance)

            # Update current position
            self.current_x = goal_x
            self.current_y = goal_y
            self.current_theta = target_angle

        self.stop_robot()

    def rotate(self, angle, angular_speed=0.1):
        """
        Rotates the robot by the specified angle.
        """
        twist = Twist()
        twist.angular.z = angular_speed if angle > 0 else -angular_speed

        rotated = 0.0
        dt = 0.02

        while abs(rotated) < abs(angle):
            if self.obstacle_detected:
                self.stop_robot()
                return
            self.publisher_.publish(twist)
            rotated += angular_speed * dt
            time.sleep(dt)

        self.stop_robot()

    def move_straight(self, distance, linear_speed=0.1):
        """
        Moves the robot straight by the specified distance.
        """
        twist = Twist()
        twist.linear.x = linear_speed

        moved = 0.0
        dt = 0.04

        while moved < distance:
            if self.obstacle_detected:
                self.stop_robot()
                return
            self.publisher_.publish(twist)
            moved += linear_speed * dt
            time.sleep(dt)

        self.stop_robot()

    def stop_robot(self):
        """
        Stops the robot by publishing zero velocity.
        """
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

    def replan_path(self):
        """
        Replans the path using the current position as the new start point.
        """
        self.get_logger().info("Replanning path to avoid obstacle...")
        # Reinitialize the RRT* planner with the new start position
        rrt_star = RRTStar(start=(self.current_x, self.current_y), goal=(4.5, 3.5), map_size=(5.0, 5.0))

        # Assume obstacle size of 0.25
        rrt_star.add_dynamic_obstacle((self.current_x, self.current_y), 0.25)

        # Plan the new path
        new_path = rrt_star.plan()
        rrt_star.plot(rrt_star, new_path)

        # Update the goal list with the new path
        self.goal_list = self.filter_path_by_distance(new_path, d_thresh=0.2)
        self.get_logger().info("Path replanned. Resuming navigation.")

        # Resume navigation
        self.go_to_waypoints()

    def filter_path_by_distance(self, path, d_thresh):
        """
        Filters out points in the path that are closer than d_thresh to the previous retained point.
        """
        if not path:
            return []

        filtered = [path[0]]
        for point in path[1:-1]:
            prev = filtered[-1]
            dist = math.hypot(point[0] - prev[0], point[1] - prev[1])
            if dist >= d_thresh:
                filtered.append(point)

        filtered.append(path[-1])

        # Adjust coordinates to match simulation reference frame
        return [(y, 5 - x) for x, y in filtered]

    def get_goals(self):
        """
        Generate the initial path using RRT*.
        """
        rrt_star = RRTStar(start=(1.5, 0.5), goal=(4.5, 3.5), map_size=(5.0, 5.0))
        path = rrt_star.plan()
        rrt_star.plot(rrt_star, path)
        return path


def main(args=None):
    """
    Entry point of the program: initializes and spins the node.
    """
    rclpy.init(args=args)
    node = GoToPoint()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
