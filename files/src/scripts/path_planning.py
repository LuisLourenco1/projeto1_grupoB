#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
import heapq
import tf
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
import matplotlib.pyplot as plt

# Define a class to represent the map and provide methods for loading and extracting obstacle positions.
class Map:
    def __init__(self, map_file):
        self.map = self.load_map(map_file)

    def load_map(self, map_file):
        # Load the map image from the specified file in grayscale.
        map_image = cv2.imread(map_file, cv2.IMREAD_GRAYSCALE)
        if map_image is None:
            raise ValueError("Unable to load the map file.")
        # Set a threshold and convert the image to a binary array representing obstacles (0) and free space (1).
        threshold = 127
        map_array = (map_image > threshold).astype(np.int64)
        return map_array

    def get_obstacle_positions(self):
        # Get the coordinates of obstacles in the map.
        ox, oy = np.where(self.map == 0)
        return ox.tolist(), oy.tolist()

# Define a heuristic function for the A* algorithm.
def heuristic(a, b):
    return ((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2) ** 0.5

# Define a function to get neighboring points of a given point.
def get_neighbors(point):
    neighbors = []
    for i in range(-1, 2):
        for j in range(-1, 2):
            if i == 0 and j == 0:
                continue
            neighbors.append((point[0] + i, point[1] + j))
    return neighbors

# Implement the A* algorithm to find a path from start to goal, avoiding obstacles.
def astar(start, goal, obstacles):
    # Round the start, goal, and obstacle points to integers.
    start = (round(start[0]), round(start[1]))
    goal = (round(goal[0]), round(goal[1]))
    obstacles = [(round(point[0]), round(point[1])) for point in obstacles]

    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {point: float('inf') for point in obstacles + [start, goal]}
    g_score[start] = 0
    f_score = {point: float('inf') for point in obstacles + [start, goal]}
    f_score[start] = heuristic(start, goal)

    while open_set:
        current = heapq.heappop(open_set)[1]
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            return path[::-1]

        for neighbor in get_neighbors(current):
            if neighbor not in obstacles:
                tentative_g_score = g_score[current] + heuristic(current, neighbor)
                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    if neighbor not in [item[1] for item in open_set]:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return None

# Define a class to represent the robot using A* path planning.
class RoboAStar:
    def __init__(self, map_file, start, goal):
        self.map = Map(map_file)
        self.start = start
        self.goal = goal
        self.current_position = start
        self.current_orientation = 0
        self.path = None
        self.obstacle_detected = True
        self.a_star = None

        # Initialize ROS node and set up subscribers and publishers.
        rospy.init_node('robo_astar', anonymous=True)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.cmd_vel_pub = rospy.Publisher('/robot/cmd_vel', Twist, queue_size=10)
        self.rviz_path_pub = rospy.Publisher('/rviz_path', Path, queue_size=10)
        self.map_pub = rospy.Publisher('/map_topic', OccupancyGrid, queue_size=10)

    def publish_map(self, obstacle_map):
        # Publish the obstacle map as an OccupancyGrid message.
        map_msg = OccupancyGrid()
        map_msg.header.stamp = rospy.Time.now()
        map_msg.header.frame_id = "map"
        map_msg.info.map_load_time = rospy.Time.now()
        map_msg.info.resolution = self.a_star.resolution
        map_msg.info.width = self.a_star.x_width
        map_msg.info.height = self.a_star.y_width
        map_msg.info.origin.position.x = self.a_star.min_x
        map_msg.info.origin.position.y = self.a_star.min_y
        map_msg.data = [100 if obstacle else 0 for row in obstacle_map for obstacle in row]
        self.map_pub.publish(map_msg)

    def odom_callback(self, data):
        # Update the robot's current position and orientation based on odometry data.
        self.current_position = (data.pose.pose.position.x, data.pose.pose.position.y)
        quaternion = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w
        )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.current_orientation = euler[2]

    def scan_callback(self, data):
        # Check for obstacles in the robot's vicinity based on laser scan data.
        for distance in data.ranges:
            if distance < 1:
                self.obstacle_detected = True
                break

    def move_robot(self):
        # Move the robot based on the A* path.
        if not self.path:
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            return

        next_point = self.path[0]
        twist = Twist()
        twist.linear.x = min(1, self.calculate_linear_speed(next_point))
        twist.angular.z = self.calculate_angular_speed(next_point)
        self.cmd_vel_pub.publish(twist)

    def calculate_linear_speed(self, next_point):
        # Calculate linear speed based on the distance to the next point.
        dx = next_point[0] - self.current_position[0]
        dy = next_point[1] - self.current_position[1]
        distance = (dx ** 2 + dy ** 2) ** 0.5
        speed = min(1, distance)
        return speed

    def calculate_angular_speed(self, next_point):
        # Calculate angular speed based on the angle to the next point.
        angle_to_target = np.arctan2(next_point[1] - self.current_position[1], next_point[0] - self.current_position[0])
        angle_diff = angle_to_target - self.current_orientation
        angular_speed = min(1, abs(angle_diff)) * np.sign(angle_diff)
        return angular_speed

    def run(self):
        # Main loop for the robot's behavior.
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.obstacle_detected:
                # If an obstacle is detected, generate an A* path and visualize it.
                ox, oy = self.map.get_obstacle_positions()
                self.obstacle_detected = False

                path = astar(self.current_position, self.goal, list(zip(ox, oy)))
                
                x_values = [point[0] for point in path]
                y_values = [point[1] for point in path]

                # Plot the path
                plt.plot(x_values, y_values, marker='o', linestyle='-', label='Path')

                plt.scatter(ox, oy, color='red', marker='s', label='Obstacles')

                # Add labels and title to the plot
                plt.xlabel('X Coordinate')
                plt.ylabel('Y Coordinate')
                plt.title('Path Generated by Path Planning Algorithm')
                plt.show()

                if path:
                    self.path = path
                    path_msg = Path()
                    path_msg.header.frame_id = "map"
                    for x, y in path:
                        pose = PoseStamped()
                        pose.pose.position.x = x
                        pose.pose.position.y = y
                        path_msg.poses.append(pose)
                    self.rviz_path_pub.publish(path_msg)
                else:
                    print("Unable to find a path.")

            self.move_robot()

            if self.path and self.reached_point(self.path[0]):
                self.path.pop(0)

            rate.sleep()

    def reached_point(self, point, threshold=10):
        # Check if the robot has reached a given point within a specified threshold.
        dx = point[0] - self.current_position[0]
        dy = point[1] - self.current_position[1]
        distance = (dx ** 2 + dy ** 2) ** 0.5
        return distance < threshold


if __name__ == '__main__':
    try:
        # Set the map file path and initial and goal positions for the robot.
        robo = RoboAStar('/home/$USER/dcrobot_ws/src/projeto1_grupoB/mapa.pgm', (565, 257), (565, 390))
        robo.run()
    except rospy.ROSInterruptException:
        pass
