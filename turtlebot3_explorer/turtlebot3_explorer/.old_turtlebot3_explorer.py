import rclpy
import math
import time
import numpy as np
from rclpy.node import Node
from rclpy.duration import Duration
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from .navigator import BasicNavigator
from action_msgs.msg import GoalStatus


class GoalPositionPlanner(Node):
    def __init__(self):
        super().__init__("goal_position_planner")
        self.costmap_subscription = self.create_subscription(
            OccupancyGrid,
            "/global_costmap/costmap",
            self.costmap_callback,
            10
        )
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            "/map",
            self.map_callback,
            10
        )
        self._init_nav2()
        self.goal_pub = self.create_publisher(
            PoseStamped, 'goal_pose', 10)  # 10 is queue_size
        self.unknown_threshold = -1  # Customize the threshold to determine unknown area
        self.obstacle_threshold = 90  # Threshold for considering a cell as an obstacle
        self.grid = None
        self.unknown_cells = []
        self.border_cells = []
        self.best_goal = PoseStamped()
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.initialized = True
        self.finished = False


    def map_callback(self, msg):
        # Process the map data and determine the best goal position
        self.map_data = msg.data
        slam_map_width = msg.info.width
        slam_map_height = msg.info.height

        self.grid = [[self.map_data[y * self.map_width + x] for x in range(slam_map_width)] for y in range(slam_map_height)]
        # self.get_logger().info('map dimensions= {:.2f} {:.2f}'.format(self.map_width*self.resolution,self.map_height*self.resolution))
        if self.initialized is True:
            self.explore()


    def costmap_callback(self, msg):
        # Process the map data and determine the best goal position
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.resolution = msg.info.resolution

        if self.initialized is False and self.grid is not None:
            self.x_origin = msg.info.origin.position.x
            self.y_origin = msg.info.origin.position.y 
            self.initialize()
            self.get_logger().info('Initialized')

       
    def initialize(self):
            self.choose_best_goal()
            self.navigator.goToPose(self.best_goal)
            self.initialized = True

    def explore(self):
        #chat said costmap data not map data !!!!!!!!!!!!!!!!!
        frontiers = self.find_frontiers(self.grid)
        sorted_frontiers = self.sort_frontiers(frontiers, self.x_origin, self.y_origin, self.resolution)
        self.best_goal.pose.position.x = (sorted_frontiers[1] + 0.5) * self.resolution + self.x_origin
        self.best_goal.pose.position.y = (sorted_frontiers[0] + 0.5) * self.resolution + self.y_origin
        # self.choose_best_goal()
        if self.navigator.isNavComplete() and sorted_frontiers:
            self.navigator.goToPose(self.best_goal)
        # self.get_logger().info('Goal has status code: {0}'.format(self.navigator.getResult()))
        # self.get_logger().info('Goal result: {:.2f}'.format(self.navigator.getResult()))
        if self.finished:
            self.get_logger().info('------------Exploring finished-------------')
            return
        

    def find_frontiers(self, costmap_data):
        # Implement your logic here to find frontiers in the costmap
        # This could involve using algorithms like edge detection, clustering, or graph-based methods
        # For simplicity, we'll assume the costmap is a binary map, and frontiers are cells adjacent to free space

        frontiers = []
        kernel = np.array([[1, 1, 1], [1, 0, 1], [1, 1, 1]])

        # Use a 3x3 kernel to find frontiers
        for i in range(1, costmap_data.shape[0] - 1):
            for j in range(1, costmap_data.shape[1] - 1):
                if costmap_data[i, j] == 0:
                    neighbor_sum = np.sum(kernel * costmap_data[i - 1 : i + 2, j - 1 : j + 2])
                    if neighbor_sum > 0 and neighbor_sum < 8:
                        frontiers.append((i, j))

        return frontiers
    
    def sort_frontiers(self, frontiers, x_origin, y_origin, resolution):
        # Sort frontiers based on their distances from the robot's current position
        sorted_frontiers = sorted(
            frontiers, key=lambda f: self.calculate_distance(f, x_origin, y_origin, resolution)
        )
        return sorted_frontiers

    def calculate_distance(self, frontier, x_origin, y_origin, resolution):
        # Calculate the Euclidean distance between the robot's current position and a frontier cell
        robot_x = x_origin / resolution
        robot_y = y_origin / resolution
        frontier_x = frontier[1]
        frontier_y = frontier[0]
        distance = np.sqrt((robot_x - frontier_x) ** 2 + (robot_y - frontier_y) ** 2)
        return distance

    def choose_best_goal(self):
        self.find_unknown_areas()
        # Evaluate frontier points and select the best goal position
        self.evaluate_frontier_points()
        self.unknown_cells.clear()
        self.border_cells.clear()

    def find_unknown_areas(self):
        for y in range(self.map_height):
            for x in range(self.map_width):
                if self.grid[y][x] <= self.unknown_threshold:
                    if self.is_border_cell(x, y):
                        self.border_cells.append((x, y))
                    else:
                        self.unknown_cells.append((x, y))


    def is_border_cell(self, x, y):
        # Check if the cell is a border cell by checking its neighbors
        neighbors = [
            (x - 1, y),
            (x + 1, y),
            (x, y - 1),
            (x, y + 1)
        ]
        known = False
        unknown = False
        for nx, ny in neighbors:
            if nx < 0 or nx >= self.map_width or ny < 0 or ny >= self.map_height:
                return False           
            # if self.grid[ny][nx] > self.obstacle_threshold:
            #     return False           
            # if self.grid[ny][nx] > self.unknown_threshold:
            #     known = True
            # if self.grid[ny][nx] <= self.unknown_threshold:
            #     unknown = True
            # if known and unknown is True:
            #     return True
            if self.grid[ny][nx] <= self.unknown_threshold:
                return True        

        return False

    def evaluate_frontier_points(self):
        best_information_gain = float('-inf')
        for frontier_point in self.border_cells:
            x, y = frontier_point
            goal = PoseStamped()
            goal.pose.position.x = x * self.resolution + self.x_origin
            goal.pose.position.y = y * self.resolution + self.y_origin
            # Compute the information gain for the current frontier point
            information_gain = self.compute_information_gain(x, y)
            self.get_logger().info('Calculating inf. gain for: {:.2f} {:.2f}'.format(goal.pose.position.x,goal.pose.position.y))
            self.get_logger().info('Inf. gain is {:.2f}'.format(information_gain))
            if information_gain > best_information_gain:
              self.best_goal = goal
              best_information_gain = information_gain
              self.get_logger().info('New self.best_goal= {:.2f} {:.2f} with information gain= {:.2f}'.format(self.best_goal.pose.position.x,self.best_goal.pose.position.y,best_information_gain))

    def compute_information_gain(self, x, y):
        information_gain = 0.0
        radius = 5  # Radius around the frontier point to calculate information gain
        # Calculate the coordinates of the cells within the radius
        start_x = max(x - radius, 0)
        end_x = min(x + radius, len(self.grid[0]) - 1)
        start_y = max(y - radius, 0)
        end_y = min(y + radius, len(self.grid) - 1)
        # Count the number of unknown and obstacle cells within the radius
        unknown_count = 0
        obstacle_count = 0
        for j in range(start_y, end_y + 1):
            for i in range(start_x, end_x + 1):
                if self.grid[j][i] <= self.unknown_threshold:
                    unknown_count += 1
                if self.grid[j][i] >= self.obstacle_threshold:
                    obstacle_count += 1

        # Calculate the probability of encountering an obstacle or unknown cell
        total_cells = (end_x - start_x + 1) * (end_y - start_y + 1)
        p_obstacle = obstacle_count / total_cells
        p_unknown = unknown_count / total_cells
        # Calculate the information gain using the entropy formula
        if p_obstacle > 0:
            information_gain -= 1.5 * p_obstacle * math.log2(p_obstacle)
        if p_unknown > 0:
            information_gain += p_unknown * math.log2(p_unknown)
        return information_gain



    def _init_nav2(self):
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
        self.start_time = self.navigator.get_clock().now()
        self.timeout_duration = Duration(seconds=120)


def main(args=None):
    rclpy.init(args=args)
    goal_position_planner = GoalPositionPlanner()
    rclpy.spin(goal_position_planner)
    goal_position_planner.destroy_node()
    rclpy.shutdown()

if   __name__ == '__main__':
    main()
