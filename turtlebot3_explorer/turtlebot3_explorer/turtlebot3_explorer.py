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
from .nav2_toolkit import BasicNavigator
from action_msgs.msg import GoalStatus


class GoalPositionPlanner(Node):
    def __init__(self):
        super().__init__("goal_position_planner")

        self._init_nav2()
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)  # 10 is queue_size
        self.unknown_threshold = 0  # Customize the threshold to determine unknown area
        self.obstacle_threshold = 90  # Threshold for considering a cell as an obstacle

        self.unknown_cells = []
        self.border_cells = []
        self.best_goal = PoseStamped()
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.finished = False
        self.initialize()
        self.explore()

    def _init_nav2(self):
        self.nav2_toolkit = BasicNavigator()
        self.nav2_toolkit.waitUntilNav2Active()
        self.start_time = self.nav2_toolkit.get_clock().now()
        self.timeout_duration = Duration(seconds=120)

    def initialize(self):
        self.global_costmap = self.nav2_toolkit.getGlobalCostmap()
        self.map_data = self.global_costmap.data
        self.map_width = self.global_costmap.metadata.size_x
        self.map_height = self.global_costmap.metadata.size_y
        self.resolution = self.global_costmap.metadata.resolution
        self.origin = self.global_costmap.metadata.origin
        self.grid = np.array(self.map_data).reshape((self.map_width, self.map_height))

    def explore(self):
        while self.finished is False:
            # Algorithm for finding the best position
            exploration_radius = 5

            best_position_score = float("-inf")

            frontier_cells = []

            for x in range(self.map_width):
                for y in range(self.map_height):
                    if self.is_within_exploration_radius(x, y, exploration_radius):
                        if self.grid[x][y] == self.unknown_threshold:
                            continue  # Ignore unknown cells

                        is_frontier = False
                        for i in range(max(0, x - 1), min(x + 2, self.map_width)):
                            for j in range(max(0, y - 1), min(y + 2, self.map_height)):
                                if (
                                    self.grid[i][j] == self.unknown_threshold
                                    and self.grid[x][y] != self.unknown_threshold
                                ):
                                    is_frontier = True
                                    break

                            if is_frontier:
                                break

                        if is_frontier:
                            frontier_cells.append((x, y))

            if len(frontier_cells) == 0:
                self.get_logger().warn("No frontier cells found. Stopping exploration.")
                self.finished = True
                break

            for x, y in frontier_cells:
                position_score = self.calculate_position_score(x, y)
                if position_score > best_position_score:
                    best_position_score = position_score
                    self.best_goal.pose.position.x = float(x)*self.resolution + self.origin.position.x
                    self.best_goal.pose.position.y = float(y)*self.resolution + self.origin.position.y


            # Check if there are no frontier cells available
            if len(frontier_cells) == 0:
                self.get_logger().warn("No frontier cells found. Stopping exploration.")
                self.finished = True
                break

            self.get_logger().info('Goal result: {:.2f}  {:.2f}'.format(self.best_goal.pose.position.x,self.best_goal.pose.position.y))
            if self.nav2_toolkit.isTaskComplete():
                self.nav2_toolkit.goToPose(self.best_goal)
    def is_within_exploration_radius(self, x, y, radius):
        robot_x = self.origin.position.x + (x * self.resolution)
        robot_y = self.origin.position.y + (y * self.resolution)

        distance = math.sqrt((robot_x - self.origin.position.x)**2 + (robot_y - self.origin.position.y)**2)
        return distance <= radius

    def calculate_position_score(self, x, y):
        if self.grid[x][y] == self.unknown_threshold:
            return 0  # Ignore unknown cells

        if self.grid[x][y] >= self.obstacle_threshold:
            return 0  # Ignore obstacle cells

        # Check if the position is surrounded by unknown cells
        for i in range(max(0, x - 1), min(x + 2, self.map_width)):
            for j in range(max(0, y - 1), min(y + 2, self.map_height)):
                if self.grid[i][j] == self.unknown_threshold and self.grid[x][y] != self.unknown_threshold:
                    return 0  # Ignore if surrounded by unknown cells

        # Rest of the scoring calculation remains the same as before
        position_score = 0
        robot_x = self.origin.position.x + (x * self.resolution)
        robot_y = self.origin.position.y + (y * self.resolution)
        target_distance = math.sqrt((robot_x - self.best_goal.pose.position.x) ** 2 + (robot_y - self.best_goal.pose.position.y) ** 2)
        distance_weight = 1.0
        position_score += distance_weight / (target_distance + 1e-5)

        # Calculate average neighbor cost and adjust position score
        neighbor_cost_sum = 0
        neighbor_count = 0
        for i in range(max(0, x - 1), min(x + 2, self.map_width)):
            for j in range(max(0, y - 1), min(y + 2, self.map_height)):
                if i == x and j == y:
                    continue

                cost = self.grid[i][j]
                if cost != self.unknown_threshold and cost < self.obstacle_threshold:
                    neighbor_cost_sum += cost
                    neighbor_count += 1

        if neighbor_count > 0:
            average_neighbor_cost = neighbor_cost_sum / neighbor_count
            cost_weight = 0.5
            position_score += cost_weight * (1.0 - average_neighbor_cost / self.obstacle_threshold)

        return position_score





def main(args=None):
    rclpy.init(args=args)
    goal_position_planner = GoalPositionPlanner()
    rclpy.spin(goal_position_planner)
    goal_position_planner.destroy_node()
    rclpy.shutdown()

if   __name__ == '__main__':
    main()
