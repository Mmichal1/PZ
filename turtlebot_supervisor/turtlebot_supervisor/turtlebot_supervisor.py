import rclpy
import numpy as np
import time
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Int16 
from turtlebot_interfaces.action import ReserveSector

class TurtleBotSupervisor(Node):

    def __init__(self, supervisor_callback_group):
        super().__init__('supervisor')

        self._handle_parameters()
        self._create_map()
        self._create_publishers()
        self._create_timers()
        self._create_subscriber(supervisor_callback_group)
        self._create_action(supervisor_callback_group)

    def _handle_parameters(self):
        self._declare_default_parameters()
        self.grid_frame_id = self.get_parameter('grid_frame_id').value
        self.size_x = self.get_parameter('size_x').value
        self.size_y = self.get_parameter('size_y').value
        self.resolution = self.get_parameter('resolution').value

    def _declare_default_parameters(self):
        self.declare_parameter('grid_frame_id', 'map')
        self.declare_parameter('size_x', 5)
        self.declare_parameter('size_y', 5)
        self.declare_parameter('resolution', 1.0)

    def _create_map(self):
        self.grid = OccupancyGrid()
        self.grid.header.frame_id = self.grid_frame_id
        self.grid.info.resolution = self.resolution
        self.grid.info.width = self.size_x
        self.grid.info.height = self.size_y
        self.grid.info.origin.position.x = 0.0
        self.grid.info.origin.position.y = 0.0
        self.grid.data = np.full(self.size_x * self.size_y, 0, dtype=int).tolist()
        self.get_logger().info(f'Map size: x: {self.grid.info.width}, y: {self.grid.info.height}, resolution: {self.grid.info.resolution}.')

    def _create_publishers(self):
        self.publisher_occupancy_grid = self.create_publisher(OccupancyGrid, '/supervisor_map', 10)

    def _create_timers(self):
        self.timer_mine_publisher = self.create_timer(1.0, self.publish_grid)

    def _create_subscriber(self, supervisor_callback_group):
        self.subscriber_free_sector = self.create_subscription(
            Int16, '/supervisor/free_sector', self.listener_callback, 10, callback_group=supervisor_callback_group)
        
    def _create_action(self, supervisor_callback_group):
        self._action_server = ActionServer(self, ReserveSector, 'supervisor', 
                                            execute_callback=self.execute_callback,
                                            goal_callback=self.goal_callback,
                                            cancel_callback=self.cancel_callback,
                                            callback_group=supervisor_callback_group)
        
    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
    
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        result = ReserveSector.Result()
        
        self.get_logger().info(
            f'Querying for permission to enter sector: {goal_handle.request.sector}')
        
        while True:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                result.permission = False
                return result
            
            if self.grid.data[goal_handle.request.sector] == 0:
                self.grid.data[goal_handle.request.sector] = 100
                break

            self.get_logger().info(
                f'Sector: {goal_handle.request.sector} occupied, waiting...')
            time.sleep(1)

        self.get_logger().info(
            f'Permission to enter sector {goal_handle.request.sector} granted, value: {self.grid.data[goal_handle.request.sector]}')
        
        goal_handle.succeed()
        result.permission = True
        return result

    def publish_grid(self):
        self.grid.header.stamp = self.get_clock().now().to_msg()
        self.publisher_occupancy_grid.publish(self.grid)
        # self.get_logger().info(f'{self.grid.data}')
        self.get_logger().debug('Publishing grid')

    def listener_callback(self, sector_id):
        self.grid.data[sector_id.data] = 0
        self.get_logger().info(f'Sector freed: {sector_id.data}')

def main(args=None):
    rclpy.init(args=args)
    supervisor_callback_group = ReentrantCallbackGroup()
    supervisor = TurtleBotSupervisor(supervisor_callback_group)
    executor = MultiThreadedExecutor()
    executor.add_node(supervisor)
    try:
        print("")
        supervisor.get_logger().info('Supervisor running')
        executor.spin()
    except KeyboardInterrupt:
        supervisor.get_logger().info('KeyboardInterrupt, shutting down.\n')
    supervisor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
