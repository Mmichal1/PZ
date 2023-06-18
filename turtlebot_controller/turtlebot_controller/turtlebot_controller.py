import rclpy
import math
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist
from std_msgs.msg import Int8
from turtlebot_interfaces.action import ReserveSector

class TurtlebotController(Node):

    last_pose = []

    def __init__(self):
        super().__init__('turtlebot_controller')

        self.last_pose = [0, 0]
        self.next_pose = [0, 0]
        self.last_requested_sector_id = -1
        self.last_sector_id = -1
        self.current_sector_id = -1

        self._handle_parameters()
        self._create_subscribers()
        self._create_publishers()
        self._create_timers()

        self._action_client = ActionClient(self, ReserveSector, 'supervisor')

    def send_goal(self, sector_id):
        goal_msg = ReserveSector.Goal()
        goal_msg.sector = sector_id

        self.timer_stop_robot.reset()

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.permission:
            self.timer_stop_robot.cancel()
        
    def _handle_parameters(self):
        self._declare_default_parameters()
        self.namespace = self.get_parameter('namespace').value
        self.sector_lower_bound = self.get_parameter('sector_lower_bound').value
        self.sector_upper_bound = self.get_parameter('sector_upper_bound').value
        self.size_x = self.get_parameter('size_x').value
        self.size_y = self.get_parameter('size_y').value

    def _declare_default_parameters(self):
        self.declare_parameter('namespace', '/robot1')
        self.declare_parameter('sector_lower_bound', 0.15)
        self.declare_parameter('sector_upper_bound', 0.85)
        self.declare_parameter('size_x', 10)
        self.declare_parameter('size_y', 10)

    def _create_subscribers(self):
        self.subscription_pose = self.create_subscription(PoseWithCovarianceStamped, self.namespace + '/amcl_pose', self.listener_callback_pose, 10)
        self.subscription_next_sector = self.create_subscription(PoseStamped, self.namespace + '/current_goal', self.listener_callback_next_sector, 10)

    def _create_publishers(self):
        self.publisher_free_sector = self.create_publisher(Int8, '/supervisor/free_sector', 10)
        self.publisher_stop_robot = self.create_publisher(Twist, self.namespace + '/cmd_vel', 10)
    
    def _create_timers(self):
        self.timer_stop_robot = self.create_timer(0.01, self.timer_callback)
        self.timer_stop_robot.cancel()

    def timer_callback(self):
        twist = Twist()
        twist.linear.x = 0.0
        self.publisher_stop_robot.publish(twist)
        self.get_logger().info(f'{self.namespace} stopping...')

    def listener_callback_pose(self, robot_pose):
        self.last_pose[0] = robot_pose.pose.pose.position.x
        self.last_pose[1] = robot_pose.pose.pose.position.y

        if self.is_decimal_in_range(self.last_pose[0]) and self.is_decimal_in_range(self.last_pose[1]):
            
            self.current_sector_id = self.calculate_sector_id(math.floor(self.last_pose[0]), - math.floor(self.last_pose[1] + 1))

            if self.current_sector_id != self.last_sector_id:
                sector_id = Int8()
                sector_id.data = self.last_sector_id
                self.publisher_free_sector.publish(sector_id)
                self.last_sector_id = self.current_sector_id


    def listener_callback_next_sector(self, robot_pose):
        self.next_pose[0] = robot_pose.pose.position.x
        self.next_pose[1] = robot_pose.pose.position.y
        sector_id = self.calculate_sector_id(math.floor(self.next_pose[0]), - math.floor(self.next_pose[1] + 1))
        if sector_id != self.last_requested_sector_id:
            self.send_goal(sector_id)
            self.last_requested_sector_id = sector_id

    def is_decimal_in_range(self, value):
        value_decimal = abs(value % 1)

        if self.sector_lower_bound <= value_decimal <= self.sector_upper_bound:
            return True
        else:
            return False
    
    def calculate_sector_id(self, x, y):
        transformed_x = x + self.size_x // 2
        transformed_y = y + self.size_y // 2
        sector_id = (self.size_y - transformed_y - 1) * self.size_x + transformed_x
        return int(sector_id)


def main(args=None):
    rclpy.init(args=args)
    turtlebot_controller = TurtlebotController()
    rclpy.spin(turtlebot_controller)
    turtlebot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()