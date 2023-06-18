
from enum import Enum
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import FollowWaypoints, NavigateToPose
from nav2_msgs.srv import ManageLifecycleNodes

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.duration import Duration


class NavigationResult(Enum):
    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3


class BasicNavigator(Node):

    def __init__(self):
        super().__init__(node_name='basic_navigator')
        self._handle_parameters()
        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = 'map'
        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None

        amcl_pose_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)

        self.initial_pose_received = True
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, self.namespace + 'navigate_to_pose')
        self.follow_waypoints_client = ActionClient(self, FollowWaypoints, self.namespace + 'FollowWaypoints')
        self.localization_pose_sub = self.create_subscription(PoseWithCovarianceStamped,
                                                              self.namespace + 'amcl_pose',
                                                              self._amclPoseCallback,
                                                              amcl_pose_qos)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped,
                                                      self.namespace + 'initialpose',
                                                      10)
        self.current_goal_pub = self.create_publisher(PoseStamped,
                                                      self.namespace + 'current_goal',
                                                      10)
            
    def _handle_parameters(self):
        self._declare_default_parameters()
        self.namespace = self.get_parameter('namespace').value

    def _declare_default_parameters(self):
        self.declare_parameter('namespace', '')

    def followWaypoints(self, poses):
        """Send a `FollowWaypoints` action request."""
        self.debug("Waiting for 'FollowWaypoints' action server")
        while not self.follow_waypoints_client.wait_for_server(timeout_sec=1.0):
            self.info(f"{self.namespace}/FollowWaypoints action server not available, waiting...")

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = poses

        self.info('Following ' + str(len(goal_msg.poses)) + ' goals.' + '...')
        send_goal_future = self.follow_waypoints_client.send_goal_async(goal_msg,
                                                                        self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Following ' + str(len(poses)) +
                       ' waypoints request was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def cancelNav(self):
        """Cancel pending navigation request of any type."""
        self.info('Canceling current goal.')
        if self.result_future:
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future)
        return

    def isNavComplete(self):
        """Check if the navigation request of any type is complete yet."""
        if not self.result_future:
            # task was cancelled or completed
            return True
        rclpy.spin_until_future_complete(
            self, self.result_future, timeout_sec=0.10)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                self.debug(
                    'Goal with failed with status code: {0}'.format(self.status))
                return True
        else:
            # Timed out, still processing, not complete yet
            return False

        self.debug('Goal succeeded!')
        return True

    def getFeedback(self):
        """Get the pending action feedback message."""
        return self.feedback

    def getResult(self):
        """Get the pending action result message."""
        if self.status == GoalStatus.STATUS_SUCCEEDED:
            return NavigationResult.SUCCEEDED
        elif self.status == GoalStatus.STATUS_ABORTED:
            return NavigationResult.FAILED
        elif self.status == GoalStatus.STATUS_CANCELED:
            return NavigationResult.CANCELED
        else:
            return NavigationResult.UNKNOWN

    def waitUntilNav2Active(self):
        """Block until the full navigation system is up and running."""
        self._waitForNodeToActivate('amcl')
        self._waitForInitialPose()
        self._waitForNodeToActivate('bt_navigator')
        self.info('Nav2 is ready for use!')
        return

    def lifecycleShutdown(self):
        """Shutdown nav2 lifecycle system."""
        self.info('Shutting down lifecycle nodes based on lifecycle_manager.')
        for srv_name, srv_type in self.get_service_names_and_types():
            if srv_type[0] == 'nav2_msgs/srv/ManageLifecycleNodes':
                self.info('Shutting down ' + srv_name)
                mgr_client = self.create_client(ManageLifecycleNodes, srv_name)
                while not mgr_client.wait_for_service(timeout_sec=1.0):
                    self.info(srv_name + ' service not available, waiting...')
                req = ManageLifecycleNodes.Request()
                req.command = ManageLifecycleNodes.Request().SHUTDOWN
                future = mgr_client.call_async(req)
                rclpy.spin_until_future_complete(self, future)
                future.result()
        return

    def _waitForNodeToActivate(self, node_name):
        # Waits for the node within the tester namespace to become active
        self.debug('Waiting for ' + node_name + ' to become active..')
        node_service = self.namespace + node_name + '/get_state'
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.info(node_service + ' service not available, waiting...')

        req = GetState.Request()
        state = 'unknown'
        while state != 'active':
            self.debug('Getting ' + node_name + ' state...')
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                self.debug('Result of get_state: %s' % state)
            time.sleep(2)
        return

    def _waitForInitialPose(self):
        while not self.initial_pose_received:
            self.info('Setting initial pose')
            self._setInitialPose()
            self.info('Waiting for amcl_pose to be received')
            rclpy.spin_once(self, timeout_sec=1.0)
        return

    def _amclPoseCallback(self, msg):
        self.debug('Received amcl pose')
        self.initial_pose_received = True
        return

    def _feedbackCallback(self, msg):
        self.debug('Received action feedback message')
        self.feedback = msg.feedback
        return

    def _setInitialPose(self):
        msg = PoseWithCovarianceStamped()
        msg.pose.pose = self.initial_pose.pose
        msg.header.frame_id = self.initial_pose.header.frame_id
        msg.header.stamp = self.initial_pose.header.stamp
        self.info('Publishing Initial Pose')
        self.initial_pose_pub.publish(msg)
        return

    def info(self, msg):
        self.get_logger().info(msg)
        return

    def warn(self, msg):
        self.get_logger().warn(msg)
        return

    def error(self, msg):
        self.get_logger().error(msg)
        return

    def debug(self, msg):
        self.get_logger().debug(msg)
        return

def start_navigation():

    navigator = BasicNavigator()

    navigator.waitUntilNav2Active()

    while True:
        
        goal_poses = []
        goal_pose1 = PoseStamped()
        goal_pose2 = PoseStamped()
        goal_pose3 = PoseStamped()
        goal_pose4 = PoseStamped()
        goal_pose5 = PoseStamped()
        goal_pose6 = PoseStamped()
        goal_pose7 = PoseStamped()
        goal_pose8 = PoseStamped()

        if navigator.namespace == '/robot1/':
            goal_pose1.pose.position.x = 0.5
            goal_pose1.pose.position.y = 0.5
            goal_pose1.pose.orientation.w = 0.7071068
            goal_pose1.pose.orientation.z = 0.7071068

            goal_pose2.pose.position.x = -0.5 #180
            goal_pose2.pose.position.y = 0.5
            goal_pose2.pose.orientation.w = 0.0
            goal_pose2.pose.orientation.z = 1.0

            goal_pose3.pose.position.x = -0.5 #90
            goal_pose3.pose.position.y = 1.5
            goal_pose3.pose.orientation.w = 0.7071068
            goal_pose3.pose.orientation.z = 0.7071068

            goal_pose4.pose.position.x = 0.5 #0
            goal_pose4.pose.position.y = 1.5
            goal_pose4.pose.orientation.w = 1.0
            goal_pose4.pose.orientation.z = 0.0

            goal_pose5.pose.position.x = 1.5 #0
            goal_pose5.pose.position.y = 1.5
            goal_pose5.pose.orientation.w = 1.0
            goal_pose5.pose.orientation.z = 0.0

            goal_pose6.pose.position.x = 1.5 #270
            goal_pose6.pose.position.y = 0.5
            goal_pose6.pose.orientation.w = -0.7071068
            goal_pose6.pose.orientation.z = 0.7071068

            goal_pose7.pose.position.x = 1.5
            goal_pose7.pose.position.y = -0.5
            goal_pose7.pose.orientation.w = -0.7071068
            goal_pose7.pose.orientation.z = 0.7071068

            goal_pose8.pose.position.x = 0.5
            goal_pose8.pose.position.y = -0.5
            goal_pose8.pose.orientation.w = 0.0
            goal_pose8.pose.orientation.z = 1.0

            goal_pose1.header.frame_id = 'map'
            goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
            goal_poses.append(goal_pose1)
            goal_pose2.header.frame_id = 'map'
            goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
            goal_poses.append(goal_pose2)
            goal_pose3.header.frame_id = 'map'
            goal_pose3.header.stamp = navigator.get_clock().now().to_msg()
            goal_poses.append(goal_pose3)
            goal_pose4.header.frame_id = 'map'
            goal_pose4.header.stamp = navigator.get_clock().now().to_msg()
            goal_poses.append(goal_pose4)
            goal_pose5.header.frame_id = 'map'
            goal_pose5.header.stamp = navigator.get_clock().now().to_msg()
            goal_poses.append(goal_pose5)
            goal_pose6.header.frame_id = 'map'
            goal_pose6.header.stamp = navigator.get_clock().now().to_msg()
            goal_poses.append(goal_pose6)
            goal_pose7.header.frame_id = 'map'
            goal_pose7.header.stamp = navigator.get_clock().now().to_msg()
            goal_poses.append(goal_pose7)
            goal_pose8.header.frame_id = 'map'
            goal_pose8.header.stamp = navigator.get_clock().now().to_msg()
            goal_poses.append(goal_pose8)

            

        elif navigator.namespace == '/robot2/':
            goal_pose1.pose.position.x = 0.5 #90
            goal_pose1.pose.position.y = -0.5
            goal_pose1.pose.orientation.w = 1.0
            goal_pose1.pose.orientation.z = 0.0

            goal_pose2.pose.position.x = 0.5
            goal_pose2.pose.position.y = 0.5
            goal_pose2.pose.orientation.w = 0.7071068
            goal_pose2.pose.orientation.z = 0.7071068

            goal_pose3.pose.position.x = -0.5
            goal_pose3.pose.position.y = 0.5
            goal_pose3.pose.orientation.w = 0.0
            goal_pose3.pose.orientation.z =1.0

            goal_pose4.pose.position.x = -0.5
            goal_pose4.pose.position.y = -0.5
            goal_pose4.pose.orientation.w = -0.7071068
            goal_pose4.pose.orientation.z =  0.7071068

            goal_pose1.header.frame_id = 'map'
            goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
            goal_poses.append(goal_pose1)

            goal_pose2.header.frame_id = 'map'
            goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
            goal_poses.append(goal_pose2)

            goal_pose3.header.frame_id = 'map'
            goal_pose3.header.stamp = navigator.get_clock().now().to_msg()
            goal_poses.append(goal_pose3)

            goal_pose4.header.frame_id = 'map'
            goal_pose4.header.stamp = navigator.get_clock().now().to_msg()
            goal_poses.append(goal_pose4)

        nav_start = navigator.get_clock().now()
        navigator.followWaypoints(goal_poses)

        i = 0
        while not navigator.isNavComplete():
            i = i + 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Executing current waypoint: ' +
                      str(feedback.current_waypoint + 1) + '/' + str(len(goal_poses)))
                now = navigator.get_clock().now()
                navigator.current_goal_pub.publish(goal_poses[feedback.current_waypoint])

                if now - nav_start > Duration(seconds=600.0):
                    navigator.cancelNav() 

        result = navigator.getResult()
        if result == NavigationResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == NavigationResult.CANCELED:
            print('Goal was canceled!')
            break
        elif result == NavigationResult.FAILED:
            print('Goal failed!')
            break
        else:
            print('Goal has an invalid return status!')
            break

    navigator.lifecycleShutdown()
    exit(0)


def main():
    rclpy.init()
    start_navigation()

if __name__ == '__main__':
    main()