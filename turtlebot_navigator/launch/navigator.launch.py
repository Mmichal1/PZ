from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    namespace_1_arg = DeclareLaunchArgument('namespace1', default_value='/robot1/',
                                            description='Namespace robot 1')

    namespace_2_arg = DeclareLaunchArgument('namespace2', default_value='/robot2/',
                                            description='Namespace robot 2')

    robot1_navigator = Node(
        package='turtlebot_navigator',
        executable='navigator',
        parameters=[
            {'namespace': LaunchConfiguration('namespace1')},
        ],
        emulate_tty=True
    )

    robot2_navigator = Node(
        package='turtlebot_controller',
        executable='navigator',
        parameters=[
            {'namespace': LaunchConfiguration('namespace2')},
        ],
        emulate_tty=True
    )

    return LaunchDescription([
        namespace_1_arg,
        namespace_2_arg,
        robot1_navigator,
        robot2_navigator
    ])
