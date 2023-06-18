from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    size_x_arg = DeclareLaunchArgument('size_x', default_value='4',
                                       description='Map x size')

    size_y_arg = DeclareLaunchArgument('size_y', default_value='4',
                                       description='Map y size')

    namespace_1_arg = DeclareLaunchArgument('namespace1', default_value='/robot1',
                                           description='Namespace robot 1')
    
    namespace_2_arg = DeclareLaunchArgument('namespace2', default_value='/robot2',
                                           description='Namespace robot 2')

    robot1_controller = Node(
        package='turtlebot_supervisor',
        executable='controller',
        parameters=[
            {'size_y': LaunchConfiguration('size_y')},
            {'size_x': LaunchConfiguration('size_x')},
            {'namespace': LaunchConfiguration('namespace1')},
        ],
        emulate_tty=True
    )

    robot2_controller = Node(
        package='turtlebot_supervisor',
        executable='controller',
        parameters=[
            {'size_y': LaunchConfiguration('size_y')},
            {'size_x': LaunchConfiguration('size_x')},
            {'namespace': LaunchConfiguration('namespace2')},
        ],
        emulate_tty=True
    )

    supervisor = Node(
        package='turtlebot_supervisor',
        executable='supervisor',
        parameters=[],
        emulate_tty=True
    )

    multi_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('turtlebot3_navigation2'), '/launch/multi_tb3_navigation_launch.py']),
        launch_arguments={}.items()
    )

    return LaunchDescription([
        size_x_arg,
        size_y_arg,
        namespace_1_arg,
        namespace_2_arg,
        robot1_controller,
        robot2_controller,
        supervisor, 
        multi_navigation
    ])
