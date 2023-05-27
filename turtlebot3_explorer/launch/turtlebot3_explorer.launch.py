from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot3_explorer',
            executable='turtlebot3_explorer',
            name='turtlebot3_explorer',
            output='screen',
            parameters=[]
        ),
    ])
