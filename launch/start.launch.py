from launch import LaunchDescription
from launch_ros.actions import Node
# Launch Args
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
# Load Params Yaml
import os
from ament_index_python.packages import get_package_share_directory

ARGUMENTS = [
    DeclareLaunchArgument(
        'data',
        default_value='true',
        description='The data arguement, by default has value true',
    ),
    DeclareLaunchArgument(
        'rate',
        default_value='1',
        description='The double arguement, by default has value 1.0',
    ),
]

def generate_launch_description():
        # Launch args
        data = LaunchConfiguration('data')
        rate = LaunchConfiguration('rate')

        talker_node = Node(
            package='ros2-dynamic-reconfigure',
            executable='dr_talker_node',
            name='dr_talker_node',
            parameters=[os.path.join(get_package_share_directory('ros2-dynamic-reconfigure'), 'config', 'params.yaml')],
        )

        dyn_recof_node = Node(
            package='rqt_reconfigure',
            executable='rqt_reconfigure',
            name='rqt_reconfigure_node',
        )

        # Define launch description
        ld = LaunchDescription(ARGUMENTS)
        ld.add_action(talker_node)
        ld.add_action(dyn_recof_node)

        # Return launch description
        return ld