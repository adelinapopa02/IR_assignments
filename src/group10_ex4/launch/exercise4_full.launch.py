from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. Locate the initialization launch file from the ir package
    ir_package_dir = get_package_share_directory('ir')
    ir_launch_file = os.path.join(ir_package_dir, 'launch', 'exercise_4.launch.py')

    # 2. Include the initialization launch file (Simulation/Environment setup)
    initialization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ir_launch_file)
    )

    # 3. Define the Turtlebot Server Node (The Lidar processor and Service responder)
    turtlebot_server_node = Node(
        package='group10_ex4',
        executable='turtlebot_server',
        name='turtlebot_server_node',
        output='screen',
        emulate_tty=True
    )

    # 4. Define the Burrow Client Node (The resource generator and Service caller)
    burrow_client_node = Node(
        package='group10_ex4',
        executable='burrow_client',
        name='burrow_client_node',
        output='screen',
        emulate_tty=True
    )

    # Return the combined launch description
    return LaunchDescription([
        initialization_launch,
        turtlebot_server_node,
        burrow_client_node
    ])