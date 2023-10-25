
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os 

def generate_launch_description():
    bringup_dir = get_package_share_directory('path_optimizer_2')
    decomp_test_node = Node(
        package='path_optimizer_2',
        executable='path_optimizer_2_demo',
        name='decomp_test_node',
        output='screen',
    )
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(bringup_dir, 'config', 'rvizConfig.rviz')],
        output='screen')

    ld = LaunchDescription()
    ld.add_action(decomp_test_node)
    ld.add_action(start_rviz_cmd)
    return ld