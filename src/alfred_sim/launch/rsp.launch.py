import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('alfred_sim'))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
    robot_description_config = Command(['xacro ', xacro_file,])
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config,}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )

    # Launch!
    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_publisher,
    ])

## Additionally : 
## To visualize : rviz2 -d src/alfred_sim/rviz/view_bot.rviz 
