import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():

    #package name 
    package_name='alfred_sim'
    
    # Checking if we were given any launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    urdf = LaunchConfiguration('urdf')

    # Getting path to default files
    pkg_path = os.path.join(get_package_share_directory(package_name))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')

    # Declaring launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use sim time if true')

    declare_urdf = DeclareLaunchArgument(
            name='urdf', default_value=xacro_file,
            description='Path to the robot description file')   

    # Create a robot_state_publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,'robot_description': Command(['xacro ', urdf])}]
    )

    # Launch!
    return LaunchDescription([
        declare_use_sim_time,
        declare_urdf, 
        node_robot_state_publisher,
    ])

## Additionally : 
## To visualize : rviz2 -d src/alfred_sim/rviz/view_bot.rviz 
