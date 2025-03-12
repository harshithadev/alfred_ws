import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():

    package_name='alfred_robot'

    # Launch configurations
    rviz = LaunchConfiguration('rviz')

    # Path to default world 
    default_rviz = os.path.join(get_package_share_directory(package_name),'rviz', 'view_bot.rviz')
    robot_controllers = os.path.join(get_package_share_directory(package_name),'config','my_controllers.yaml')

    urdf_path = os.path.join(get_package_share_directory(package_name),'description','robot.urdf.xacro')   
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'false', 'urdf': urdf_path}.items()
    )
    
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': urdf_path},
                    robot_controllers]
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            'diff_cont',
            '--param-file',
            robot_controllers,
            ],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', default_rviz],
        output='screen',)

    # Launch them all!
    return LaunchDescription([
        rsp,
        controller_manager, 
        rviz2,
        diff_drive_spawner,
        joint_broad_spawner,
        
    ])