import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():

    package_name='alfred_sim'
    world_package='aws_robomaker_small_house_world'

    # Launch configurations
    world = LaunchConfiguration('world')
    rviz = LaunchConfiguration('rviz')

    # Path to default world 
    default_world = os.path.join(get_package_share_directory(world_package),'worlds', 'small_house.world')
    default_rviz = os.path.join(get_package_share_directory(package_name),'rviz', 'view_bot.rviz')
    robot_controllers = os.path.join(get_package_share_directory(package_name),'config','my_controllers.yaml')
    bridge_params = os.path.join(get_package_share_directory(package_name),'config','gz_bridge.yaml')
    ekf_params = os.path.join(get_package_share_directory(package_name),'config','ekf.yaml')

    # declare launch arguments 
    declare_world = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
        )
    urdf_path = os.path.join(get_package_share_directory(package_name),'description','robot.urdf.xacro')   
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'urdf': urdf_path}.items()
    )

    # Launch the gazebo server to initialize the simulation
    gazebo_server = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([os.path.join(
                        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
                    )]), launch_arguments={'gz_args': ['-r -s -v4 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    # Always launch the gazebo client to visualize the simulation
    gazebo_client = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([os.path.join(
                        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
                    )]), launch_arguments={'gz_args': '-g '}.items()
    )

    # Run the spawner node from the ros_gz_sim package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='ros_gz_sim', 
                        executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'alfred',
                                   '-z', '0.1'],
                        output='screen')
    
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

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        parameters=[
            ekf_params,
            {'use_sim_time': True},
        ]
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', default_rviz],
        output='screen',)

    # Launch them all!
    return LaunchDescription([
        declare_world,
        rsp,
        ros_gz_bridge,
        gazebo_server,
        gazebo_client,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        ekf_node,
        rviz2
    ])

## to open with house model on gazebo : 
## ros2 launch alfred_sim launch_sim.launch.py world:=/home/harshdev/alfred_ws/src/aws-robomaker-small-house-world/worlds/small_house.world