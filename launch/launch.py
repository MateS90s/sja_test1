import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    
    # Okresl nazwe pakietu i sciezke do pliku xacro
    pkg_name = 'sja'
    file_subpath = 'description/sja.urdf.xacro'


    # Use xacro to process the file
    xacro_file_path = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    # Przetwarzanie pliku XACRO
    robot_description_raw = xacro.process_file(xacro_file_path).toxml()

    # Ścieżka do pliku .world
    world_file_path = os.path.join(
        get_package_share_directory('sja'), 'worlds', 'example.world'
    )

    # Ścieżka do pliku konfiguracyjnego kontrolerów
    controller_config_file_path = os.path.join(
        get_package_share_directory('sja'), 'config', 'controller_config.yaml'
    )

    # Dodanie węzła do publikowania stanu robota
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw
        }]
    )

    # Konfiguracja Gazebo z własnym plikiem .world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
            )
        ]),
        launch_arguments={'world': world_file_path}.items()
    )

    # Dodanie węzła do publikowania modelu robota w Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'sja', '-x', '0', '-y', '0', '-z', '1.0'],
        output='screen'
    )

    # Dodanie węzła joint_state_publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # Dodanie węzła rosbridge_server
    rosbridge_server = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{
            'port': 9090
        }]
    )

    # Dodanie statycznej transformacji
    static_transform_publisher_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'base_link'],
        output='screen'
    )

    # Dodanie węzła controller_manager
    controller_manager = Node(
       package='controller_manager',
       executable='ros2_control_node',
       parameters=[controller_config_file_path],
       output='screen'
    )

    # Ładowanie i uruchamianie kontrolerów po uruchomieniu controller_manager
    load_joint_state_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_controller'],
        output='screen'
    )

    load_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        joint_state_publisher,
        rosbridge_server,
        static_transform_publisher_base,
        TimerAction(
            period=10.0,
            actions=[controller_manager]
        ),
        TimerAction(
            period=25.0,  # Zwiększony czas na uruchomienie controller_manager
            actions=[load_joint_state_controller]
        ),
        TimerAction(
            period=30.0,  # Zwiększony czas na uruchomienie arm_controller
            actions=[load_arm_controller]
        )
    ])
