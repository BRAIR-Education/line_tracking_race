from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition 
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution, 
    Command
)

from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


pkg_project_bringup = get_package_share_directory("line_tracking_race_bringup")
pkg_project_gazebo = get_package_share_directory("line_tracking_race_gazebo")
pkg_project_description = get_package_share_directory("line_tracking_race_description")
    
pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

def generate_ros_gz_bridge_description(context, *args, **kwargs):

    # Get launch configuration values
    use_ros2_control = LaunchConfiguration('use_ros2_control').perform(context)  # actual value

    ros_gz_bridge_conf_file = f"ros_gz_bridge_{"ros2" if use_ros2_control.lower() == "true" else "gazebo"}_control.yaml"
    
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': PathJoinSubstitution([pkg_project_bringup, "config", ros_gz_bridge_conf_file]),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )
    
    return [gz_bridge]

def generate_ros2_control_description(context, *args, **kwargs):

    use_ros2_control = LaunchConfiguration('use_ros2_control').perform(context) # actual value
    car_ros2_control_file = LaunchConfiguration("car_ros2_control_file")

    ld = LaunchDescription()

    if use_ros2_control.lower() == "true":
        joint_state_broadcaster_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
        )
        ld.add_action(joint_state_broadcaster_spawner)
        
        car_controllers_conf_file = PathJoinSubstitution([pkg_project_bringup,
                                                          "config",
                                                          car_ros2_control_file])
        
        diff_drive_base_controller_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'diff_drive_base_controller',
                '--param-file',
                car_controllers_conf_file,
                '--controller-ros-args',
                '-r /diff_drive_base_controller/cmd_vel:=/cmd_vel',
                '--controller-ros-args',
                '-r /diff_drive_base_controller/odom:=/odom',
                ],
        )
        ld.add_action(RegisterEventHandler(OnProcessExit(target_action=joint_state_broadcaster_spawner,
                                                        on_exit=[diff_drive_base_controller_spawner])))

    return [ld]
    
def generate_launch_description():

    # Declare launch arguments
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value='race_track.world',
        description='The world file'
    )

    car_file_arg = DeclareLaunchArgument(
        'car_file',
        default_value='car.urdf.xacro',
        description='The car model file (xacro/urdf/sdf)'
    )

    car_ros2_control_file_arg = DeclareLaunchArgument(
        'car_ros2_control_file',
        default_value='car_control_diff_drive.yaml',
        description='The car ros2_control configuration yaml file (yaml)'
    )

    use_ros2_control_arg = DeclareLaunchArgument(
        'use_ros2_control',
        default_value='false',
        description='Use ros2_control instead of gazebo control plugin'
    )

    x_pos_arg = DeclareLaunchArgument(
        "x_pos", default_value="0.0", description="X position to spawn the robot"
    )
    y_pos_arg = DeclareLaunchArgument(
        "y_pos", default_value="0.0", description="Y position to spawn the robot"
    )
    z_pos_arg = DeclareLaunchArgument(
        "z_pos", default_value="0.45", description="Z position to spawn the robot"
    )
    yaw_arg = DeclareLaunchArgument(
        "yaw", default_value="3.142", description="Yaw orientation to spawn the robot"
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true', description='Open RViz.')
    
    # --------- Launch Gazebo server and ros_gz_bridge with ros_gz_sim.launch ---------
    # https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_sim/launch/ros_gz_sim.launch.py
    # It allows to compose the bridge and gazebo see https://gazebosim.org/docs/latest/ros2_overview/
    # Doesn't work due to https://github.com/gazebosim/ros_gz/issues/774 ,
    # a workaround is to launch them separately
    #
    # gz_sim_ros_bridge = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'ros_gz_sim.launch.py'])
    #     ),
    #     launch_arguments={
    #         "log_level" : "info",
    #         "use_composition" : "true",
    #         "create_own_container" : "true",
    #         # --- gzbridge ---
    #         "bridge_name": "ros_gz_bridge",
    #         "config_file": PathJoinSubstitution(
    #             [pkg_project_bringup, "config", "ros_gz_bridge.yaml"]
    #         ),
    #         # --- gzserver ---
    #         "world_sdf_file": PathJoinSubstitution([pkg_project_gazebo, 'worlds', LaunchConfiguration("world_file")])
    #     }.items(),
    # )
    # # launch gazebo GUI since ros_gz_sim.launch launches only the gz server
    # gz_gui_cmd = ExecuteProcess(
    #     cmd=['gz', 'sim', '-g'],
    #     output='screen',
    #     emulate_tty=True,
    #     name='gazebo_gui'
    # )

    # launch gazebo
    world_file = PathJoinSubstitution([pkg_project_gazebo, 'worlds', LaunchConfiguration("world_file")])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={
            # -v -> verbose
            # -r -> start simulation (required by ros2_control) 
            'gz_args': [world_file, " -r -v 1"], 
            'on_exit_shutdown': 'True'
        }.items(),
    )

    # Launch bridge separately, avoiding ros_gz_bridge.launch,
    # using the executable ros_gz_bridge/parameter_bridge
    # Its configuration depends on the the type of control (gazebo, ros2_control) 
    gz_bridge = OpaqueFunction(function=generate_ros_gz_bridge_description)

    # --------- Load car xacro/urdf ---------
    urdf_path = PathJoinSubstitution([PathJoinSubstitution([pkg_project_description, "models"]),
                                     "urdf",
                                     LaunchConfiguration("car_file")])

    robot_description_content = Command(['xacro ', urdf_path,
                                         ' use_ros2_control:=', LaunchConfiguration("use_ros2_control")])

    robot_state_publisher = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      parameters=[{
                                          'robot_description': ParameterValue(robot_description_content, value_type=str),
                                      }])

    # --------- Spawn Robot node ---------
    spawn_robot_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "car",
            "-topic", "robot_description",
            "-x", LaunchConfiguration("x_pos"),
            "-y", LaunchConfiguration("y_pos"),
            "-z", LaunchConfiguration("z_pos"),
            "-Y", LaunchConfiguration("yaw"),
        ],
        output="screen",
    )

    # spawn ros2_control-related nodes AFTER spawn_robot_node
    ros2_control_description = OpaqueFunction(function=generate_ros2_control_description)
    ros2_control = RegisterEventHandler(OnProcessExit(target_action=spawn_robot_node,
                                                      on_exit=[ros2_control_description]))

    # Visualize in RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', PathJoinSubstitution([pkg_project_bringup, "config", 'line_tracking_race.rviz'])],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription(
        [
            world_file_arg,
            car_file_arg,
            x_pos_arg,
            y_pos_arg,
            z_pos_arg,
            yaw_arg,
            use_ros2_control_arg,
            car_ros2_control_file_arg,
            # rviz_arg,
            # gz_sim_ros_bridge,
            # gz_gui_cmd,
            gz_sim,
            gz_bridge,
            robot_state_publisher,
            spawn_robot_node,
            ros2_control
            # rviz
        ]
    )
