from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("dvl_plane_fit")
    ros_gz_sim_share = FindPackageShare("ros_gz_sim")

    world = LaunchConfiguration("world")
    sim_params = LaunchConfiguration("sim_params")
    bridge_config = LaunchConfiguration("bridge_config")
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_world = DeclareLaunchArgument(
        "world",
        default_value=PathJoinSubstitution([pkg_share, "worlds", "rov_wall.sdf"])
    )

    declare_sim_params = DeclareLaunchArgument(
        "sim_params",
        default_value=PathJoinSubstitution([pkg_share, "config", "sim.yaml"])
    )

    declare_bridge_config = DeclareLaunchArgument(
        "bridge_config",
        default_value=PathJoinSubstitution([pkg_share, "config", "bridge.yaml"])
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true"
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_gz_sim_share, "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={
            "gz_args": world
        }.items()
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        parameters=[
            {"config_file": bridge_config}
        ]
    )

    dvl_node = Node(
        package="dvl_plane_fit",
        executable="dvl_ros_node",
        name="dvl_ros_node",
        output="screen",
        parameters=[
            sim_params,
            {"use_sim_time": use_sim_time}
        ]
    )

    return LaunchDescription([
        declare_world,
        declare_sim_params,
        declare_bridge_config,
        declare_use_sim_time,
        gz_sim,
        bridge,
        dvl_node,
    ])