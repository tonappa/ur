import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _load_config():
    """Legge automata_config.yaml dal share installato di ur_automata_bringup.

    Nota: dopo aver modificato il YAML in src/, ricordarsi di ribuildare
    ur_automata_bringup (i launch file leggono dal package share, non da src).
    """
    cfg_path = os.path.join(
        get_package_share_directory("ur_automata_bringup"),
        "config",
        "automata_config.yaml",
    )
    with open(cfg_path, "r") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    cfg = _load_config()

    declared_arguments = [
        DeclareLaunchArgument(
            "ur_type",
            default_value=cfg["robot"]["type"],
            description="UR robot model. Default letto da automata_config.yaml.",
        ),
        DeclareLaunchArgument(
            "robot_ip",
            default_value=cfg["robot"]["ip"],
            description="IP address of the UR controller (URSim or real). "
                        "Default letto da automata_config.yaml.",
        ),
        DeclareLaunchArgument(
            "headless_mode",
            default_value="false",
            description="If true the driver pushes the URScript directly without the External Control program.",
        ),
        DeclareLaunchArgument(
            "kinematics_params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("ur_description"),
                "config",
                LaunchConfiguration("ur_type"),
                "default_kinematics.yaml",
            ]),
            description="YAML produced by ur_calibration for this specific robot. "
            "Defaults to the nominal kinematics from ur_description (OK for URSim).",
        ),
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="",
            description="tf_prefix for joint and link names.",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="false",
            description="The MoveIt launch starts its own RViz.",
        ),
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="scaled_joint_trajectory_controller",
            description="Controller activated at startup; MoveIt expects scaled_joint_trajectory_controller.",
        ),
    ]

    description_file = PathJoinSubstitution([
        FindPackageShare("ur_automata_description"),
        "urdf",
        "ur_automata.urdf.xacro",
    ])

    controllers_file = PathJoinSubstitution([
        FindPackageShare("ur_automata_bringup"),
        "config",
        "ur_automata_controllers.yaml",
    ])

    upstream_control = PathJoinSubstitution([
        FindPackageShare("ur_robot_driver"),
        "launch",
        "ur_control.launch.py",
    ])

    return LaunchDescription(declared_arguments + [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(upstream_control),
            launch_arguments={
                "ur_type": LaunchConfiguration("ur_type"),
                "robot_ip": LaunchConfiguration("robot_ip"),
                "headless_mode": LaunchConfiguration("headless_mode"),
                "kinematics_params_file": LaunchConfiguration("kinematics_params_file"),
                "tf_prefix": LaunchConfiguration("tf_prefix"),
                "launch_rviz": LaunchConfiguration("launch_rviz"),
                "initial_joint_controller": LaunchConfiguration("initial_joint_controller"),
                "use_mock_hardware": "false",
                "description_file": description_file,
                "controllers_file": controllers_file,
            }.items(),
        ),
    ])
