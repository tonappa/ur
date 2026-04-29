from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5e",
            description="UR robot model.",
        ),
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.1.97",
            description="IP address of the UR controller (URSim or real).",
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
