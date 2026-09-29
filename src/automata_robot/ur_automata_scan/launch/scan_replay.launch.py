import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def _load_config():
    """Reads automata_config.yaml from the installed share of ur_automata_bringup."""
    cfg_path = os.path.join(
        get_package_share_directory("ur_automata_bringup"),
        "config",
        "automata_config.yaml",
    )
    with open(cfg_path, "r") as f:
        return yaml.safe_load(f)


def _load_kinematics_yaml():
    """kinematics.yaml of ur_automata_moveit_config: the replay does not use the
    IK, but without it the robot model loader warns about missing plugins."""
    path = os.path.join(
        get_package_share_directory("ur_automata_moveit_config"),
        "config",
        "kinematics.yaml",
    )
    with open(path, "r") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    cfg = _load_config()
    p = cfg["planning"]
    s = cfg["scan"]

    # Plays back the scan recorded with scan_sequence.launch.py record:=true.
    # Starts paused: ros2 service call /scan_replay_node/start std_srvs/srv/Trigger {}
    replay_node = Node(
        package="ur_automata_scan",
        executable="scan_replay_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"robot_description_kinematics": _load_kinematics_yaml()},
            {
            "recording_file":            str(s.get("recording_file", "")),
            "global_frame":              p["global_frame"],
            "planning_group":            p["group"],
            "end_effector_link":         p["end_effector_link"],
            "trajectory_scaling_factor": p["trajectory_scaling_factor"],
            "scan_center":               s["center"],
            "scan_radius":               s["radius"],
            "scan_planning_time":        s["planning_time"],
            },
        ],
    )

    return LaunchDescription([replay_node])
