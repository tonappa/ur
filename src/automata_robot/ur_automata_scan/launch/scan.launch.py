import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_config():
    """Legge automata_config.yaml dal share installato di ur_automata_bringup."""
    cfg_path = os.path.join(
        get_package_share_directory("ur_automata_bringup"),
        "config",
        "automata_config.yaml",
    )
    with open(cfg_path, "r") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    cfg = _load_config()
    p = cfg["planning"]
    s = cfg["scan"]

    scan_node = Node(
        package="ur_automata_scan",
        executable="scan_executor_node",
        name="scan_executor_node",
        output="screen",
        parameters=[{
            # Parametri di planning (da sezione `planning`)
            "global_frame":              p["global_frame"],
            "planning_group":            p["group"],
            "end_effector_link":         p["end_effector_link"],
            "home_pose_name":            p["home_pose_name"],
            "trajectory_scaling_factor": p["trajectory_scaling_factor"],
            # Parametri di scan (da sezione `scan`)
            "scan_center":                 s["center"],
            "scan_radius":                 s["radius"],
            "scan_hemisphere":             s["hemisphere"],
            "scan_direction":              s["direction"],
            "scan_num_rings":              s["num_rings"],
            "scan_points_per_ring":        s["points_per_ring"],
            "scan_equator_exclusion_deg":  s["equator_exclusion_deg"],
        }],
    )

    return LaunchDescription([scan_node])
