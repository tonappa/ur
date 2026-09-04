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


def _load_kinematics_yaml():
    """Carica kinematics.yaml di ur_automata_moveit_config come dizionario.
    Serve a istanziare il plugin IK (KDL) sul RobotState locale del nodo:
    senza questi parametri il setFromIK fallisce con 'No kinematics solver
    instantiated for group ur_manipulator'."""
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
    kinematics_yaml = _load_kinematics_yaml()

    scan_node = Node(
        package="ur_automata_scan",
        executable="scan_sequence_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            # I parametri del solver IK vanno sotto `robot_description_kinematics`.
            {"robot_description_kinematics": kinematics_yaml},
            {
            # Parametri di planning (da sezione `planning`)
            "global_frame":              p["global_frame"],
            "planning_group":            p["group"],
            "end_effector_link":         p["end_effector_link"],
            "home_pose_name":            p["home_pose_name"],
            "lower_home_pose_name":      p["lower_home_pose_name"],
            "trajectory_scaling_factor": p["trajectory_scaling_factor"],
            # Parametri di scan (da sezione `scan`)
            "scan_center":                 s["center"],
            "scan_radius":                 s["radius"],
            "scan_hemisphere":             s["hemisphere"],
            "scan_direction":              s["direction"],
            "scan_num_rings":              s["num_rings"],
            "scan_points_per_ring":        s["points_per_ring"],
            "scan_num_arc":                s["num_arc"],
            "scan_points_per_arc":         s["points_per_arc"],
            "scan_equator_exclusion_upper_deg": s["equator_exclusion_upper_deg"],
            "scan_equator_exclusion_lower_deg": s["equator_exclusion_lower_deg"],
            "scan_lock_pitch":                  s["lock_pitch"],
            "scan_stagger_rings":               s["stagger_rings"],
            "scan_adaptive_rings":              s["adaptive_rings"],
            "scan_occlusion_check":            s["occlusion_check"],
            "scan_occlusion_threshold_deg":    s["occlusion_threshold_deg"],
            "scan_fallback_search":            s["fallback_search"],
            "scan_fallback_radius_mm":         s["fallback_radius_mm"],
            "scan_fallback_planning_time":     s["fallback_planning_time"],
            "scan_fallback_max_plan_attempts": s["fallback_max_plan_attempts"],
            "scan_planner":                s["planner"],
            "scan_fallback_planner":       s["fallback_planner"],
            "scan_ompl_algorithm":         s["ompl_algorithm"],
            "scan_pitch_search_range_deg": s["pitch_search_range_deg"],
            "scan_pitch_search_step_deg":  s["pitch_search_step_deg"],
            "scan_pitch_xparallel_bias":   s["pitch_xparallel_bias"],
            "scan_ik_timeout":             s["ik_timeout"],
            "scan_planning_time":          s["planning_time"],
            "scan_planning_attempts":      s["planning_attempts"],
            },
        ],
    )

    return LaunchDescription([scan_node])
