import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _load_yaml(package, *path):
    with open(os.path.join(get_package_share_directory(package), *path), "r") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    cfg = _load_yaml("ur_automata_bringup", "config", "automata_config.yaml")
    p = cfg["planning"]
    s = cfg["scan"]

    # Same IK setup as scan_sequence.launch.py: TRAC-IK in Speed mode for the
    # enumeration (move_group keeps its own kinematics.yaml).
    kinematics_yaml = _load_yaml("ur_automata_moveit_config", "config", "kinematics.yaml")
    kinematics_yaml[p["group"]]["solve_type"] = "Speed"
    # Acceleration limits of joint_limits.yaml, loaded into the robot model of
    # the node: they drive the PTP time estimate.
    joint_limits_yaml = _load_yaml("ur_automata_moveit_config", "config", "joint_limits.yaml")

    retry_param = {}
    retry_times = [float(t) for t in (s.get("retry_planning_times") or [])]
    if retry_times:
        retry_param["scan_retry_planning_times"] = retry_times

    # Same scan settings as scan_sequence_node (automata_config.yaml), so both
    # nodes plan the same waypoints with the same constraints.
    scan_node = Node(
        package="ur_automata_scan_tsp",
        executable="scan_tsp_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"robot_description_kinematics": kinematics_yaml},
            {"robot_description_planning": joint_limits_yaml},
            retry_param,
            {
            "global_frame":              p["global_frame"],
            "planning_group":            p["group"],
            "end_effector_link":         p["end_effector_link"],
            "home_pose_name":            p["home_pose_name"],
            "lower_home_pose_name":      p["lower_home_pose_name"],
            "trajectory_scaling_factor": p["trajectory_scaling_factor"],
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
            "scan_lock_pitch":             s["lock_pitch"],
            "scan_stagger_rings":          s["stagger_rings"],
            "scan_adaptive_rings":         s["adaptive_rings"],
            "scan_occlusion_check":        s["occlusion_check"],
            "scan_occlusion_disk_radius":  float(s.get("occlusion_disk_radius", 0.15)),
            "scan_occlusion_margin":       float(s.get("occlusion_margin", 0.01)),
            "scan_fallback_search":        s["fallback_search"],
            "scan_fallback_radius_mm":     s["fallback_radius_mm"],
            "scan_planners":               s["planners"],
            "scan_ompl_algorithm":         s["ompl_algorithm"],
            "scan_retry_ompl_algorithm":   str(s.get("retry_ompl_algorithm", "RRTConnect")),
            "scan_pitch_search_range_deg": s["pitch_search_range_deg"],
            "scan_pitch_search_step_deg":  s["pitch_search_step_deg"],
            "scan_pitch_xparallel_bias":   s["pitch_xparallel_bias"],
            "scan_enum_ik_timeout":        s["enum_ik_timeout"],
            "scan_planning_time":          s["planning_time"],
            "scan_planning_attempts":      s["planning_attempts"],
            "scan_sectors":                int(s.get("sectors", 0)),
            "scan_sector_offset_deg":      float(s.get("sector_offset_deg", 0.0)),
            "mode":              LaunchConfiguration("mode"),
            "order":             LaunchConfiguration("order"),
            "plan_file":         LaunchConfiguration("plan_file"),
            "tsp_time_limit":    ParameterValue(LaunchConfiguration("tsp_time_limit"), value_type=float),
            "refine_iterations": ParameterValue(LaunchConfiguration("refine_iterations"), value_type=int),
            "blocked_penalty_s": ParameterValue(LaunchConfiguration("blocked_penalty_s"), value_type=float),
            "tsp_check_neighbours": ParameterValue(LaunchConfiguration("tsp_check_neighbours"), value_type=int),
            "start_tolerance_rad": ParameterValue(LaunchConfiguration("start_tolerance_rad"), value_type=float),
            "speed": ParameterValue(LaunchConfiguration("speed"), value_type=float),
            },
        ],
    )

    return LaunchDescription([
        # plan: plan and save without moving | execute: load, check, execute
        DeclareLaunchArgument("mode", default_value="plan"),
        # alternate: sector order, then TSP and DP in turn | tsp: OR-Tools order, then DP
        # gtsp: order and IK together | sectors: order of scan_sequence_node
        DeclareLaunchArgument("order", default_value="alternate"),
        DeclareLaunchArgument("plan_file", default_value="/home/ros/ur/recordings/scan_tsp.yaml"),
        DeclareLaunchArgument("tsp_time_limit", default_value="10.0"),
        DeclareLaunchArgument("refine_iterations", default_value="3"),
        DeclareLaunchArgument("blocked_penalty_s", default_value="10.0"),
        # order:=tsp|gtsp: straight lines checked toward the N cheapest neighbours (0 = off)
        DeclareLaunchArgument("tsp_check_neighbours", default_value="10"),
        DeclareLaunchArgument("start_tolerance_rad", default_value="0.01"),
        # execute: speed relative to the plan, 1.0 = as planned, 0.5 = half speed (max 1.0)
        DeclareLaunchArgument("speed", default_value="1.0"),
        scan_node,
    ])
