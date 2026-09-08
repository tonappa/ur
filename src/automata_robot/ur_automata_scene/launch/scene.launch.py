"""Lancia il nodo scene_publisher leggendo i parametri da
ur_automata_bringup/config/automata_config.yaml."""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def _load_config():
    cfg_path = os.path.join(
        get_package_share_directory("ur_automata_bringup"),
        "config",
        "automata_config.yaml",
    )
    with open(cfg_path, "r") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    cfg = _load_config()

    global_frame = cfg["planning"]["global_frame"]
    scan_center = [float(v) for v in cfg["scan"]["center"]]
    platform_sim = bool(cfg["scan"].get("platform_sim", True))
    platform_mesh = str(cfg["scan"].get("platform_mesh", "disk.stl"))
    platform_margin = float(cfg["scan"].get("platform_margin", 0.0))
    table_margin = float(cfg["scan"].get("table_margin", 0.0))
    walls = cfg["scan"].get("walls", {}) or {}
    robot = cfg.get("robot", {}) or {}
    base_xyz = [float(v) for v in robot.get("base_xyz", [0.0, 0.0, 0.0])]
    base_rpy = [float(v) for v in robot.get("base_rpy", [0.0, 0.0, 0.0])]

    scene_node = Node(
        package="ur_automata_scene",
        executable="scene_publisher_node",
        name="scene_publisher_node",
        output="screen",
        parameters=[{
            "global_frame": global_frame,
            "scan_center": scan_center,
            "platform_sim": platform_sim,
            "platform_mesh": platform_mesh,
            "platform_margin": platform_margin,
            "table_margin": table_margin,
            "wall_back_y": float(walls.get("back_y", 0.0)),
            "wall_left_x": float(walls.get("left_x", 0.0)),
            "wall_right_x": float(walls.get("right_x", 0.0)),
            "base_xyz": base_xyz,
            "base_rpy": base_rpy,
        }],
    )

    return LaunchDescription([scene_node])
