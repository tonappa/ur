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

    scene_node = Node(
        package="ur_automata_scene",
        executable="scene_publisher_node",
        name="scene_publisher_node",
        output="screen",
        parameters=[{
            "global_frame": global_frame,
            "scan_center": scan_center,
        }],
    )

    return LaunchDescription([scene_node])
