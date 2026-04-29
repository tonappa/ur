import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

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

    use_moveit_value = cfg['planning']['use_moveit']
    use_moveit_str_condition = 'true' if use_moveit_value else 'false'

    use_rviz_value = cfg['planning']['use_rviz']
    use_rviz_str_condition = 'true' if use_rviz_value else 'false'

    use_scene_value = cfg['planning']['use_scene']
    use_scene_str_condition = 'true' if use_scene_value else 'false'

    declared_arguments = [
        DeclareLaunchArgument(
            'use_moveit',
            default_value=use_moveit_str_condition,
            description='Launch MoveIt if true. Default is read from automata_config.yaml.'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value=use_rviz_str_condition,
            description='Launch RViz if true. Default is read from automata_config.yaml.'
        ),
        DeclareLaunchArgument(
            'use_scene',
            default_value=use_scene_str_condition,
            description='Spawn the planning-scene objects if true. Default is read from automata_config.yaml.'
        )
    ]

    pkg_share_dir = get_package_share_directory('ur_automata_bringup')

    ur_automata_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share_dir, 'launch', 'ur_automata_control.launch.py')
        )
    )

    ur_automata_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share_dir, 'launch', 'ur_automata_moveit.launch.py')
        ),
        launch_arguments={
            'use_rviz':  LaunchConfiguration('use_rviz'),
            'use_scene': LaunchConfiguration('use_scene'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_moveit'))
    )

    return LaunchDescription(declared_arguments + [
        ur_automata_control_launch,
        ur_automata_moveit_launch
    ])
