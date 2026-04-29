import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder

################################## config load ##################################
def _load_config():

    cfg_path = os.path.join(
        get_package_share_directory("ur_automata_bringup"),
        "config",
        "automata_config.yaml",
    )
    with open(cfg_path, "r") as f:
        return yaml.safe_load(f)


def launch_setup(context, *args, **kwargs):
    # ur_type deve essere risolto qui (stringa) perché MoveItConfigsBuilder lo
    # passa direttamente all'xacro come mapping; LaunchConfiguration non andrebbe.
    ur_type = LaunchConfiguration("ur_type").perform(context)
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    moveit_config = (
        MoveItConfigsBuilder("ur_automata", package_name="ur_automata_moveit_config")
        .robot_description(mappings={"ur_type": ur_type})
        .to_moveit_configs()
    )

    wait_robot_description = Node(
        package="ur_robot_driver",
        executable="wait_for_robot_description",
        output="screen",
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time,
             "publish_robot_description_semantic": True},
        ],
    )

    rviz_config = PathJoinSubstitution([
        FindPackageShare("ur_automata_bringup"),
        "rviz",
        "automata.rviz",
    ])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        condition=IfCondition(use_rviz),
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": use_sim_time},
        ],
    )

    return [
        wait_robot_description,
        RegisterEventHandler(
            OnProcessExit(
                target_action=wait_robot_description,
                on_exit=[move_group_node, rviz_node],
            )
        ),
    ]

def generate_launch_description():
    
    cfg = _load_config()

    use_scene_val = cfg['planning']['use_scene']
    use_scene_str_condition = "true" if use_scene_val else "false"

    use_rviz_val = cfg['planning']['use_rviz']
    use_rviz_str_condition = "true" if use_rviz_val else "false"

    declared_arguments = [
        DeclareLaunchArgument(
            "ur_type",
            default_value=cfg["robot"]["type"],
            description="UR robot model (ur5, ur5e, ...). Default letto da automata_config.yaml. "
                        "DEVE coincidere con quello passato a ur_automata_control.launch.py: altrimenti "
                        "il modello cinematico del move_group non corrisponde al TF pubblicato dal driver.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value=use_rviz_str_condition,
            description="Launch RViz with the MoveIt MotionPlanning panel. Default from automata_config.yaml.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Set true only when the bringup uses a simulated clock (Gazebo).",
        ),
        DeclareLaunchArgument(
            "use_scene",
            default_value=use_scene_str_condition,
            description="Launch scene.launch.py if true. The default is read from automata_config.yaml.",
        ),
    ]

    scene_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ur_automata_scene"), "launch", "scene.launch.py"
            ])
        ),
        condition=IfCondition(LaunchConfiguration("use_scene"))
    )

    return LaunchDescription(declared_arguments + [
        scene_launch,
        OpaqueFunction(function=launch_setup),
    ])
