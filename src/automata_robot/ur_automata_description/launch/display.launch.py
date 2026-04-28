import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
   # salvo paths
    pkg_share = get_package_share_directory('ur_automata_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'ur_automata.urdf.xacro')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'config.rviz')

   # argomenti x launch
    ur_type_arg = DeclareLaunchArgument(
        'ur_type',
        default_value='ur5e',
        description='UR type (ur3, ur5, ur10, ur3e, ur5e, ur10e, etc.)'
        )

    name_arg = DeclareLaunchArgument('name',
        default_value='ur',
        description='Robot name'
        )

    ur_type = LaunchConfiguration('ur_type')
    name = LaunchConfiguration('name')

    robot_desc = ParameterValue(Command(['xacro ', xacro_file, ' ur_type:=', ur_type, ' name:=', name]), value_type=str)

    return LaunchDescription([
        ur_type_arg,
        name_arg,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file]
        )
    ])