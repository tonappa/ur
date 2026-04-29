from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("ur_automata", package_name="ur_automata_moveit_config")
        .robot_description(mappings={"use_mock_hardware": "true"})
        .to_moveit_configs()
    )
    return generate_demo_launch(moveit_config)
