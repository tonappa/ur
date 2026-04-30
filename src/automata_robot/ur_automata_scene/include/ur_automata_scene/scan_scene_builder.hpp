#ifndef UR_AUTOMATA_SCENE__SCAN_SCENE_BUILDER_HPP_
#define UR_AUTOMATA_SCENE__SCAN_SCENE_BUILDER_HPP_

#include <string>

#include <Eigen/Core>
#include <rclcpp/time.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>

namespace ur_automata_scene {

// Build the planning scene with: table, support disk, target sphere,
// and three legs that hold the disk in place. All objects are placed
// in `global_frame`. `center` is the position of the support disk
// (and the target sphere), expressed in meters.
moveit_msgs::msg::PlanningScene build_scan_scene(
    const std::string & global_frame,
    const Eigen::Vector3d & center,
    const rclcpp::Time & stamp);

}  // namespace ur_automata_scene

#endif  // UR_AUTOMATA_SCENE__SCAN_SCENE_BUILDER_HPP_
