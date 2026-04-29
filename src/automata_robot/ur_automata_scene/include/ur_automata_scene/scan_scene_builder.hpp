#ifndef UR_AUTOMATA_SCENE__SCAN_SCENE_BUILDER_HPP_
#define UR_AUTOMATA_SCENE__SCAN_SCENE_BUILDER_HPP_

#include <string>

#include <Eigen/Core>
#include <rclcpp/time.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>

namespace ur_automata_scene {

// Costruisce la PlanningScene con tavolo + disco di supporto + due gambe del
// supporto + sfera marker al centro dello scan. Tutti gli oggetti sono espressi
// nel frame `global_frame`. `center` è la posizione del piatto cilindrico
// (e della sfera marker) in metri.
moveit_msgs::msg::PlanningScene build_scan_scene(
    const std::string & global_frame,
    const Eigen::Vector3d & center,
    const rclcpp::Time & stamp);

}  // namespace ur_automata_scene

#endif  // UR_AUTOMATA_SCENE__SCAN_SCENE_BUILDER_HPP_
