#ifndef UR_AUTOMATA_SCENE__SCAN_SCENE_BUILDER_HPP_
#define UR_AUTOMATA_SCENE__SCAN_SCENE_BUILDER_HPP_

#include <string>

#include <Eigen/Core>
#include <rclcpp/time.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>

namespace ur_automata_scene {

// Opzioni della scena, lette da automata_config.yaml (sezione `scan`).
struct SceneOptions {
  // true  -> piattaforma ricostruita con disco + 3 gambe cilindriche (modo classico)
  // false -> mesh STL `meshes/<platform_mesh>` (geometria reale)
  bool platform_sim = true;
  // File STL in meshes/ (mm, stesso riferimento di disk.stl: disco vicino al
  // robot centrato in x 200, faccia superiore a y 504). Usato con platform_sim false.
  std::string platform_mesh = "disk.stl";

  // Keep-out di margine, in metri (0 = nessun oggetto):
  //   platform_margin -> cilindro attorno al disco (raggio +m, spessore +2m)
  //   table_margin    -> lastra alta m sul tavolo, sotto la sfera
  double platform_margin = 0.0;
  double table_margin = 0.0;

  // Muri della cella, in metri nel frame globale (0 = nessun muro):
  //   wall_back_y  -> parete dietro il robot, a y negativa
  //   wall_left_x  -> parete laterale a x negativa
  //   wall_right_x -> parete laterale a x positiva
  double wall_back_y = 0.0;
  double wall_left_x = 0.0;
  double wall_right_x = 0.0;

  // Posa della base del robot nel frame globale (metri, radianti), la stessa
  // di robot.base_xyz / base_rpy nel YAML. Se diversa da zero, dietro la base
  // viene messa una parete di montaggio perpendicolare all'asse Z della base:
  // il robot e' fissato a un muro, non al tavolo.
  Eigen::Vector3d base_xyz = Eigen::Vector3d::Zero();
  Eigen::Vector3d base_rpy = Eigen::Vector3d::Zero();
};

// Build the planning scene with: table, platform, target sphere and artefact,
// plus the optional keep-outs and walls in `opt`. All objects are placed in
// `global_frame`. `center` is the top face of the platform (and the target
// sphere), expressed in meters.
moveit_msgs::msg::PlanningScene build_scan_scene(
    const std::string & global_frame,
    const Eigen::Vector3d & center,
    const rclcpp::Time & stamp,
    const SceneOptions & opt);

}  // namespace ur_automata_scene

#endif  // UR_AUTOMATA_SCENE__SCAN_SCENE_BUILDER_HPP_
