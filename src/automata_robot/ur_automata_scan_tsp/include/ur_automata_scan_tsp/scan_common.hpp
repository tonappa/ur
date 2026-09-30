#pragma once

// Pieces of ur_automata_scan/src/scan_sequence_node.cpp that scan_tsp_node
// needs. They are static functions inside that node, so they cannot be linked:
// they are copied here with the same logic (comments translated, the planner
// statistics and the table output removed). Keep them in sync by hand if the
// candidate filters or the planner chain of scan_sequence_node change.

#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <rclcpp/rclcpp.hpp>

// ---------------------------------------------------------------------------
// IK candidates
// ---------------------------------------------------------------------------

// A valid configuration that reaches a waypoint (or its fallback point).
struct Candidate {
  std::vector<double> joints;          // group joints, JointModelGroup order
  double pitch_offset_rad = 0.0;       // rotation around Y_TCP applied to the base pose
  geometry_msgs::msg::Pose pose;       // pose actually used (with the pitch applied)
  bool is_fallback = false;            // true if the pose is a nearby point, not the waypoint
};

// Alternating pitch offsets in radians: [0, +step, -step, +2 step, -2 step, ...] up to +-range.
std::vector<double> build_pitch_offsets_rad(double range_deg, double step_deg);

// Look-at pose: position `pos`, Y axis toward `center`.
geometry_msgs::msg::Pose make_lookat(const Eigen::Vector3d & pos, const Eigen::Vector3d & center);

// Settings of the candidate enumeration (same meaning as the scan_* parameters).
struct EnumSettings {
  std::string group;
  std::string ee_link;
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  double radius = 0.3;
  std::vector<double> pitch_offsets;   // {0} when the pitch is locked
  double ik_timeout = 0.003;
  bool occlusion_check = false;
  double occlusion_disk_radius = 0.15;
  double occlusion_margin = 0.01;
  bool fallback_search = false;
  double fallback_radius_mm = 20.0;
};

struct EnumResult {
  std::vector<std::vector<Candidate>> layers;   // one per waypoint, empty = unreachable
  std::vector<std::string> view_blocked_by;     // non-empty = scene object between camera and center
  size_t total_candidates = 0;
  int collision_rejects = 0;
  int occlusion_rejects = 0;
  double seconds = 0.0;
};

// FASE 1 of scan_sequence_node: every valid IK configuration of every waypoint
// (pitch x IK branches, seeded from the previous waypoint in list order), with
// the scene collision, platform visibility and line-of-sight filters, the slow
// IK retry and the nearby fallback points.
EnumResult enumerate_candidates(const EnumSettings & s,
                                const std::vector<geometry_msgs::msg::Pose> & waypoints,
                                const moveit::core::RobotState & start_state,
                                const planning_scene::PlanningScenePtr & scene);

// True if the straight joint-space line from a to b collides with nothing
// (one sample every ~3 degrees on the joint that moves the most), i.e. the
// path Pilz PTP would follow. Always true without a scene.
bool segment_is_free(const planning_scene::PlanningScenePtr & scene,
                     const moveit::core::RobotState & reference,
                     const moveit::core::JointModelGroup * jmg,
                     const std::vector<double> & a, const std::vector<double> & b);

// ---------------------------------------------------------------------------
// Planner chain
// ---------------------------------------------------------------------------
struct PlannerChoice {
  std::string pipeline;   // e.g. "pilz_industrial_motion_planner" or "ompl"
  std::string planner;    // e.g. "PTP" or "RRTConnectkConfigDefault"
  bool is_circ = false;   // Pilz CIRC: needs the "center" constraint and a start on the sphere
  std::string label() const { return pipeline + "/" + planner; }
};

struct PlannerSetup {
  std::vector<PlannerChoice> chain;           // tried in order, first success wins
  moveit_msgs::msg::Constraints circ_center;  // path constraint required by CIRC
};

// YAML name ("ompl", "pilz_ptp", "pilz_lin", "pilz_circ", "stomp") -> planner.
bool resolve_planner(const std::string & name, const std::string & ompl_algorithm, PlannerChoice & out);

// Path constraint that Pilz CIRC reads as the arc center.
moveit_msgs::msg::Constraints make_circ_center_constraint(
  const std::string & frame, const std::string & link, const Eigen::Vector3d & center);

// plan() along the planner chain; the target (and the start state) must be set
// by the caller. allow_circ = false skips CIRC (start not on the scan sphere).
// used_label (optional) receives the planner that succeeded.
bool plan_with_fallback(moveit::planning_interface::MoveGroupInterface & move_group,
                        moveit::planning_interface::MoveGroupInterface::Plan & plan,
                        const PlannerSetup & planners, bool allow_circ,
                        const rclcpp::Logger & logger, std::string * used_label = nullptr);
