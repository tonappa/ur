// scan_tsp_node: experimental scan planner, to compare with scan_sequence_node.
//
//   mode:=plan     Plans the whole scan WITHOUT moving the robot and saves it:
//     1. IK candidates of every waypoint (same filters as scan_sequence_node);
//     2. visit order: order:=tsp   OR-Tools TSP on the waypoints (RoboTSP-like:
//                                  order first, IK configurations after),
//                     order:=gtsp  OR-Tools generalized TSP on all the IK
//                                  candidates (order and configuration together),
//                     order:=sectors  the order of scan_sequence_node,
//                     order:=alternate  from the sectors solution, TSP on the
//                                  chosen configurations and DP in turn;
//     3. IK configurations for that order: layered DP (ur_automata_scan) with
//        lazy checks of the straight joint-space segments;
//     4. every motion planned with MoveIt (same planner chain, retries and
//        recovery poses as scan_sequence_node), durations measured;
//     5. refinement: the measured durations replace the estimates and 2-4 are
//        repeated; the best measured plan is saved to plan_file.
//   mode:=execute  Loads plan_file, checks it against the current cell, then
//     executes it (starts paused, like scan_replay_node). If the robot is not
//     in the start state of the plan, it first goes there with a planned motion:
//       ros2 service call /scan_tsp_node/start std_srvs/srv/Trigger {}
//       ros2 service call /scan_tsp_node/pause std_srvs/srv/Trigger {}
//
// Costs are durations (s). The estimate of a segment is the Pilz PTP time on
// the straight joint line (see ptp_time); the orders found are heuristic
// solutions, not proven optimal.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include <geometric_shapes/bodies.h>
#include <geometric_shapes/body_operations.h>
#include <geometric_shapes/shapes.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "ur_automata_scan/frustum_markers.hpp"
#include "ur_automata_scan/planning_scene_client.hpp"
#include "ur_automata_scan/scan_sequence_planner.hpp"
#include "ur_automata_scan/sphere_waypoint_generator.hpp"
#include "ur_automata_scan_tsp/plan_file.hpp"
#include "ur_automata_scan_tsp/route_solver.hpp"
#include "ur_automata_scan_tsp/scan_common.hpp"
#include "ur_automata_scan_tsp/tsp_planning.hpp"

using MoveGroup = moveit::planning_interface::MoveGroupInterface;

static std::atomic<bool> g_paused{true};

static double seconds_since(std::chrono::steady_clock::time_point t0)
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
}

// ============================================================================
// Scene fingerprint: one axis-aligned box per world object
// ============================================================================
static std::vector<SceneObjectBox> scene_boxes(const planning_scene::PlanningScene & scene)
{
  std::vector<SceneObjectBox> result;
  collision_detection::WorldConstPtr world = scene.getWorld();
  for (const std::string & id : world->getObjectIds()) {
    collision_detection::World::ObjectConstPtr obj = world->getObject(id);
    if (!obj) continue;
    Eigen::AlignedBox3d box;
    for (size_t k = 0; k < obj->shapes_.size(); ++k) {
      const shapes::Shape * shape = obj->shapes_[k].get();
      const Eigen::Isometry3d & pose = obj->global_shape_poses_[k];
      if (shape->type == shapes::MESH) {
        const auto * mesh = static_cast<const shapes::Mesh *>(shape);
        for (unsigned int v = 0; v < mesh->vertex_count; ++v) {
          const double * p = mesh->vertices + 3 * v;
          box.extend(pose * Eigen::Vector3d(p[0], p[1], p[2]));
        }
      } else {
        std::unique_ptr<bodies::Body> body(bodies::createBodyFromShape(shape));
        if (!body) continue;
        body->setPose(pose);
        bodies::AABB aabb;
        body->computeBoundingBox(aabb);
        box.extend(aabb);
      }
    }
    if (box.isEmpty()) continue;
    SceneObjectBox entry;
    entry.id = id;
    entry.box = {box.min().x(), box.min().y(), box.min().z(), box.max().x(), box.max().y(), box.max().z()};
    result.push_back(entry);
  }
  return result;
}

// ============================================================================
// Planned motions
// ============================================================================
// One planned motion (a leg); a segment between two nodes of the tour is one
// leg, or two when it passes through a recovery pose.
struct Leg {
  std::string label;
  int waypoint = -1;                 // waypoint reached at the end, -1 = none
  moveit_msgs::msg::RobotTrajectory trajectory;
  std::string planner;
  double duration = 0.0;             // s
  double joint_motion = 0.0;         // sum over joints of the path length (rad)
};

struct Segment {
  bool ok = false;
  bool via_recovery = false;         // direct motion not plannable: through a recovery pose
  std::vector<Leg> legs;
  double duration() const
  {
    double t = 0.0;
    for (const Leg & leg : legs) t += leg.duration;
    return t;
  }
};

static double trajectory_joint_motion(const moveit_msgs::msg::RobotTrajectory & traj)
{
  double total = 0.0;
  const auto & points = traj.joint_trajectory.points;
  for (size_t k = 1; k < points.size(); ++k) {
    for (size_t j = 0; j < points[k].positions.size() && j < points[k - 1].positions.size(); ++j) {
      total += std::abs(points[k].positions[j] - points[k - 1].positions[j]);
    }
  }
  return total;
}

// Everything the planning loop needs to call MoveIt.
struct PlanContext {
  MoveGroup * move_group = nullptr;
  const moveit::core::JointModelGroup * jmg = nullptr;
  moveit::core::RobotState * reference = nullptr;
  PlannerSetup planners;
  PlannerSetup retry_planners;
  std::vector<double> retry_times;
  double planning_time = 5.0;
  rclcpp::Logger logger = rclcpp::get_logger("scan_tsp_node");
};

// Plans one motion from `from` to `to` (joint configurations, group order),
// with the planner chain and then the retries, like scan_sequence_node.
static bool plan_leg(PlanContext & ctx, const std::vector<double> & from, const std::vector<double> & to,
                     bool allow_circ, Leg & leg)
{
  MoveGroup & mg = *ctx.move_group;
  moveit::core::RobotState start(*ctx.reference);
  start.setJointGroupPositions(ctx.jmg, from);
  start.update();
  moveit::core::RobotState goal(*ctx.reference);
  goal.setJointGroupPositions(ctx.jmg, to);
  goal.update();

  mg.setStartState(start);
  mg.clearPathConstraints();
  mg.clearPoseTargets();
  mg.setJointValueTarget(goal);
  mg.setPlanningTime(ctx.planning_time);

  MoveGroup::Plan plan;
  bool ok = plan_with_fallback(mg, plan, ctx.planners, allow_circ, ctx.logger, &leg.planner);
  for (double retry_time : ctx.retry_times) {
    if (ok) break;
    mg.setPlanningTime(retry_time);
    ok = plan_with_fallback(mg, plan, ctx.retry_planners, allow_circ, ctx.logger, &leg.planner);
  }
  mg.setPlanningTime(ctx.planning_time);
  if (!ok || plan.trajectory.joint_trajectory.points.empty()) return false;

  leg.trajectory = plan.trajectory;
  leg.duration = rclcpp::Duration(plan.trajectory.joint_trajectory.points.back().time_from_start).seconds();
  leg.joint_motion = trajectory_joint_motion(plan.trajectory);
  return true;
}

// ============================================================================
// Nodes of the tour: global candidate ids
// ============================================================================
// Every IK candidate has a global id; START and HOME are the two fixed ends of
// the tour (the scan starts from the current state and ends at the home pose).
constexpr int START = -1;
constexpr int HOME  = -2;

struct CandidateTable {
  std::vector<int> first_id;                 // per waypoint: global id of its candidate 0
  std::vector<std::pair<int, int>> owner;    // per global id: (waypoint, candidate)
  std::vector<const std::vector<double> *> joints;
};

// ============================================================================
// MAIN
// ============================================================================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node   = rclcpp::Node::make_shared("scan_tsp_node");
  auto logger = node->get_logger();
  rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_WARN);

  using TriggerSrv = std_srvs::srv::Trigger;
  auto start_srv = node->create_service<TriggerSrv>(
    "~/start",
    [](const std::shared_ptr<TriggerSrv::Request>, std::shared_ptr<TriggerSrv::Response> res) {
      g_paused = false;
      res->success = true;
      res->message = "execution started/resumed";
    });
  auto pause_srv = node->create_service<TriggerSrv>(
    "~/pause",
    [](const std::shared_ptr<TriggerSrv::Request>, std::shared_ptr<TriggerSrv::Response> res) {
      g_paused = true;
      res->success = true;
      res->message = "execution paused (stops after the current motion)";
    });
  // mode:=execute shows the waypoints of the plan. Created here, well before
  // the first publication, so RViz has time to connect.
  auto markers_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/scan_waypoints_markers", 10);

  // --- Parameters: same names as scan_sequence_node, plus the plan ones ---
  node->declare_parameter<std::string>("global_frame",              "world");
  node->declare_parameter<std::string>("planning_group",            "ur_manipulator");
  node->declare_parameter<std::string>("end_effector_link",         "ee_automata_tcp");
  node->declare_parameter<std::string>("home_pose_name",            "home");
  node->declare_parameter<std::string>("lower_home_pose_name",      "lower_scan_ready");
  node->declare_parameter<double>     ("trajectory_scaling_factor", 0.1);
  node->declare_parameter<std::vector<double>>("scan_center",       {0.0, 0.4, 0.5});
  node->declare_parameter<double>     ("scan_radius",               0.35);
  node->declare_parameter<std::string>("scan_hemisphere",           "upper");
  node->declare_parameter<std::string>("scan_direction",            "latitudinal");
  node->declare_parameter<int>        ("scan_num_rings",            4);
  node->declare_parameter<int>        ("scan_points_per_ring",      8);
  node->declare_parameter<int>        ("scan_num_arc",              6);
  node->declare_parameter<int>        ("scan_points_per_arc",       5);
  node->declare_parameter<double>     ("scan_equator_exclusion_upper_deg", 10.0);
  node->declare_parameter<double>     ("scan_equator_exclusion_lower_deg", 10.0);
  node->declare_parameter<bool>       ("scan_lock_pitch",           true);
  node->declare_parameter<bool>       ("scan_stagger_rings",        false);
  node->declare_parameter<bool>       ("scan_adaptive_rings",       false);
  node->declare_parameter<bool>       ("scan_occlusion_check",      false);
  node->declare_parameter<double>     ("scan_occlusion_disk_radius", 0.15);
  node->declare_parameter<double>     ("scan_occlusion_margin",     0.01);
  node->declare_parameter<bool>       ("scan_fallback_search",      false);
  node->declare_parameter<double>     ("scan_fallback_radius_mm",   20.0);
  node->declare_parameter<std::vector<std::string>>("scan_planners",
                                                    std::vector<std::string>{"pilz_ptp", "ompl"});
  node->declare_parameter<std::string>("scan_ompl_algorithm",       "RRTConnect");
  node->declare_parameter<std::string>("scan_retry_ompl_algorithm", "RRTConnect");
  node->declare_parameter<std::vector<double>>("scan_retry_planning_times", std::vector<double>{});
  node->declare_parameter<double>     ("scan_pitch_search_range_deg", 90.0);
  node->declare_parameter<double>     ("scan_pitch_search_step_deg",  15.0);
  node->declare_parameter<double>     ("scan_pitch_xparallel_bias",   0.05);
  node->declare_parameter<double>     ("scan_enum_ik_timeout",        0.003);
  node->declare_parameter<double>     ("scan_planning_time",          5.0);
  node->declare_parameter<int>        ("scan_planning_attempts",      1);
  node->declare_parameter<int>        ("scan_sectors",                0);
  node->declare_parameter<double>     ("scan_sector_offset_deg",      0.0);
  // plan | execute
  node->declare_parameter<std::string>("mode",                 "plan");
  // tsp | gtsp | sectors | alternate
  node->declare_parameter<std::string>("order",                "alternate");
  node->declare_parameter<std::string>("plan_file",            "");
  // OR-Tools search time per solve (s).
  node->declare_parameter<double>     ("tsp_time_limit",       10.0);
  // Extra rounds with measured durations after the first plan (0 = none).
  node->declare_parameter<int>        ("refine_iterations",    3);
  // Estimated extra time (s) of a segment whose straight line is blocked,
  // until its real (OMPL) duration is measured.
  node->declare_parameter<double>     ("blocked_penalty_s",    10.0);
  // order:=tsp|gtsp: before solving, check the straight lines toward this many
  // cheapest neighbour waypoints (0 = no check, costs ignore collisions).
  node->declare_parameter<int>        ("tsp_check_neighbours", 10);
  // execute: largest joint difference (rad) between the current state and
  // the state the plan starts from; above it the robot first moves there.
  node->declare_parameter<double>     ("start_tolerance_rad",  0.01);
  // execute: speed relative to the plan, 1.0 = as planned, 0.5 = half speed.
  // Only slower: values above 1.0 are refused.
  node->declare_parameter<double>     ("speed",                1.0);

  const std::string global_frame    = node->get_parameter("global_frame").as_string();
  const std::string planning_group  = node->get_parameter("planning_group").as_string();
  const std::string ee_link         = node->get_parameter("end_effector_link").as_string();
  const std::string home_pose_name  = node->get_parameter("home_pose_name").as_string();
  const std::string lower_home_pose_name = node->get_parameter("lower_home_pose_name").as_string();
  const double      scaling         = node->get_parameter("trajectory_scaling_factor").as_double();
  const auto        center_vec      = node->get_parameter("scan_center").as_double_array();
  const double      radius          = node->get_parameter("scan_radius").as_double();
  const std::string hemi_str        = node->get_parameter("scan_hemisphere").as_string();
  const std::string dir_str         = node->get_parameter("scan_direction").as_string();
  const bool        lock_pitch      = node->get_parameter("scan_lock_pitch").as_bool();
  const std::vector<std::string> planner_names = node->get_parameter("scan_planners").as_string_array();
  const std::string ompl_algorithm  = node->get_parameter("scan_ompl_algorithm").as_string();
  const std::string retry_ompl_algorithm = node->get_parameter("scan_retry_ompl_algorithm").as_string();
  const double      pitch_bias      = node->get_parameter("scan_pitch_xparallel_bias").as_double();
  const double      planning_time   = node->get_parameter("scan_planning_time").as_double();
  const int         num_sectors     = node->get_parameter("scan_sectors").as_int();
  const double      sector_offset_deg = node->get_parameter("scan_sector_offset_deg").as_double();
  const std::string mode            = node->get_parameter("mode").as_string();
  const std::string order_method    = node->get_parameter("order").as_string();
  const std::string plan_path       = node->get_parameter("plan_file").as_string();
  const double      tsp_time_limit  = node->get_parameter("tsp_time_limit").as_double();
  const int         refine_iterations = node->get_parameter("refine_iterations").as_int();
  const double      blocked_penalty_s = node->get_parameter("blocked_penalty_s").as_double();
  const int         tsp_check_neighbours = node->get_parameter("tsp_check_neighbours").as_int();
  const double      start_tolerance = node->get_parameter("start_tolerance_rad").as_double();

  auto fail = [&](const std::string & message) {
    std::fprintf(stderr, "%s\n", message.c_str());
    rclcpp::shutdown();
    return 1;
  };
  if (center_vec.size() != 3) return fail("scan_center must have exactly 3 values (x, y, z).");
  if (mode != "plan" && mode != "execute") return fail("mode must be plan or execute.");
  if (order_method != "tsp" && order_method != "gtsp" && order_method != "sectors" &&
      order_method != "alternate") {
    return fail("order must be tsp, gtsp, sectors or alternate.");
  }
  if (plan_path.empty()) return fail("plan_file is empty.");
  const double speed = node->get_parameter("speed").as_double();
  if (speed <= 0.0 || speed > 1.0) return fail("speed must be greater than 0 and at most 1.0.");

  // --- Waypoints: same generator and parameters as scan_sequence_node ---
  ScanConfig cfg;
  cfg.center = Eigen::Vector3d(center_vec[0], center_vec[1], center_vec[2]);
  cfg.radius = radius;
  cfg.num_rings       = node->get_parameter("scan_num_rings").as_int();
  cfg.points_per_ring = node->get_parameter("scan_points_per_ring").as_int();
  cfg.num_arc         = node->get_parameter("scan_num_arc").as_int();
  cfg.points_per_arc  = node->get_parameter("scan_points_per_arc").as_int();
  cfg.equator_exclusion_upper_rad = node->get_parameter("scan_equator_exclusion_upper_deg").as_double() * M_PI / 180.0;
  cfg.equator_exclusion_lower_rad = node->get_parameter("scan_equator_exclusion_lower_deg").as_double() * M_PI / 180.0;
  cfg.stagger_rings  = node->get_parameter("scan_stagger_rings").as_bool();
  cfg.adaptive_rings = node->get_parameter("scan_adaptive_rings").as_bool();
  if      (hemi_str == "upper") cfg.hemisphere = HEMISPHERE_UPPER;
  else if (hemi_str == "lower") cfg.hemisphere = HEMISPHERE_LOWER;
  else if (hemi_str == "full")  cfg.hemisphere = HEMISPHERE_FULL;
  else return fail("scan_hemisphere must be upper, lower or full.");
  if      (dir_str == "latitudinal")  cfg.direction = SCAN_LATITUDINAL;
  else if (dir_str == "longitudinal") cfg.direction = SCAN_LONGITUDINAL;
  else return fail("scan_direction must be latitudinal or longitudinal.");

  std::vector<geometry_msgs::msg::Pose> upper_pts, lower_pts;
  if (cfg.hemisphere == HEMISPHERE_FULL) {
    ScanConfig cfg_upper = cfg; cfg_upper.hemisphere = HEMISPHERE_UPPER;
    ScanConfig cfg_lower = cfg; cfg_lower.hemisphere = HEMISPHERE_LOWER;
    upper_pts = generate_waypoints(cfg_upper);
    lower_pts = generate_waypoints(cfg_lower);
  } else if (cfg.hemisphere == HEMISPHERE_UPPER) {
    upper_pts = generate_waypoints(cfg);
  } else {
    lower_pts = generate_waypoints(cfg);
  }

  // Settings that decide the waypoints and which IK solutions are valid: saved
  // with the plan and compared before executing it.
  std::map<std::string, std::string> waypoint_params;
  for (const char * name : {"scan_hemisphere", "scan_direction", "scan_num_rings", "scan_points_per_ring",
                            "scan_num_arc", "scan_points_per_arc", "scan_equator_exclusion_upper_deg",
                            "scan_equator_exclusion_lower_deg", "scan_lock_pitch", "scan_stagger_rings",
                            "scan_adaptive_rings", "scan_occlusion_check", "scan_occlusion_disk_radius",
                            "scan_occlusion_margin", "scan_fallback_search", "scan_fallback_radius_mm",
                            "scan_pitch_search_range_deg", "scan_pitch_search_step_deg",
                            "scan_sectors", "scan_sector_offset_deg"}) {
    waypoint_params[name] = node->get_parameter(name).value_to_string();
  }

  // --- MoveIt ---
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });
  auto shutdown = [&](int code) {
    rclcpp::shutdown();
    spinner.join();
    return code;
  };

  MoveGroup move_group(node, planning_group);
  move_group.setEndEffectorLink(ee_link);
  move_group.setPoseReferenceFrame(global_frame);
  move_group.setMaxVelocityScalingFactor(scaling);
  move_group.setMaxAccelerationScalingFactor(scaling);
  move_group.setPlanningTime(planning_time);
  move_group.setNumPlanningAttempts(node->get_parameter("scan_planning_attempts").as_int());
  move_group.setWorkspace(-0.75, -0.625, -0.25, +0.75, +1.125, +1.25);   // as scan_sequence_node

  const moveit::core::RobotModelConstPtr model = move_group.getRobotModel();
  const moveit::core::JointModelGroup * jmg = model->getJointModelGroup(planning_group);
  const std::vector<std::string> joint_names = jmg->getVariableNames();
  planning_scene::PlanningScenePtr scene = fetch_planning_scene(node, model, logger);
  moveit::core::RobotStatePtr current_ptr = move_group.getCurrentState(10.0);
  if (!current_ptr) {
    std::fprintf(stderr, "Stato attuale del robot non disponibile (/joint_states?).\n");
    return shutdown(1);
  }
  moveit::core::RobotState current(*current_ptr);
  std::vector<double> current_joints;
  current.copyJointGroupPositions(jmg, current_joints);

  // Joint limits of the model: velocity from the URDF, acceleration from
  // joint_limits.yaml (the launch file passes it as robot_description_planning).
  std::vector<double> max_velocity, max_acceleration;
  for (const std::string & name : joint_names) {
    const moveit::core::VariableBounds & b = model->getVariableBounds(name);
    if (!b.velocity_bounded_ || !b.acceleration_bounded_) {
      std::fprintf(stderr, "Giunto %s senza limite di velocita' o accelerazione nel modello: "
                   "lanciare con scan_tsp.launch.py (carica joint_limits.yaml).\n", name.c_str());
      return shutdown(1);
    }
    max_velocity.push_back(b.max_velocity_);
    max_acceleration.push_back(b.max_acceleration_);
  }

  // Visit list, numbered like the table of scan_sequence_node (#N).
  std::vector<geometry_msgs::msg::Pose> waypoints;
  std::vector<SectorBlock> blocks;
  if (num_sectors > 0) {
    Eigen::Vector3d base_pos = current.getGlobalLinkTransform("base_link").translation();
    double phi_front = std::atan2(base_pos.y() - cfg.center.y(), base_pos.x() - cfg.center.x())
                     + sector_offset_deg * M_PI / 180.0;
    order_by_sectors(upper_pts, cfg.center, phi_front, num_sectors, true,  waypoints, blocks);
    order_by_sectors(lower_pts, cfg.center, phi_front, num_sectors, false, waypoints, blocks);
  } else {
    waypoints.insert(waypoints.end(), upper_pts.begin(), upper_pts.end());
    waypoints.insert(waypoints.end(), lower_pts.begin(), lower_pts.end());
  }
  const size_t upper_count = upper_pts.size();
  std::vector<std::vector<double>> waypoint_values;
  for (const auto & p : waypoints) {
    waypoint_values.push_back({p.position.x, p.position.y, p.position.z,
                               p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w});
  }

  // Camera pose at the end of a recorded motion, with the current model.
  auto camera_pose_of = [&](const RecordedSegment & seg, Eigen::Isometry3d & camera) {
    moveit::core::RobotState st(current);
    st.setVariablePositions(seg.joint_names, seg.points.back().positions);
    st.update();
    camera = st.getGlobalLinkTransform(ee_link);
  };

  // ==========================================================================
  // EXECUTE: load, check, run
  // ==========================================================================
  if (mode == "execute") {
    const auto load_t0 = std::chrono::steady_clock::now();
    ScanPlan saved;
    std::string error;
    if (!load_plan(plan_path, saved, error)) {
      std::fprintf(stderr, "Piano non leggibile: %s\nPianificare con mode:=plan.\n", error.c_str());
      return shutdown(1);
    }
    const double load_s = seconds_since(load_t0);
    const auto check_t0 = std::chrono::steady_clock::now();
    const PlanData & d = saved.data;
    const ScanRecording & rec = saved.recording;
    std::printf("Piano: %s (%s, ordine %s)  %zu movimenti, %d/%d waypoint\n", plan_path.c_str(),
                rec.created.c_str(), d.method.c_str(), rec.segments.size(),
                rec.waypoints_reached, rec.waypoints_total);

    // Every reason is collected, then reported together.
    std::vector<std::string> reasons;
    char buf[256];
    if (rec.planning_group != planning_group || rec.end_effector_link != ee_link || rec.global_frame != global_frame) {
      reasons.push_back("gruppo, end effector o frame diversi da quelli attuali");
    }
    if (d.joint_names != joint_names) reasons.push_back("giunti del gruppo diversi da quelli del modello");
    for (const auto & kv : waypoint_params) {
      auto it = d.waypoint_params.find(kv.first);
      if (it == d.waypoint_params.end() || it->second != kv.second) {
        std::snprintf(buf, sizeof(buf), "parametro %s: nel piano %s, ora %s", kv.first.c_str(),
                      it == d.waypoint_params.end() ? "assente" : it->second.c_str(), kv.second.c_str());
        reasons.push_back(buf);
      }
    }
    if (d.waypoints.size() != waypoint_values.size()) {
      std::snprintf(buf, sizeof(buf), "waypoint: %zu nel piano, %zu ora", d.waypoints.size(), waypoint_values.size());
      reasons.push_back(buf);
    } else {
      int moved = 0;
      for (size_t i = 0; i < waypoint_values.size(); ++i) {
        const std::vector<double> & a = d.waypoints[i];
        const std::vector<double> & b = waypoint_values[i];
        double dist = std::sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) + (a[2] - b[2]) * (a[2] - b[2]));
        double angle = Eigen::Quaterniond(a[6], a[3], a[4], a[5]).angularDistance(Eigen::Quaterniond(b[6], b[3], b[4], b[5]));
        if (dist > 0.001 || angle > M_PI / 180.0) ++moved;
      }
      if (moved > 0) {
        std::snprintf(buf, sizeof(buf), "%d waypoint in posizione diversa (centro o raggio della sfera cambiati)", moved);
        reasons.push_back(buf);
      }
    }
    if (std::abs(d.scaling - scaling) > 1e-6) {
      std::snprintf(buf, sizeof(buf), "trajectory_scaling_factor: nel piano %.3f, ora %.3f", d.scaling, scaling);
      reasons.push_back(buf);
    }
    for (size_t k = 0; k < joint_names.size() && k < d.max_velocity.size() && k < d.max_acceleration.size(); ++k) {
      if (std::abs(d.max_velocity[k] - max_velocity[k]) > 1e-6 ||
          std::abs(d.max_acceleration[k] - max_acceleration[k]) > 1e-6) {
        std::snprintf(buf, sizeof(buf), "limiti di velocita'/accelerazione di %s cambiati", joint_names[k].c_str());
        reasons.push_back(buf);
      }
    }
    // Start state: if the robot is not there, it goes there after /start.
    bool move_to_start = false;
    if (d.start_joints.size() != current_joints.size()) {
      reasons.push_back("stato iniziale del piano assente o con un numero di giunti diverso");
    } else {
      double worst = 0.0;
      for (size_t k = 0; k < current_joints.size(); ++k) {
        worst = std::max(worst, std::abs(d.start_joints[k] - current_joints[k]));
      }
      if (worst > start_tolerance) {
        move_to_start = true;
        std::printf("Robot a %.3f rad dallo stato iniziale del piano (tolleranza %.3f): "
                    "ci andra' dopo /start.\n", worst, start_tolerance);
      }
    }
    // Scene objects.
    if (!scene) {
      reasons.push_back("planning scene non disponibile: impossibile controllare scena e collisioni");
    } else {
      std::map<std::string, std::vector<double>> now_boxes;
      for (const SceneObjectBox & b : scene_boxes(*scene)) now_boxes[b.id] = b.box;
      std::set<std::string> saved_ids;
      for (const SceneObjectBox & b : d.scene) {
        saved_ids.insert(b.id);
        auto it = now_boxes.find(b.id);
        if (it == now_boxes.end()) {
          reasons.push_back("oggetto '" + b.id + "' della scena non c'e' piu'");
          continue;
        }
        double worst = 0.0;
        for (size_t k = 0; k < 6; ++k) worst = std::max(worst, std::abs(it->second[k] - b.box[k]));
        if (worst > 0.001) {
          std::snprintf(buf, sizeof(buf), "oggetto '%s' spostato o cambiato (%.1f mm)", b.id.c_str(), worst * 1000.0);
          reasons.push_back(buf);
        }
      }
      for (const auto & kv : now_boxes) {
        if (saved_ids.count(kv.first) == 0) reasons.push_back("oggetto '" + kv.first + "' nuovo nella scena");
      }
    }
    // Camera poses with the current model, joints known, no collisions.
    int camera_mismatches = 0;
    bool joints_known = true;
    std::string collision_at;
    for (const RecordedSegment & seg : rec.segments) {
      for (const std::string & name : seg.joint_names) {
        if (!model->hasJointModel(name)) joints_known = false;
      }
      if (!joints_known) break;
      if (seg.camera_pose.size() == 7) {
        Eigen::Isometry3d camera;
        camera_pose_of(seg, camera);
        const Eigen::Vector3d p(seg.camera_pose[0], seg.camera_pose[1], seg.camera_pose[2]);
        const Eigen::Quaterniond q(seg.camera_pose[6], seg.camera_pose[3], seg.camera_pose[4], seg.camera_pose[5]);
        if ((camera.translation() - p).norm() > 0.002 ||
            Eigen::Quaterniond(camera.linear()).angularDistance(q) > M_PI / 180.0) {
          ++camera_mismatches;
        }
      }
      if (scene && collision_at.empty()) {
        moveit::core::RobotState st(current);
        for (const RecordedPoint & p : seg.points) {
          st.setVariablePositions(seg.joint_names, p.positions);
          st.update();
          if (scene->isStateColliding(st, planning_group)) {
            std::snprintf(buf, sizeof(buf), "il movimento '%s' (t = %.2f s) collide con la scena attuale",
                          seg.label.c_str(), p.time);
            collision_at = buf;
            break;
          }
        }
      }
    }
    if (!joints_known) reasons.push_back("il piano usa giunti assenti nel modello");
    if (camera_mismatches > 0) {
      std::snprintf(buf, sizeof(buf), "%d waypoint: la camera non torna con il modello attuale (TCP, end effector "
                    "o base cambiati)", camera_mismatches);
      reasons.push_back(buf);
    }
    if (!collision_at.empty()) reasons.push_back(collision_at);
    const double check_s = seconds_since(check_t0);

    if (!reasons.empty()) {
      std::fprintf(stderr, "\nPiano NON riutilizzabile:\n");
      for (const std::string & r : reasons) std::fprintf(stderr, "  - %s\n", r.c_str());
      std::fprintf(stderr, "Ripianificare: ros2 launch ur_automata_scan_tsp scan_tsp.launch.py mode:=plan\n");
      return shutdown(1);
    }
    double motion_s = 0.0;
    for (const RecordedSegment & seg : rec.segments) motion_s += seg.points.back().time / speed;
    std::printf("Controlli superati. Caricamento %.3f s, controlli %.3f s. Movimenti del piano: %.1f s (speed %.2f)\n",
                load_s, check_s, motion_s, speed);
    // Gray camera frustums on the waypoints of the plan.
    visualization_msgs::msg::MarkerArray markers = make_frustum_markers(rec, global_frame);
    publish_first_frustums(markers_pub, markers);

    std::printf("In pausa: ros2 service call /scan_tsp_node/start std_srvs/srv/Trigger {}\n");
    std::fflush(stdout);

    while (g_paused.load() && rclcpp::ok()) std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (!rclcpp::ok()) return shutdown(1);

    // Planned motion to the start state of the plan (as scan_replay_node),
    // slowed like the rest. Not counted in the execution time.
    if (move_to_start) {
      move_group.setMaxVelocityScalingFactor(scaling * speed);
      move_group.setMaxAccelerationScalingFactor(scaling * speed);
      move_group.setJointValueTarget(d.start_joints);
      MoveGroup::Plan start_plan;
      bool start_ok = false;
      const std::string pipelines[2][2] = {{"pilz_industrial_motion_planner", "PTP"},
                                           {"ompl", "RRTConnectkConfigDefault"}};
      for (const auto & pipeline : pipelines) {
        move_group.setPlanningPipelineId(pipeline[0]);
        move_group.setPlannerId(pipeline[1]);
        if (move_group.plan(start_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
          start_ok = move_group.execute(start_plan) == moveit::core::MoveItErrorCode::SUCCESS;
          break;
        }
      }
      if (!start_ok) {
        std::fprintf(stderr, "Impossibile portare il robot nello stato iniziale del piano.\n");
        return shutdown(1);
      }
      std::printf("Robot nello stato iniziale del piano.\n");
      std::fflush(stdout);
    }

    // The execution time starts after /start and the motion to the start state.
    const auto exec_t0 = std::chrono::steady_clock::now();
    int waypoints_done = 0;
    bool completed = true;
    for (size_t k = 0; k < rec.segments.size(); ++k) {
      while (g_paused.load() && rclcpp::ok()) std::this_thread::sleep_for(std::chrono::milliseconds(100));
      if (!rclcpp::ok()) {
        completed = false;
        break;
      }
      const RecordedSegment seg = scale_segment_speed(rec.segments[k], speed);
      moveit_msgs::msg::RobotTrajectory traj;
      traj.joint_trajectory.joint_names = seg.joint_names;
      for (const RecordedPoint & p : seg.points) {
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions       = p.positions;
        point.velocities      = p.velocities;
        point.accelerations   = p.accelerations;
        point.time_from_start = rclcpp::Duration::from_seconds(p.time);
        traj.joint_trajectory.points.push_back(point);
      }
      if (move_group.execute(traj) != moveit::core::MoveItErrorCode::SUCCESS) {
        std::fprintf(stderr, "Movimento %zu/%zu ('%s') non eseguito: esecuzione interrotta.\n",
                     k + 1, rec.segments.size(), seg.label.c_str());
        completed = false;
        break;
      }
      if (seg.waypoint >= 0) {
        ++waypoints_done;
        // The camera is on waypoint seg.waypoint: this is where the photo will be taken.
        set_frustum_reached(markers, k);
        markers_pub->publish(markers);
      }
      std::printf("  %3zu/%zu  %-14s  (%.1f s)\n", k + 1, rec.segments.size(), seg.label.c_str(),
                  seconds_since(exec_t0));
      std::fflush(stdout);
    }
    std::printf("\nEsecuzione %s: %d waypoint in %.1f s (movimenti del piano %.1f s) | "
                "riuso: caricamento %.3f s + controlli %.3f s\n",
                completed ? "completata" : "INTERROTTA", waypoints_done, seconds_since(exec_t0),
                motion_s, load_s, check_s);
    std::fflush(stdout);
    return shutdown(completed ? 0 : 1);
  }

  // ==========================================================================
  // PLAN
  // ==========================================================================
  const auto plan_t0 = std::chrono::steady_clock::now();

  // --- 1. IK candidates ---
  EnumSettings es;
  es.group = planning_group;
  es.ee_link = ee_link;
  es.center = cfg.center;
  es.radius = cfg.radius;
  es.pitch_offsets = build_pitch_offsets_rad(node->get_parameter("scan_pitch_search_range_deg").as_double(),
                                             node->get_parameter("scan_pitch_search_step_deg").as_double());
  if (lock_pitch) es.pitch_offsets = {0.0};
  es.ik_timeout = node->get_parameter("scan_enum_ik_timeout").as_double();
  es.occlusion_check = node->get_parameter("scan_occlusion_check").as_bool();
  es.occlusion_disk_radius = node->get_parameter("scan_occlusion_disk_radius").as_double();
  es.occlusion_margin = node->get_parameter("scan_occlusion_margin").as_double();
  es.fallback_search = node->get_parameter("scan_fallback_search").as_bool();
  es.fallback_radius_mm = node->get_parameter("scan_fallback_radius_mm").as_double();
  const EnumResult cands = enumerate_candidates(es, waypoints, current, scene);
  if (!rclcpp::ok()) return shutdown(1);

  std::vector<int> reachable;   // waypoints with at least one candidate
  int view_blocked = 0;
  CandidateTable table;
  table.first_id.assign(waypoints.size(), -1);
  for (size_t i = 0; i < waypoints.size(); ++i) {
    if (!cands.view_blocked_by[i].empty()) ++view_blocked;
    if (cands.layers[i].empty()) continue;
    reachable.push_back(static_cast<int>(i));
    table.first_id[i] = static_cast<int>(table.owner.size());
    for (size_t c = 0; c < cands.layers[i].size(); ++c) {
      table.owner.push_back({static_cast<int>(i), static_cast<int>(c)});
      table.joints.push_back(&cands.layers[i][c].joints);
    }
  }
  const int unreachable = static_cast<int>(waypoints.size() - reachable.size()) - view_blocked;
  std::printf("Candidati: %zu su %zu waypoint raggiungibili (%d vista coperta, %d irraggiungibili) in %.1f s\n",
              cands.total_candidates, reachable.size(), view_blocked, unreachable, cands.seconds);

  moveit::core::RobotState home_state(current);
  home_state.setToDefaultValues(jmg, home_pose_name);
  std::vector<double> home_joints;
  home_state.copyJointGroupPositions(jmg, home_joints);

  auto joints_of = [&](int id) -> const std::vector<double> & {
    if (id == START) return current_joints;
    if (id == HOME) return home_joints;
    return *table.joints[id];
  };
  auto pitch_cost = [&](int id) -> double {
    if (id < 0) return 0.0;
    return pitch_bias * std::abs(cands.layers[table.owner[id].first][table.owner[id].second].pitch_offset_rad);
  };

  // --- Segment costs: measured duration when known, else PTP estimate ---
  std::vector<double> scaled_velocity, scaled_acceleration;
  for (size_t k = 0; k < max_velocity.size(); ++k) {
    scaled_velocity.push_back(max_velocity[k] * scaling);
    scaled_acceleration.push_back(max_acceleration[k] * scaling);
  }
  std::map<std::pair<int, int>, double> measured;    // (from id, to id) -> planned duration (s)
  std::map<std::pair<int, int>, bool> line_free;     // straight joint line checked: free or not
  auto estimate = [&](int a, int b) { return ptp_time(joints_of(a), joints_of(b), scaled_velocity, scaled_acceleration); };
  // Straight joint line a -> b collision-free? Checked once, then cached.
  auto line_check = [&](int a, int b) -> bool {
    auto it = line_free.find({a, b});
    if (it != line_free.end()) return it->second;
    bool is_free = segment_is_free(scene, current, jmg, joints_of(a), joints_of(b));
    line_free[{a, b}] = is_free;
    return is_free;
  };
  auto segment_cost = [&](int a, int b) -> double {
    auto m = measured.find({a, b});
    if (m != measured.end()) return m->second;
    double t = estimate(a, b);
    auto f = line_free.find({a, b});
    if (f != line_free.end() && !f->second) t += blocked_penalty_s;
    return t;
  };

  // --- Planning context and cache of planned segments ---
  PlanContext ctx;
  ctx.move_group = &move_group;
  ctx.jmg = jmg;
  ctx.reference = &current;
  ctx.planning_time = planning_time;
  ctx.retry_times = node->get_parameter("scan_retry_planning_times").as_double_array();
  ctx.logger = logger;
  for (const std::string & name : planner_names) {
    PlannerChoice choice;
    if (resolve_planner(name, ompl_algorithm, choice)) ctx.planners.chain.push_back(choice);
  }
  if (ctx.planners.chain.empty()) {
    std::fprintf(stderr, "No valid planner in scan_planners.\n");
    return shutdown(1);
  }
  ctx.planners.circ_center = make_circ_center_constraint(global_frame, ee_link, cfg.center);
  ctx.retry_planners = ctx.planners;
  for (PlannerChoice & choice : ctx.retry_planners.chain) {
    if (choice.pipeline == "ompl") resolve_planner("ompl", retry_ompl_algorithm, choice);
  }
  std::map<std::pair<int, int>, Segment> planned;
  double moveit_seconds = 0.0;

  auto recovery_joints = [&](const std::string & name) {
    moveit::core::RobotState st(current);
    st.setToDefaultValues(jmg, name);
    std::vector<double> q;
    st.copyJointGroupPositions(jmg, q);
    return q;
  };
  auto label_of = [&](int id) {
    if (id == HOME) return std::string("posa ") + home_pose_name;
    return "wp " + std::to_string(table.owner[id].first);
  };

  // Plans the segment a -> b (cached): direct, else through the recovery pose
  // of b's hemisphere, else through home (recover_to_safe_pose of scan_sequence_node).
  auto plan_segment = [&](int a, int b) -> const Segment & {
    auto it = planned.find({a, b});
    if (it != planned.end()) return it->second;
    const auto t0 = std::chrono::steady_clock::now();
    Segment seg;
    Leg leg;
    leg.label = label_of(b);
    leg.waypoint = (b >= 0) ? table.owner[b].first : -1;
    const bool from_sphere = (a >= 0);   // CIRC only from a waypoint
    if (plan_leg(ctx, joints_of(a), joints_of(b), from_sphere && b >= 0, leg)) {
      seg.ok = true;
      seg.legs.push_back(leg);
    } else {
      seg.via_recovery = true;
      std::vector<std::string> recovery_names;
      const bool lower = (b >= 0) && static_cast<size_t>(table.owner[b].first) >= upper_count;
      recovery_names.push_back(lower ? lower_home_pose_name : home_pose_name);
      if (recovery_names.front() != home_pose_name) recovery_names.push_back(home_pose_name);
      for (const std::string & name : recovery_names) {
        const std::vector<double> r = recovery_joints(name);
        Leg to_recovery, from_recovery = leg;
        to_recovery.label = "posa " + name;
        if (plan_leg(ctx, joints_of(a), r, false, to_recovery) &&
            plan_leg(ctx, r, joints_of(b), false, from_recovery)) {
          seg.ok = true;
          seg.legs = {to_recovery, from_recovery};
          break;
        }
      }
    }
    moveit_seconds += seconds_since(t0);
    return planned.emplace(std::make_pair(a, b), seg).first->second;
  };

  // One full candidate plan: the tour as global ids (START ... HOME).
  struct Tour {
    std::vector<int> ids;          // START, candidates..., HOME
    std::vector<int> visit_order;  // waypoints, in visit order
    double estimated = 0.0;        // sum of the pure PTP estimates
    double measured_time = 0.0;
    double joint_motion = 0.0;
    int reached = 0;
    int unplannable = 0;           // segments with no direct motion (recovery or dropped)
    int dropped = 0;               // waypoints skipped because nothing reached them
    int recoveries = 0;            // segments planned through a recovery pose
    std::map<std::string, int> planners;
    std::vector<Leg> legs;
  };

  // --- 2 + 3. Order, then IK configurations with the lazy DP ---
  // Layered DP (with lazy segment checks) for a visit order. `chain`
  // receives START, the chosen candidate of each waypoint, HOME. Returns the
  // DP cost (estimated seconds + pitch preference).
  auto run_dp = [&](const std::vector<int> & order, std::vector<int> & chain, int & dp_iterations) -> double {
    // Layers in visit order, plus a last layer with the home pose.
    std::vector<std::vector<SeqCandidate>> layers;
    for (int wp : order) {
      std::vector<SeqCandidate> layer;
      for (const Candidate & c : cands.layers[wp]) {
        SeqCandidate sc;
        sc.joints = c.joints;
        sc.extra_cost = pitch_bias * std::abs(c.pitch_offset_rad);
        layer.push_back(sc);
      }
      layers.push_back(layer);
    }
    layers.push_back({SeqCandidate{home_joints, 0.0}});
    auto id_at = [&](int layer, int c) {
      if (layer < 0) return START;
      if (layer == static_cast<int>(order.size())) return HOME;
      return table.first_id[order[layer]] + c;
    };
    SeqCost dp_cost;
    dp_cost.blocked_penalty = 0.0;   // the penalty is already in segment_cost
    dp_cost.edge_cost = [&](int pl, int pc, int l, int c) { return segment_cost(id_at(pl, pc), id_at(l, c)); };
    EdgeFilter known_free = [&](int, int, int, int) { return true; };
    EdgeFilter check = [&](int pl, int pc, int l, int c) {
      const int a = id_at(pl, pc), b = id_at(l, c);
      if (measured.count({a, b})) return true;   // real duration known: no guess needed
      return line_check(a, b);
    };
    SeqResult seq = choose_sequence_lazy(current_joints, layers, dp_cost, known_free, check, 200, &dp_iterations);
    chain.assign(1, START);
    for (size_t k = 0; k < layers.size(); ++k) {
      if (seq.chosen[k] < 0) continue;
      chain.push_back(id_at(static_cast<int>(k), seq.chosen[k]));
    }
    return seq.total_cost;
  };

  auto build_tour = [&](int round, double & order_s, double & dp_s, std::string & solver_status) -> Tour {
    auto t0 = std::chrono::steady_clock::now();
    std::vector<int> order;   // waypoints
    solver_status = "-";
    if (order_method == "sectors") {
      order = reachable;
    } else if (order_method == "tsp") {
      // Nodes: 0 = start, 1..n = reachable waypoints, n + 1 = home. Cost of
      // i -> j = cheapest pair of their candidates (a lower bound of the real
      // segment: the DP chooses the candidates afterwards).
      const int n = static_cast<int>(reachable.size());
      const int size = n + 2;
      std::vector<int64_t> cost(static_cast<size_t>(size) * size, 0);
      auto ids_of = [&](int node_index) {
        std::vector<int> ids;
        if (node_index == 0) ids.push_back(START);
        else if (node_index == n + 1) ids.push_back(HOME);
        else {
          int wp = reachable[node_index - 1];
          for (size_t c = 0; c < cands.layers[wp].size(); ++c) ids.push_back(table.first_id[wp] + static_cast<int>(c));
        }
        return ids;
      };
      std::vector<std::vector<int>> node_ids(size);
      for (int k = 0; k < size; ++k) node_ids[k] = ids_of(k);
      // Cheapest pair of candidates between two nodes (a, b receive the pair).
      auto best_pair = [&](int x, int y, int & best_a, int & best_b) {
        double best = std::numeric_limits<double>::infinity();
        for (int a : node_ids[x]) {
          for (int b : node_ids[y]) {
            double c = segment_cost(a, b);
            if (c < best) {
              best = c;
              best_a = a;
              best_b = b;
            }
          }
        }
        return best;
      };
      std::vector<double> best_cost(static_cast<size_t>(size) * size, 0.0);
      for (int x = 0; x < size; ++x) {
        for (int y = 0; y < size; ++y) {
          if (x == y || y == 0 || x == n + 1 || (x == 0 && y == n + 1)) continue;
          int a = 0, b = 0;
          best_cost[static_cast<size_t>(x) * size + y] = best_pair(x, y, a, b);
        }
      }
      // Collision-aware costs toward the nearest nodes: the cheapest pair is
      // checked with the straight joint line; if it collides the penalty
      // applies and the next pair is tried. Without this the TSP jumps
      // between the hemispheres through the platform (straight lines blocked,
      // the segments go to OMPL). Far nodes keep the optimistic cost: the tour
      // does not use them.
      for (int x = 0; x <= n && tsp_check_neighbours > 0; ++x) {
        std::vector<std::pair<double, int>> near;
        for (int y = 1; y < size; ++y) {
          if (y == x || (x == 0 && y == n + 1)) continue;
          near.push_back({best_cost[static_cast<size_t>(x) * size + y], y});
        }
        std::sort(near.begin(), near.end());
        if (static_cast<int>(near.size()) > tsp_check_neighbours) near.resize(tsp_check_neighbours);
        for (const auto & entry : near) {
          const int y = entry.second;
          double c = 0.0;
          for (int attempt = 0; attempt < 20; ++attempt) {
            int a = 0, b = 0;
            c = best_pair(x, y, a, b);
            if (measured.count({a, b}) || line_check(a, b)) break;
          }
          best_cost[static_cast<size_t>(x) * size + y] = c;
        }
      }
      for (size_t k = 0; k < cost.size(); ++k) cost[k] = static_cast<int64_t>(std::llround(best_cost[k] * 1000.0));
      RouteResult r = solve_route(cost, size, 0, n + 1, {}, tsp_time_limit);
      solver_status = r.status;
      if (!r.ok) {
        std::printf("  OR-Tools non ha trovato un ordine (%s): uso l'ordine a settori.\n", r.status.c_str());
        order = reachable;
      } else {
        for (int k : r.nodes) order.push_back(reachable[k - 1]);
      }
    } else if (order_method == "alternate") {
      // Start from the solution of scan_sequence_node (sector order + DP),
      // then alternate: TSP on the configurations chosen so far (their real
      // segment costs, straight lines checked toward the nearest ones), DP on
      // the new order. A step is kept only if the DP cost goes down, so the
      // estimate never gets worse than the starting solution.
      order = reachable;
      std::vector<int> chain;
      int it = 0;
      double current_cost = run_dp(order, chain, it);
      for (int step = 0; step < 20 && rclcpp::ok(); ++step) {
        const int n = static_cast<int>(chain.size()) - 2;   // chain = START, n candidates, HOME
        const int size = n + 2;
        std::vector<int64_t> cost(static_cast<size_t>(size) * size, 0);
        auto arc = [&](int x, int y) { return segment_cost(chain[x], chain[y]) + pitch_cost(chain[y]); };
        for (int x = 0; x < size; ++x) {
          if (x == n + 1) continue;
          std::vector<std::pair<double, int>> near;
          for (int y = 1; y < size; ++y) {
            if (y == x || (x == 0 && y == n + 1)) continue;
            near.push_back({arc(x, y), y});
          }
          std::sort(near.begin(), near.end());
          for (size_t k = 0; k < near.size() && static_cast<int>(k) < tsp_check_neighbours; ++k) {
            const int y = near[k].second;
            if (!measured.count({chain[x], chain[y]})) line_check(chain[x], chain[y]);
          }
          for (int y = 1; y < size; ++y) {
            if (y == x || (x == 0 && y == n + 1)) continue;
            cost[static_cast<size_t>(x) * size + y] = static_cast<int64_t>(std::llround(arc(x, y) * 1000.0));
          }
        }
        std::vector<int> initial;
        for (int k = 1; k <= n; ++k) initial.push_back(k);
        RouteResult r = solve_route(cost, size, 0, n + 1, {}, tsp_time_limit, initial);
        solver_status = r.status;
        if (!r.ok) break;
        std::vector<int> new_order;
        for (int k : r.nodes) new_order.push_back(table.owner[chain[k]].first);
        std::vector<int> new_chain;
        double new_cost = run_dp(new_order, new_chain, it);
        std::printf("    alternanza %d: costo DP %.1f -> %.1f\n", step + 1, current_cost, new_cost);
        if (new_cost >= current_cost - 1e-3) break;
        order = new_order;
        chain = new_chain;
        current_cost = new_cost;
      }
    } else {   // gtsp
      // Nodes: 0 = start, 1..N = every candidate, N + 1 = home; one cluster
      // per waypoint. OR-Tools picks the order and the candidate together.
      const int n_cand = static_cast<int>(table.owner.size());
      const int size = n_cand + 2;
      auto id_of = [&](int node_index) {
        if (node_index == 0) return START;
        if (node_index == n_cand + 1) return HOME;
        return node_index - 1;
      };
      std::vector<int64_t> cost(static_cast<size_t>(size) * size, 0);
      for (int x = 0; x < size; ++x) {
        for (int y = 0; y < size; ++y) {
          if (x == y || y == 0 || x == n_cand + 1 || (x == 0 && y == n_cand + 1)) continue;
          double c = segment_cost(id_of(x), id_of(y)) + pitch_cost(id_of(y));
          cost[static_cast<size_t>(x) * size + y] = static_cast<int64_t>(std::llround(c * 1000.0));
        }
      }
      // Collision-aware costs, as for order:=tsp but per candidate: from each
      // node, the straight lines toward its cheapest candidates are checked
      // (at most 3 per waypoint, tsp_check_neighbours waypoints); a blocked
      // line gets the penalty in the matrix.
      auto waypoint_of = [&](int node_index) {
        const int id = id_of(node_index);
        return id >= 0 ? table.owner[id].first : id;
      };
      for (int x = 0; x <= n_cand && tsp_check_neighbours > 0; ++x) {
        std::vector<std::pair<int64_t, int>> near;
        for (int y = 1; y < size; ++y) {
          if (y == x || (x == 0 && y == n_cand + 1) || waypoint_of(y) == waypoint_of(x)) continue;
          near.push_back({cost[static_cast<size_t>(x) * size + y], y});
        }
        std::sort(near.begin(), near.end());
        std::map<int, int> per_waypoint;
        for (const auto & entry : near) {
          const int y = entry.second;
          int & used = per_waypoint[waypoint_of(y)];
          if (used >= 3) continue;
          if (used == 0 && static_cast<int>(per_waypoint.size()) > tsp_check_neighbours) break;
          ++used;
          const int a = id_of(x), b = id_of(y);
          if (measured.count({a, b}) || line_check(a, b)) continue;
          double c = segment_cost(a, b) + pitch_cost(b);
          cost[static_cast<size_t>(x) * size + y] = static_cast<int64_t>(std::llround(c * 1000.0));
        }
      }
      std::vector<std::vector<int>> clusters;
      for (int wp : reachable) {
        std::vector<int> cluster;
        for (size_t c = 0; c < cands.layers[wp].size(); ++c) cluster.push_back(table.first_id[wp] + static_cast<int>(c) + 1);
        clusters.push_back(cluster);
      }
      RouteResult r = solve_route(cost, size, 0, n_cand + 1, clusters, tsp_time_limit);
      solver_status = r.status;
      if (!r.ok) {
        std::printf("  OR-Tools non ha trovato un ordine (%s): uso l'ordine a settori.\n", r.status.c_str());
        order = reachable;
      } else {
        for (int k : r.nodes) order.push_back(table.owner[k - 1].first);
      }
    }
    order_s = seconds_since(t0);

    t0 = std::chrono::steady_clock::now();
    std::vector<int> chain;
    int dp_iterations = 0;
    run_dp(order, chain, dp_iterations);
    dp_s = seconds_since(t0);

    Tour tour;
    tour.ids = chain;
    tour.visit_order = order;
    for (size_t k = 1; k < tour.ids.size(); ++k) tour.estimated += estimate(tour.ids[k - 1], tour.ids[k]);
    std::printf("  giro %d: ordine %.2f s (%s), DP %.2f s (%d iterazioni)\n", round, order_s,
                solver_status.c_str(), dp_s, dp_iterations);
    return tour;
  };

  // --- 4. Plan every segment of the tour with MoveIt, measure ---
  auto measure_tour = [&](Tour & tour) {
    int at = START;
    std::vector<int> kept = {START};
    for (size_t k = 1; k < tour.ids.size(); ++k) {
      if (!rclcpp::ok()) return;
      const int next = tour.ids[k];
      const Segment & seg = plan_segment(at, next);
      if (seg.via_recovery) ++tour.unplannable;
      if (seg.via_recovery && seg.ok) ++tour.recoveries;
      if (!seg.ok) {
        if (next == HOME) {
          std::printf("  ritorno a home non pianificabile da %s\n", label_of(at).c_str());
          continue;
        }
        ++tour.dropped;   // nothing reaches this waypoint from here: skipped
        continue;
      }
      for (const Leg & leg : seg.legs) {
        tour.legs.push_back(leg);
        tour.measured_time += leg.duration;
        tour.joint_motion += leg.joint_motion;
        ++tour.planners[leg.planner];
      }
      if (next >= 0) ++tour.reached;
      kept.push_back(next);
      at = next;
    }
    tour.ids = kept;
  };

  // --- 5. Rounds: estimate -> plan -> measured costs -> again ---
  Tour best;
  bool have_best = false;
  std::set<std::vector<int>> seen_tours;
  int round = 0;
  double order_total_s = 0.0, dp_total_s = 0.0;
  std::string solver_status;
  for (; round <= refine_iterations && rclcpp::ok(); ++round) {
    double order_s = 0.0, dp_s = 0.0;
    Tour tour = build_tour(round, order_s, dp_s, solver_status);
    order_total_s += order_s;
    dp_total_s += dp_s;
    if (seen_tours.count(tour.ids)) {
      std::printf("  giro %d: stessa sequenza di un giro precedente, fine del raffinamento.\n", round);
      break;
    }
    seen_tours.insert(tour.ids);
    const std::vector<int> planned_ids = tour.ids;
    measure_tour(tour);

    // How well the estimates describe the planned motions.
    int off = 0;
    double abs_error = 0.0;
    for (size_t k = 1; k < planned_ids.size(); ++k) {
      auto it = planned.find({planned_ids[k - 1], planned_ids[k]});
      if (it == planned.end() || !it->second.ok) continue;
      double e = estimate(planned_ids[k - 1], planned_ids[k]);
      double m = it->second.duration();
      abs_error += std::abs(m - e);
      if (std::abs(m - e) > std::max(0.5, 0.3 * e)) ++off;
      measured[{planned_ids[k - 1], planned_ids[k]}] = m;
    }
    for (const auto & kv : planned) {
      if (!kv.second.ok) measured[kv.first] = 1000.0;   // not plannable: avoid
    }
    std::printf("  giro %d: stima %.1f s, pianificato %.1f s (errore medio %.2f s/tratto, %d tratti lontani "
                "dalla stima) | %d/%zu waypoint, %d tratti non pianificabili, movimento giunti %.1f rad\n",
                round, tour.estimated, tour.measured_time,
                planned_ids.size() > 1 ? abs_error / (planned_ids.size() - 1) : 0.0, off,
                tour.reached, waypoints.size(), tour.unplannable, tour.joint_motion);
    std::fflush(stdout);

    if (!have_best || tour.reached > best.reached ||
        (tour.reached == best.reached && tour.measured_time < best.measured_time)) {
      best = tour;
      have_best = true;
    }
  }
  const double plan_total_s = seconds_since(plan_t0);
  if (!have_best || !rclcpp::ok()) return shutdown(1);

  int ompl_segments = 0;
  for (const auto & kv : best.planners) {
    if (kv.first.rfind("ompl", 0) == 0) ompl_segments += kv.second;
  }
  std::printf("\n=== Piano (ordine %s, %d giri) ===\n", order_method.c_str(), round);
  std::printf("Tempo di pianificazione: %.1f s (candidati %.1f s, ordine %.1f s, DP %.1f s, MoveIt %.1f s)\n",
              plan_total_s, cands.seconds, order_total_s, dp_total_s, moveit_seconds);
  std::printf("Tempo della traiettoria: %.1f s (ritorno a %s incluso) | stima PTP %.1f s\n",
              best.measured_time, home_pose_name.c_str(), best.estimated);
  std::printf("Movimento dei giunti: %.1f rad | waypoint %d/%zu (%d vista coperta, %d irraggiungibili, "
              "%d saltati) | tratti non pianificabili direttamente: %d | tratti OMPL: %d\n",
              best.joint_motion, best.reached, waypoints.size(), view_blocked, unreachable, best.dropped,
              best.unplannable, ompl_segments);
  std::printf("Planner:");
  for (const auto & kv : best.planners) std::printf("  %s %d", kv.first.c_str(), kv.second);
  std::printf("\nOR-Tools: %s (soluzione euristica, nessuna prova di ottimalita').\n", solver_status.c_str());

  // --- Save ---
  ScanPlan out;
  char created[32];
  std::time_t now = std::time(nullptr);
  std::strftime(created, sizeof(created), "%Y-%m-%d %H:%M:%S", std::localtime(&now));
  out.recording.created           = created;
  out.recording.planning_group    = planning_group;
  out.recording.end_effector_link = ee_link;
  out.recording.global_frame      = global_frame;
  out.recording.center            = {cfg.center.x(), cfg.center.y(), cfg.center.z()};
  out.recording.radius            = cfg.radius;
  out.recording.waypoints_total   = static_cast<int>(waypoints.size());
  out.recording.waypoints_reached = best.reached;
  out.recording.recoveries        = best.recoveries;
  for (const Leg & leg : best.legs) {
    RecordedSegment seg;
    seg.label = leg.label;
    seg.waypoint = leg.waypoint;
    seg.joint_names = leg.trajectory.joint_trajectory.joint_names;
    for (const auto & pt : leg.trajectory.joint_trajectory.points) {
      RecordedPoint p;
      p.time          = rclcpp::Duration(pt.time_from_start).seconds();
      p.positions     = pt.positions;
      p.velocities    = pt.velocities;
      p.accelerations = pt.accelerations;
      seg.points.push_back(p);
    }
    if (leg.waypoint >= 0) {
      Eigen::Isometry3d camera;
      camera_pose_of(seg, camera);
      const Eigen::Quaterniond q(camera.linear());
      seg.camera_pose = {camera.translation().x(), camera.translation().y(), camera.translation().z(),
                         q.x(), q.y(), q.z(), q.w()};
    }
    out.recording.segments.push_back(seg);
  }
  PlanData & d = out.data;
  d.method = order_method;
  d.joint_names = joint_names;
  d.start_joints = current_joints;
  for (size_t k = 1; k < best.ids.size(); ++k) {
    if (best.ids[k] >= 0) d.visit_order.push_back(table.owner[best.ids[k]].first);
  }
  d.waypoints = waypoint_values;
  d.waypoint_params = waypoint_params;
  d.scaling = scaling;
  d.max_velocity = max_velocity;
  d.max_acceleration = max_acceleration;
  if (scene) d.scene = scene_boxes(*scene);
  d.metrics = {
    {"planning_s", plan_total_s},
    {"trajectory_s", best.measured_time},
    {"estimated_s", best.estimated},
    {"joint_motion_rad", best.joint_motion},
    {"waypoints_reached", static_cast<double>(best.reached)},
    {"segments_not_plannable", static_cast<double>(best.unplannable)},
    {"ompl_segments", static_cast<double>(ompl_segments)},
    {"rounds", static_cast<double>(round)},
  };
  std::error_code dir_error;
  std::filesystem::path dir = std::filesystem::path(plan_path).parent_path();
  if (!dir.empty()) std::filesystem::create_directories(dir, dir_error);
  std::string error;
  if (save_plan(plan_path, out, error)) {
    std::printf("Piano salvato: %s (%zu movimenti). Eseguire con mode:=execute.\n",
                plan_path.c_str(), out.recording.segments.size());
  } else {
    std::printf("Piano NON salvato: %s\n", error.c_str());
  }
  std::fflush(stdout);
  return shutdown(0);
}
