// scan_replay_node: plays back a scan recorded by scan_sequence_node
// (record:=true). No IK, no sequence planning, no motion planning: the
// recorded trajectories are executed one after the other, so every work
// session moves the robot the same way and with the same timing.
//
// Before moving, the recording is checked against the current cell:
//   1) same scan center and radius as automata_config.yaml;
//   2) the current robot model puts the camera where the recording says, at
//      the end of every waypoint motion (catches a changed TCP / end effector);
//   3) no recorded point collides with the current planning scene.
// Then the robot goes to the first recorded point with a planned motion, and
// the recorded motions follow. It starts paused, like scan_sequence_node:
//   ros2 service call /scan_replay_node/start std_srvs/srv/Trigger {}
//   ros2 service call /scan_replay_node/pause std_srvs/srv/Trigger {}

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <map>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "ur_automata_scan/frustum_markers.hpp"
#include "ur_automata_scan/planning_scene_client.hpp"
#include "ur_automata_scan/scan_recording.hpp"

static std::atomic<bool> g_paused{true};   // start paused, wait for /start

// Recorded motion -> trajectory message for MoveGroupInterface::execute().
static moveit_msgs::msg::RobotTrajectory to_trajectory(const RecordedSegment & seg)
{
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
  return traj;
}

// Waits while the replay is paused. Returns false if ROS is shutting down.
static bool wait_while_paused()
{
  bool announced = false;
  while (g_paused.load() && rclcpp::ok()) {
    if (!announced) {
      std::printf("In pausa: ros2 service call /scan_replay_node/start std_srvs/srv/Trigger {}\n");
      std::fflush(stdout);
      announced = true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return rclcpp::ok();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node   = rclcpp::Node::make_shared("scan_replay_node");
  auto logger = node->get_logger();
  rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_WARN);

  using TriggerSrv = std_srvs::srv::Trigger;
  auto start_srv = node->create_service<TriggerSrv>(
    "~/start",
    [](const std::shared_ptr<TriggerSrv::Request>, std::shared_ptr<TriggerSrv::Response> res) {
      g_paused = false;
      res->success = true;
      res->message = "replay started/resumed";
    });
  auto pause_srv = node->create_service<TriggerSrv>(
    "~/pause",
    [](const std::shared_ptr<TriggerSrv::Request>, std::shared_ptr<TriggerSrv::Response> res) {
      g_paused = true;
      res->success = true;
      res->message = "replay paused (stops after the current motion)";
    });
  // Created here, well before the first publication, so RViz has time to connect.
  auto markers_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/scan_waypoints_markers", 10);

  // --- Parameters (same names as scan_sequence_node) ---
  node->declare_parameter<std::string>("recording_file",            "");
  node->declare_parameter<std::string>("global_frame",              "world");
  node->declare_parameter<std::string>("planning_group",            "ur_manipulator");
  node->declare_parameter<std::string>("end_effector_link",         "ee_automata_tcp");
  node->declare_parameter<double>     ("trajectory_scaling_factor", 0.1);
  node->declare_parameter<std::vector<double>>("scan_center",       {0.0, 0.4, 0.5});
  node->declare_parameter<double>     ("scan_radius",               0.35);
  node->declare_parameter<double>     ("scan_planning_time",        5.0);
  // Speed of the replay relative to the recording: 1.0 = as recorded,
  // 0.5 = half speed. Only slower: values above 1.0 are refused.
  node->declare_parameter<double>     ("speed",                     1.0);

  const std::string recording_file = node->get_parameter("recording_file").as_string();
  const std::string global_frame   = node->get_parameter("global_frame").as_string();
  const std::string planning_group = node->get_parameter("planning_group").as_string();
  const std::string ee_link        = node->get_parameter("end_effector_link").as_string();
  const double      scaling        = node->get_parameter("trajectory_scaling_factor").as_double();
  const auto        center         = node->get_parameter("scan_center").as_double_array();
  const double      radius         = node->get_parameter("scan_radius").as_double();
  const double      planning_time  = node->get_parameter("scan_planning_time").as_double();
  const double      speed          = node->get_parameter("speed").as_double();
  if (speed <= 0.0 || speed > 1.0) {
    std::fprintf(stderr, "speed deve essere maggiore di 0 e al massimo 1.0 (ricevuto %.3f).\n", speed);
    rclcpp::shutdown();
    return 1;
  }

  // --- Load the recording ---
  ScanRecording rec;
  std::string error;
  if (!load_recording(recording_file, rec, error)) {
    std::fprintf(stderr, "Registrazione non leggibile: %s\n", error.c_str());
    rclcpp::shutdown();
    return 1;
  }
  std::printf("Registrazione: %s (taratura del %s)\n", recording_file.c_str(), rec.created.c_str());
  std::printf("  %zu movimenti, %d/%d waypoint, %d recovery\n",
              rec.segments.size(), rec.waypoints_reached, rec.waypoints_total, rec.recoveries);

  // --- Check 1: same scan geometry and same robot setup ---
  bool same_setup = rec.planning_group == planning_group && rec.end_effector_link == ee_link &&
                    rec.center.size() == 3 && center.size() == 3 &&
                    std::abs(rec.radius - radius) < 1e-3;
  for (size_t k = 0; same_setup && k < 3; ++k) {
    if (std::abs(rec.center[k] - center[k]) > 1e-3) same_setup = false;
  }
  if (!same_setup) {
    std::fprintf(stderr,
      "La registrazione non corrisponde alla configurazione attuale (gruppo, end effector, "
      "centro o raggio della sfera). Rifare la taratura con record:=true.\n");
    rclcpp::shutdown();
    return 1;
  }

  // --- MoveIt ---
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  moveit::planning_interface::MoveGroupInterface move_group(node, planning_group);
  move_group.setEndEffectorLink(ee_link);
  // Only for the planned motion to the first recorded point, slowed like the rest.
  move_group.setMaxVelocityScalingFactor(scaling * speed);
  move_group.setMaxAccelerationScalingFactor(scaling * speed);
  move_group.setPlanningTime(planning_time);

  planning_scene::PlanningScenePtr scene =
    fetch_planning_scene(node, move_group.getRobotModel(), logger);
  moveit::core::RobotState state(*move_group.getCurrentState());

  // --- Check 2: the current model puts the camera where it was recorded ---
  int pose_mismatches = 0;
  for (const RecordedSegment & seg : rec.segments) {
    for (const std::string & name : seg.joint_names) {
      if (!move_group.getRobotModel()->hasJointModel(name)) {
        std::fprintf(stderr, "Giunto '%s' della registrazione assente nel modello.\n", name.c_str());
        rclcpp::shutdown();
        spinner.join();
        return 1;
      }
    }
    if (seg.camera_pose.size() != 7) continue;
    state.setVariablePositions(seg.joint_names, seg.points.back().positions);
    state.update();
    const Eigen::Isometry3d camera = state.getGlobalLinkTransform(ee_link);
    const Eigen::Vector3d recorded_pos(seg.camera_pose[0], seg.camera_pose[1], seg.camera_pose[2]);
    const Eigen::Quaterniond recorded_q(seg.camera_pose[6], seg.camera_pose[3],
                                        seg.camera_pose[4], seg.camera_pose[5]);
    const double position_error = (camera.translation() - recorded_pos).norm();
    const double angle_error = Eigen::Quaterniond(camera.linear()).angularDistance(recorded_q);
    if (position_error > 0.002 || angle_error > 1.0 * M_PI / 180.0) {
      if (pose_mismatches < 5) {
        std::fprintf(stderr, "  %s: camera a %.1f mm / %.1f gradi dalla posa registrata\n",
                     seg.label.c_str(), position_error * 1000.0, angle_error * 180.0 / M_PI);
      }
      ++pose_mismatches;
    }
  }
  if (pose_mismatches > 0) {
    std::fprintf(stderr,
      "%d waypoint non tornano con il modello attuale del robot (TCP, end effector o base "
      "cambiati?). Rifare la taratura con record:=true.\n", pose_mismatches);
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  // --- Check 3: no recorded point collides with the current scene ---
  if (!scene) {
    std::fprintf(stderr, "Planning scene non disponibile: impossibile verificare le collisioni.\n");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }
  double total_motion_s = 0.0;
  for (const RecordedSegment & seg : rec.segments) {
    for (const RecordedPoint & p : seg.points) {
      state.setVariablePositions(seg.joint_names, p.positions);
      state.update();
      if (scene->isStateColliding(state, planning_group)) {
        std::fprintf(stderr,
          "Il movimento '%s' (t = %.2f s) collide con la scena attuale: qualcosa e' cambiato "
          "nella cella. Rifare la taratura con record:=true.\n", seg.label.c_str(), p.time);
        rclcpp::shutdown();
        spinner.join();
        return 1;
      }
    }
    total_motion_s += seg.points.back().time / speed;
  }
  std::printf("Controlli superati: modello e scena coerenti con la registrazione. "
              "Durata dei movimenti registrati: %.1f s (speed %.2f)\n", total_motion_s, speed);

  // --- Gray camera frustums on the recorded waypoints ---
  visualization_msgs::msg::MarkerArray markers = make_frustum_markers(rec, global_frame);
  publish_first_frustums(markers_pub, markers);

  // --- Go to the first recorded point with a planned motion ---
  if (!wait_while_paused()) {
    spinner.join();
    return 0;
  }
  const RecordedSegment & first = rec.segments.front();
  std::map<std::string, double> start_target;
  for (size_t k = 0; k < first.joint_names.size(); ++k) {
    start_target[first.joint_names[k]] = first.points.front().positions[k];
  }
  move_group.setJointValueTarget(start_target);
  moveit::planning_interface::MoveGroupInterface::Plan start_plan;
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
    std::fprintf(stderr, "Impossibile portare il robot nella posa di partenza della registrazione.\n");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  // --- Replay ---
  auto t0 = std::chrono::steady_clock::now();
  int waypoints_done = 0;
  bool completed = true;
  for (size_t k = 0; k < rec.segments.size(); ++k) {
    if (!wait_while_paused()) {
      completed = false;
      break;
    }
    const RecordedSegment & seg = rec.segments[k];
    // MoveIt refuses the motion if the robot is not where it starts.
    if (move_group.execute(to_trajectory(scale_segment_speed(seg, speed))) != moveit::core::MoveItErrorCode::SUCCESS) {
      std::fprintf(stderr, "Movimento %zu/%zu ('%s') non eseguito: replay interrotto.\n",
                   k + 1, rec.segments.size(), seg.label.c_str());
      completed = false;
      break;
    }
    double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    if (seg.waypoint >= 0) {
      ++waypoints_done;
      // The camera is on waypoint seg.waypoint: this is where the photo will be taken.
      set_frustum_reached(markers, k);
      markers_pub->publish(markers);
    }
    std::printf("  %3zu/%zu  %-10s  (%.1f s)\n", k + 1, rec.segments.size(), seg.label.c_str(), elapsed);
    std::fflush(stdout);
  }

  double total_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  std::printf("\nReplay %s: %d waypoint in %.1f s (movimenti registrati: %.1f s)\n",
              completed ? "completato" : "INTERROTTO", waypoints_done, total_s, total_motion_s);
  std::fflush(stdout);

  rclcpp::shutdown();
  spinner.join();
  return completed ? 0 : 1;
}
