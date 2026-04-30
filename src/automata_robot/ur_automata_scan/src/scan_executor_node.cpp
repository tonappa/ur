#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <termios.h>
#include <unistd.h>

#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>

#include "ur_automata_scan/sphere_waypoint_generator.hpp"

using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

// ============================================================================
// Marker colors
// ============================================================================
struct Color { float r, g, b; };

const Color COLOR_GRAY   = {0.5f, 0.5f, 0.5f};  // not visited yet
const Color COLOR_YELLOW = {1.0f, 0.85f, 0.0f}; // currently being planned/executed
const Color COLOR_GREEN  = {0.0f, 1.0f, 0.0f};  // reached successfully
const Color COLOR_RED    = {1.0f, 0.0f, 0.0f};  // failed

// ============================================================================
// ANSI escape codes for the in-place table
// ============================================================================
namespace ansi {
  constexpr const char * CLEAR_SCREEN = "\033[2J";
  constexpr const char * CURSOR_HOME  = "\033[H";
  constexpr const char * HIDE_CURSOR  = "\033[?25l";
  constexpr const char * SHOW_CURSOR  = "\033[?25h";
  constexpr const char * RESET        = "\033[0m";
  constexpr const char * BOLD         = "\033[1m";
  constexpr const char * GRAY         = "\033[90m";
  constexpr const char * YELLOW       = "\033[33m";
  constexpr const char * GREEN        = "\033[32m";
  constexpr const char * RED          = "\033[31m";
  constexpr const char * CYAN         = "\033[36m";
}

// ============================================================================
// Terminal raw-mode + key reader (spacebar = pause/resume, Q = quit)
// ============================================================================
static std::atomic<bool> g_paused{true};   // start paused, wait for first SPACE
static std::atomic<bool> g_quit{false};
static std::atomic<bool> g_redraw_request{true};
static bool g_termios_saved = false;
static struct termios g_old_termios;
static bool g_stdin_is_tty = false;

static void restore_terminal()
{
  if (g_termios_saved) {
    tcsetattr(STDIN_FILENO, TCSANOW, &g_old_termios);
    g_termios_saved = false;
  }
  // Make sure cursor is visible again
  std::printf("%s", ansi::SHOW_CURSOR);
  std::fflush(stdout);
}

static void setup_raw_terminal()
{
  if (!isatty(STDIN_FILENO)) {
    // No interactive terminal: spacebar control disabled, run unattended.
    g_stdin_is_tty = false;
    g_paused = false;  // auto-start when no TTY (e.g. piped / non-interactive)
    return;
  }
  g_stdin_is_tty = true;

  if (tcgetattr(STDIN_FILENO, &g_old_termios) != 0) {
    g_stdin_is_tty = false;
    g_paused = false;
    return;
  }
  g_termios_saved = true;
  std::atexit(restore_terminal);

  struct termios raw = g_old_termios;
  raw.c_lflag &= ~(ECHO | ICANON);  // disable echo + line buffering
  raw.c_cc[VMIN]  = 1;
  raw.c_cc[VTIME] = 0;
  tcsetattr(STDIN_FILENO, TCSANOW, &raw);
}

static void key_reader_loop()
{
  if (!g_stdin_is_tty) return;
  char c;
  while (!g_quit.load() && rclcpp::ok()) {
    ssize_t n = ::read(STDIN_FILENO, &c, 1);
    if (n != 1) continue;
    if (c == ' ') {
      g_paused = !g_paused.load();
      g_redraw_request = true;
    } else if (c == 'q' || c == 'Q' || c == 0x03 /* Ctrl+C */) {
      g_quit = true;
      g_redraw_request = true;
      rclcpp::shutdown();
      break;
    }
  }
}

// ============================================================================
// Per-waypoint table state
// ============================================================================
enum class WpStatus {
  PENDING,
  RUNNING,
  DONE,
  HOMING,
  FAIL
};

struct WpRow {
  geometry_msgs::msg::Pose target;
  WpStatus status = WpStatus::PENDING;
  bool actual_set = false;            // true if actual_pose has been filled
  geometry_msgs::msg::Pose actual;    // pose actually reached (may differ when lock_roll=false)
  bool reoriented = false;            // true if actual orientation differs from target
};

// Convert quaternion (geometry_msgs) to RPY in degrees, XYZ-Euler convention
static Eigen::Vector3d quat_to_rpy_deg(const geometry_msgs::msg::Quaternion & q)
{
  Eigen::Quaterniond eq(q.w, q.x, q.y, q.z);
  // eulerAngles(0,1,2) returns intrinsic XYZ (roll-pitch-yaw) in radians
  Eigen::Vector3d rpy = eq.toRotationMatrix().eulerAngles(0, 1, 2);
  return rpy * (180.0 / M_PI);
}

// Smallest angle (radians) between two unit quaternions
static double quat_angle_diff(
  const geometry_msgs::msg::Quaternion & a,
  const geometry_msgs::msg::Quaternion & b)
{
  Eigen::Quaterniond qa(a.w, a.x, a.y, a.z);
  Eigen::Quaterniond qb(b.w, b.x, b.y, b.z);
  double d = std::abs(qa.dot(qb));
  if (d > 1.0) d = 1.0;
  return 2.0 * std::acos(d);
}

static const char * status_label(WpStatus s)
{
  switch (s) {
    case WpStatus::PENDING: return "PENDING";
    case WpStatus::RUNNING: return "RUNNING";
    case WpStatus::DONE:    return "DONE";
    case WpStatus::HOMING:  return "HOMING";
    case WpStatus::FAIL:    return "FAIL";
  }
  return "?";
}

static const char * status_color(WpStatus s)
{
  switch (s) {
    case WpStatus::PENDING: return ansi::GRAY;
    case WpStatus::RUNNING: return ansi::YELLOW;
    case WpStatus::DONE:    return ansi::GREEN;
    case WpStatus::HOMING:  return ansi::YELLOW;
    case WpStatus::FAIL:    return ansi::RED;
  }
  return ansi::RESET;
}

// ============================================================================
// Render the full table in place
// ============================================================================
static std::mutex g_render_mutex;

static void render_table(const std::vector<WpRow> & rows, bool lock_roll,
                         const std::string & planner_label)
{
  std::lock_guard<std::mutex> lock(g_render_mutex);

  std::printf("%s%s", ansi::CURSOR_HOME, ansi::CLEAR_SCREEN);

  // Header banner
  const char * mode_str = g_paused.load() ? "PAUSED " : "RUNNING";
  const char * mode_col = g_paused.load() ? ansi::YELLOW : ansi::GREEN;
  std::printf("%s%sUR_AUTOMATA SCAN  [%s%s%s]%s   %sSPACE%s = pause/resume   %sQ%s = quit\n",
    ansi::BOLD, ansi::CYAN,
    mode_col, mode_str, ansi::CYAN,
    ansi::RESET,
    ansi::BOLD, ansi::RESET,
    ansi::BOLD, ansi::RESET);
  std::printf("Planner: %s%s%s   Roll mode: %s%s%s\n",
    ansi::BOLD, planner_label.c_str(), ansi::RESET,
    ansi::BOLD,
    lock_roll ? "LOCKED  (X horizontal)" : "FREE    (MoveIt picks roll around Y_TCP)",
    ansi::RESET);
  std::printf("\n");

  // Column header
  std::printf("%s%4s | %-32s | %-22s | %s%s\n",
    ansi::BOLD, "#", "TARGET (x  y  z  /  R  P  Y°)", "STATUS", "ACTUAL (R  P  Y° if changed)",
    ansi::RESET);
  std::printf("-----+----------------------------------+------------------------+-----------------------------\n");

  for (size_t i = 0; i < rows.size(); ++i) {
    const auto & r = rows[i];
    Eigen::Vector3d rpy = quat_to_rpy_deg(r.target.orientation);

    char target_buf[80];
    std::snprintf(target_buf, sizeof(target_buf),
      "%6.3f %6.3f %6.3f / %6.1f %6.1f %6.1f",
      r.target.position.x, r.target.position.y, r.target.position.z,
      rpy.x(), rpy.y(), rpy.z());

    char status_buf[64];
    std::snprintf(status_buf, sizeof(status_buf), "%s", status_label(r.status));

    char actual_buf[80] = "";
    if (r.actual_set && r.reoriented) {
      Eigen::Vector3d arpy = quat_to_rpy_deg(r.actual.orientation);
      std::snprintf(actual_buf, sizeof(actual_buf),
        "R P Y = %6.1f %6.1f %6.1f", arpy.x(), arpy.y(), arpy.z());
    }

    std::printf(" %3zu | %-32s | %s%-22s%s | %s\n",
      i, target_buf,
      status_color(r.status), status_buf, ansi::RESET,
      actual_buf);
  }

  std::printf("\n");
  std::fflush(stdout);

  g_redraw_request = false;
}

// ============================================================================
// Marker helpers (unchanged behaviour)
// ============================================================================
static Marker make_sphere_marker(int id, const geometry_msgs::msg::Pose & pose,
                                 const std::string & frame, const Color & c)
{
  Marker m;
  m.header.frame_id = frame;
  m.ns     = "scan_waypoints";
  m.id     = id;
  m.type   = Marker::SPHERE;
  m.action = Marker::ADD;
  m.pose   = pose;
  m.scale.x = m.scale.y = m.scale.z = 0.025;
  m.color.r = c.r;
  m.color.g = c.g;
  m.color.b = c.b;
  m.color.a = 0.85f;
  return m;
}

static Marker make_arrow_marker(int id, const geometry_msgs::msg::Pose & pose,
                                const Eigen::Vector3d & center,
                                const std::string & frame, const Color & c)
{
  Marker m;
  m.header.frame_id = frame;
  m.ns     = "scan_orientations";
  m.id     = id;
  m.type   = Marker::ARROW;
  m.action = Marker::ADD;

  Eigen::Vector3d wp_pos(pose.position.x, pose.position.y, pose.position.z);
  Eigen::Vector3d direction = (center - wp_pos).normalized();
  double arrow_length = 0.06;
  Eigen::Vector3d arrow_tip = wp_pos + arrow_length * direction;

  geometry_msgs::msg::Point start, end;
  start.x = wp_pos.x();    start.y = wp_pos.y();    start.z = wp_pos.z();
  end.x   = arrow_tip.x(); end.y   = arrow_tip.y(); end.z   = arrow_tip.z();
  m.points.push_back(start);
  m.points.push_back(end);

  m.scale.x = 0.005;
  m.scale.y = 0.012;
  m.scale.z = 0.015;

  m.color.r = c.r;
  m.color.g = c.g;
  m.color.b = c.b;
  m.color.a = 0.9f;
  return m;
}

static void set_marker_color(MarkerArray & sphere_array, MarkerArray & arrow_array,
                             size_t i, const Color & c)
{
  sphere_array.markers[i].color.r = c.r;
  sphere_array.markers[i].color.g = c.g;
  sphere_array.markers[i].color.b = c.b;
  arrow_array.markers[i].color.r  = c.r;
  arrow_array.markers[i].color.g  = c.g;
  arrow_array.markers[i].color.b  = c.b;
}

static void publish_markers(
  rclcpp::Publisher<MarkerArray>::SharedPtr sphere_pub,
  rclcpp::Publisher<MarkerArray>::SharedPtr arrow_pub,
  const MarkerArray & sphere_array,
  const MarkerArray & arrow_array)
{
  sphere_pub->publish(sphere_array);
  arrow_pub->publish(arrow_array);
}

static bool go_home(moveit::planning_interface::MoveGroupInterface & move_group,
                    const std::string & home_pose_name,
                    const rclcpp::Logger & logger)
{
  RCLCPP_WARN(logger, "Recovery: moving back to '%s' ...", home_pose_name.c_str());

  // Drop any active path constraint (e.g. the look-at orientation constraint
  // used in free-roll mode) — the home pose cannot satisfy it.
  move_group.clearPathConstraints();
  move_group.setNamedTarget(home_pose_name);

  moveit::planning_interface::MoveGroupInterface::Plan home_plan;
  if (move_group.plan(home_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Recovery failed: could not plan to '%s'.", home_pose_name.c_str());
    return false;
  }

  if (move_group.execute(home_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Recovery failed: could not execute move to '%s'.", home_pose_name.c_str());
    return false;
  }

  RCLCPP_INFO(logger, "Recovery done: now at '%s'.", home_pose_name.c_str());
  return true;
}

// Block until SPACE is pressed (or quit). Redraws the table on every toggle.
static void wait_while_paused(const std::vector<WpRow> & rows, bool lock_roll,
                              const std::string & planner_label)
{
  while (g_paused.load() && !g_quit.load() && rclcpp::ok()) {
    if (g_redraw_request.load()) render_table(rows, lock_roll, planner_label);
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
  }
  if (!g_quit.load()) {
    g_redraw_request = true;
    render_table(rows, lock_roll, planner_label);
  }
}

// ============================================================================
// MAIN
// ============================================================================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node   = rclcpp::Node::make_shared("scan_executor_node");
  auto logger = node->get_logger();

  // Quiet down ROS/MoveIt info-spam so the table layout stays readable.
  // WARN/ERROR/FATAL still pass through.
  rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_WARN);

  // --- Parameters ---
  node->declare_parameter<std::string>("global_frame",              "world");
  node->declare_parameter<std::string>("planning_group",            "ur_manipulator");
  node->declare_parameter<std::string>("end_effector_link",         "ee_automata_tcp");
  node->declare_parameter<std::string>("home_pose_name",            "home");
  node->declare_parameter<double>     ("trajectory_scaling_factor", 0.1);
  node->declare_parameter<std::vector<double>>("scan_center",       {0.0, 0.4, 0.5});
  node->declare_parameter<double>     ("scan_radius",               0.35);
  node->declare_parameter<std::string>("scan_hemisphere",           "upper");
  node->declare_parameter<std::string>("scan_direction",            "latitudinal");
  node->declare_parameter<int>        ("scan_num_rings",            4);   // latitudinal
  node->declare_parameter<int>        ("scan_points_per_ring",      8);   // latitudinal
  node->declare_parameter<int>        ("scan_num_arc",              6);   // longitudinal
  node->declare_parameter<int>        ("scan_points_per_arc",       5);   // longitudinal
  node->declare_parameter<double>     ("scan_equator_exclusion_deg",10.0);
  node->declare_parameter<bool>       ("scan_lock_roll",             true);
  node->declare_parameter<std::string>("scan_planner",                "ompl");

  const std::string global_frame    = node->get_parameter("global_frame").as_string();
  const std::string planning_group  = node->get_parameter("planning_group").as_string();
  const std::string ee_link         = node->get_parameter("end_effector_link").as_string();
  const std::string home_pose_name  = node->get_parameter("home_pose_name").as_string();
  const double      scaling         = node->get_parameter("trajectory_scaling_factor").as_double();
  const auto        center_vec      = node->get_parameter("scan_center").as_double_array();
  const double      radius          = node->get_parameter("scan_radius").as_double();
  const std::string hemi_str        = node->get_parameter("scan_hemisphere").as_string();
  const std::string dir_str         = node->get_parameter("scan_direction").as_string();
  const int         num_rings       = node->get_parameter("scan_num_rings").as_int();
  const int         points_per_ring = node->get_parameter("scan_points_per_ring").as_int();
  const int         num_arc         = node->get_parameter("scan_num_arc").as_int();
  const int         points_per_arc  = node->get_parameter("scan_points_per_arc").as_int();
  const double      exclusion_deg   = node->get_parameter("scan_equator_exclusion_deg").as_double();
  const bool        lock_roll       = node->get_parameter("scan_lock_roll").as_bool();
  const std::string planner_str     = node->get_parameter("scan_planner").as_string();

  if (center_vec.size() != 3) {
    std::fprintf(stderr, "scan_center must have exactly 3 values (x, y, z).\n");
    rclcpp::shutdown();
    return 1;
  }

  // --- Build the scan configuration struct ---
  ScanConfig cfg;
  cfg.center = Eigen::Vector3d(center_vec[0], center_vec[1], center_vec[2]);
  cfg.radius = radius;
  cfg.num_rings       = num_rings;
  cfg.points_per_ring = points_per_ring;
  cfg.num_arc         = num_arc;
  cfg.points_per_arc  = points_per_arc;
  cfg.equator_exclusion_rad = exclusion_deg * M_PI / 180.0;

  if (hemi_str == "upper") {
    cfg.hemisphere = HEMISPHERE_UPPER;
  } else if (hemi_str == "lower") {
    cfg.hemisphere = HEMISPHERE_LOWER;
  } else if (hemi_str == "full") {
    cfg.hemisphere = HEMISPHERE_FULL;
  } else {
    std::fprintf(stderr, "scan_hemisphere is invalid: '%s'. Use: upper | lower | full\n", hemi_str.c_str());
    rclcpp::shutdown();
    return 1;
  }

  if (dir_str == "latitudinal") {
    cfg.direction = SCAN_LATITUDINAL;
  } else if (dir_str == "longitudinal") {
    cfg.direction = SCAN_LONGITUDINAL;
  } else {
    std::fprintf(stderr, "scan_direction is invalid: '%s'. Use: latitudinal | longitudinal\n", dir_str.c_str());
    rclcpp::shutdown();
    return 1;
  }

  // --- Generate waypoints ---
  std::vector<geometry_msgs::msg::Pose> waypoints = generate_waypoints(cfg);

  // Build the table state (one row per waypoint, all PENDING)
  std::vector<WpRow> rows;
  rows.reserve(waypoints.size());
  for (const auto & p : waypoints) {
    WpRow r;
    r.target = p;
    rows.push_back(r);
  }

  // --- Background spinner (MoveGroupInterface needs the node spinning) ---
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  // --- Marker publishers + initial gray markers ---
  auto sphere_pub = node->create_publisher<MarkerArray>("/scan_waypoints_markers",      10);
  auto arrow_pub  = node->create_publisher<MarkerArray>("/scan_waypoints_orientations", 10);

  MarkerArray sphere_array, arrow_array;
  for (size_t i = 0; i < waypoints.size(); ++i) {
    sphere_array.markers.push_back(make_sphere_marker(static_cast<int>(i), waypoints[i], global_frame, COLOR_GRAY));
    arrow_array.markers.push_back( make_arrow_marker( static_cast<int>(i), waypoints[i], cfg.center, global_frame, COLOR_GRAY));
  }
  for (int attempt = 0; attempt < 5; ++attempt) {
    publish_markers(sphere_pub, arrow_pub, sphere_array, arrow_array);
    rclcpp::sleep_for(std::chrono::milliseconds(200));
  }

  // --- MoveIt setup ---
  moveit::planning_interface::MoveGroupInterface move_group(node, planning_group);
  move_group.setEndEffectorLink(ee_link);
  move_group.setPoseReferenceFrame(global_frame);
  move_group.setMaxVelocityScalingFactor(scaling);
  move_group.setMaxAccelerationScalingFactor(scaling);
  move_group.setPlanningTime(5.0);

  // Resolve `scan_planner` string to (pipeline_id, planner_id).
  // Default fallback is OMPL/RRTConnect if the string is unknown.
  std::string pipeline_id = "ompl";
  std::string planner_id  = "RRTConnectkConfigDefault";
  if (planner_str == "ompl") {
    // defaults above
  } else if (planner_str == "pilz_ptp") {
    pipeline_id = "pilz_industrial_motion_planner";
    planner_id  = "PTP";
  } else if (planner_str == "pilz_lin") {
    pipeline_id = "pilz_industrial_motion_planner";
    planner_id  = "LIN";
  } else {
    std::fprintf(stderr,
      "WARNING: scan_planner='%s' unknown, falling back to ompl/RRTConnect.\n",
      planner_str.c_str());
  }
  move_group.setPlanningPipelineId(pipeline_id);
  move_group.setPlannerId(planner_id);
  const std::string planner_label = pipeline_id + " / " + planner_id;

  // Pilz pipelines do not honour setPathConstraints — the look-at constraint
  // used in free-roll mode would be silently ignored. Warn the user so they
  // don't chase a phantom bug.
  if (!lock_roll && pipeline_id == "pilz_industrial_motion_planner") {
    std::fprintf(stderr,
      "WARNING: planner='%s' does not support orientation path constraints; "
      "lock_roll=false will behave as a plain position target (no roll lock).\n",
      planner_str.c_str());
  }

  // --- Interactive terminal: spacebar pause/resume + Q to quit ---
  setup_raw_terminal();
  std::printf("%s", ansi::HIDE_CURSOR);
  std::thread key_thread(key_reader_loop);

  // First render — node is paused, waiting for SPACE
  render_table(rows, lock_roll, planner_label);

  // Wait for the user's first SPACE before doing anything
  wait_while_paused(rows, lock_roll, planner_label);

  // --- Visit each waypoint ---
  int successes = 0;
  int failures  = 0;

  for (size_t i = 0; i < waypoints.size(); ++i) {
    if (g_quit.load() || !rclcpp::ok()) break;

    // Pause check: if paused mid-scan, finish the current iteration loop here
    wait_while_paused(rows, lock_roll, planner_label);
    if (g_quit.load() || !rclcpp::ok()) break;

    rows[i].status = WpStatus::RUNNING;
    set_marker_color(sphere_array, arrow_array, i, COLOR_YELLOW);
    publish_markers(sphere_pub, arrow_pub, sphere_array, arrow_array);
    g_redraw_request = true;
    render_table(rows, lock_roll, planner_label);

    // Always start each iteration from a clean constraint state
    move_group.clearPathConstraints();
    move_group.clearPoseTargets();

    if (lock_roll) {
      move_group.setPoseTarget(waypoints[i], ee_link);
    } else {
      moveit_msgs::msg::OrientationConstraint oc;
      oc.link_name = ee_link;
      oc.header.frame_id = global_frame;
      oc.orientation = waypoints[i].orientation;
      oc.absolute_x_axis_tolerance = 0.01;
      oc.absolute_y_axis_tolerance = M_PI;
      oc.absolute_z_axis_tolerance = 0.01;
      oc.weight = 1.0;

      moveit_msgs::msg::Constraints path_constraints;
      path_constraints.orientation_constraints.push_back(oc);
      move_group.setPathConstraints(path_constraints);

      move_group.setPositionTarget(
        waypoints[i].position.x,
        waypoints[i].position.y,
        waypoints[i].position.z,
        ee_link);
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool plan_ok = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!plan_ok) {
      ++failures;
      rows[i].status = WpStatus::HOMING;
      set_marker_color(sphere_array, arrow_array, i, COLOR_RED);
      publish_markers(sphere_pub, arrow_pub, sphere_array, arrow_array);
      render_table(rows, lock_roll, planner_label);
      go_home(move_group, home_pose_name, logger);
      rows[i].status = WpStatus::FAIL;
      render_table(rows, lock_roll, planner_label);
      continue;
    }

    bool exec_ok = (move_group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!exec_ok) {
      ++failures;
      rows[i].status = WpStatus::HOMING;
      set_marker_color(sphere_array, arrow_array, i, COLOR_RED);
      publish_markers(sphere_pub, arrow_pub, sphere_array, arrow_array);
      render_table(rows, lock_roll, planner_label);
      go_home(move_group, home_pose_name, logger);
      rows[i].status = WpStatus::FAIL;
      render_table(rows, lock_roll, planner_label);
      continue;
    }

    // Reached: query the actual pose to detect orientation differences
    ++successes;
    rows[i].status = WpStatus::DONE;

    auto current = move_group.getCurrentPose(ee_link);
    rows[i].actual = current.pose;
    rows[i].actual_set = true;

    // Threshold: 2 degrees of total quaternion difference => "reoriented"
    double diff_deg = quat_angle_diff(rows[i].target.orientation, current.pose.orientation)
                      * 180.0 / M_PI;
    rows[i].reoriented = (!lock_roll && diff_deg > 2.0);

    set_marker_color(sphere_array, arrow_array, i, COLOR_GREEN);
    publish_markers(sphere_pub, arrow_pub, sphere_array, arrow_array);
    render_table(rows, lock_roll, planner_label);
  }

  // Final summary line, leaves table on screen
  std::printf("\n%sScan complete:%s %d / %zu reached, %d failed.\n",
    ansi::BOLD, ansi::RESET, successes, waypoints.size(), failures);
  std::fflush(stdout);

  // Cleanup
  g_quit = true;
  std::printf("%s", ansi::SHOW_CURSOR);
  std::fflush(stdout);
  restore_terminal();

  rclcpp::shutdown();
  if (key_thread.joinable()) key_thread.detach();  // blocked on read(), can't join cleanly
  spinner.join();
  return 0;
}
