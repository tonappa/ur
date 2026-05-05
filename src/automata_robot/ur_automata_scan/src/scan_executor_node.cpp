#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
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
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <std_srvs/srv/trigger.hpp>

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
    // No interactive terminal (e.g. lanciato via ros2 launch): tasti disabilitati.
    // Resta in pausa: lo start arriva dal service /scan_executor/start.
    g_stdin_is_tty = false;
    return;
  }
  g_stdin_is_tty = true;

  if (tcgetattr(STDIN_FILENO, &g_old_termios) != 0) {
    g_stdin_is_tty = false;
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
  geometry_msgs::msg::Pose actual;    // pose actually reached (may differ when lock_pitch=false)
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

// Build alternating pitch-offset sequence in radians: [0, +step, -step, +2·step, -2·step, ...]
// up to ±range (inclusive). Used when lock_pitch=false to sample roll-around-Y_TCP
// candidates for IK.
static std::vector<double> build_pitch_offsets_rad(double range_deg, double step_deg)
{
  std::vector<double> offsets;
  offsets.push_back(0.0);
  if (step_deg <= 0.0 || range_deg <= 0.0) return offsets;
  const double range_rad = range_deg * M_PI / 180.0;
  const double step_rad  = step_deg  * M_PI / 180.0;
  for (double k = step_rad; k <= range_rad + 1e-9; k += step_rad) {
    offsets.push_back(+k);
    offsets.push_back(-k);
  }
  return offsets;
}

// Distanza joint-space tra due RobotState sui giunti attivi del JointModelGroup.
// L2 sulle differenze dei giunti rivoluti; differenze wrappate su [-π, π] così
// un wrist flip da +179° a -179° conta 2°, non 358°.
static double joint_distance(
  const moveit::core::RobotState & a,
  const moveit::core::RobotState & b,
  const moveit::core::JointModelGroup * jmg)
{
  double sum = 0.0;
  for (const auto * j : jmg->getActiveJointModels()) {
    const double va = a.getJointPositions(j)[0];
    const double vb = b.getJointPositions(j)[0];
    double diff = va - vb;
    while (diff >  M_PI) diff -= 2.0 * M_PI;
    while (diff < -M_PI) diff += 2.0 * M_PI;
    sum += diff * diff;
  }
  return std::sqrt(sum);
}

// ============================================================================
// Render the full table in place
// ============================================================================
static std::mutex g_render_mutex;

static void render_table(const std::vector<WpRow> & rows, bool lock_pitch,
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
  std::printf("Services: %s/scan_executor_node/start%s   %s/scan_executor_node/pause%s   (std_srvs/srv/Trigger)\n",
    ansi::BOLD, ansi::RESET, ansi::BOLD, ansi::RESET);
  std::printf("Planner: %s%s%s   Pitch mode: %s%s%s\n",
    ansi::BOLD, planner_label.c_str(), ansi::RESET,
    ansi::BOLD,
    lock_pitch ? "LOCKED  (X horizontal, no rotation around Y_TCP)" : "FREE    (MoveIt picks pitch around Y_TCP)",
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

// Layout dell'array unificato: per ogni waypoint i ci sono 2 marker consecutivi,
//   markers[2*i]   → SPHERE  (namespace "scan_waypoints")
//   markers[2*i+1] → ARROW   (namespace "scan_orientations")
// Stesso colore su entrambi così sphere e arrow cambiano colore insieme.
static void set_marker_color(MarkerArray & arr, size_t i, const Color & c)
{
  arr.markers[2 * i].color.r     = c.r;
  arr.markers[2 * i].color.g     = c.g;
  arr.markers[2 * i].color.b     = c.b;
  arr.markers[2 * i + 1].color.r = c.r;
  arr.markers[2 * i + 1].color.g = c.g;
  arr.markers[2 * i + 1].color.b = c.b;
}

static void publish_markers(
  rclcpp::Publisher<MarkerArray>::SharedPtr pub,
  const MarkerArray & arr)
{
  pub->publish(arr);
}

static bool go_home(moveit::planning_interface::MoveGroupInterface & move_group,
                    const std::string & home_pose_name,
                    const rclcpp::Logger & logger)
{
  RCLCPP_WARN(logger, "Recovery: moving back to '%s' ...", home_pose_name.c_str());

  // Drop any active path constraint (e.g. the look-at orientation constraint
  // used in free-pitch mode) — the home pose cannot satisfy it.
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
static void wait_while_paused(const std::vector<WpRow> & rows, bool lock_pitch,
                              const std::string & planner_label)
{
  while (g_paused.load() && !g_quit.load() && rclcpp::ok()) {
    if (g_redraw_request.load()) render_table(rows, lock_pitch, planner_label);
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
  }
  if (!g_quit.load()) {
    g_redraw_request = true;
    render_table(rows, lock_pitch, planner_label);
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

  // --- Service interface: /scan_executor/start e /scan_executor/pause ---
  // Lo scan parte sempre in pausa. Per farlo partire (o riprendere dopo una pausa)
  //   ros2 service call /scan_executor/start std_srvs/srv/Trigger {}
  // Per metterlo in pausa mid-scan
  //   ros2 service call /scan_executor/pause std_srvs/srv/Trigger {}
  // I tasti SPACE/Q restano attivi se il nodo gira su un TTY interattivo.
  using TriggerSrv = std_srvs::srv::Trigger;

  auto start_srv = node->create_service<TriggerSrv>(
    "~/start",
    [](const std::shared_ptr<TriggerSrv::Request> /*req*/,
       std::shared_ptr<TriggerSrv::Response> res)
    {
      g_paused = false;
      g_redraw_request = true;
      res->success = true;
      res->message = "scan started/resumed";
    });

  auto pause_srv = node->create_service<TriggerSrv>(
    "~/pause",
    [](const std::shared_ptr<TriggerSrv::Request> /*req*/,
       std::shared_ptr<TriggerSrv::Response> res)
    {
      g_paused = true;
      g_redraw_request = true;
      res->success = true;
      res->message = "scan paused";
    });

  // --- Parameters ---
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
  node->declare_parameter<int>        ("scan_num_rings",            4);   // latitudinal
  node->declare_parameter<int>        ("scan_points_per_ring",      8);   // latitudinal
  node->declare_parameter<int>        ("scan_num_arc",              6);   // longitudinal
  node->declare_parameter<int>        ("scan_points_per_arc",       5);   // longitudinal
  node->declare_parameter<double>     ("scan_equator_exclusion_deg",10.0);
  node->declare_parameter<bool>       ("scan_lock_pitch",             true);
  node->declare_parameter<std::string>("scan_planner",                "ompl");
  node->declare_parameter<std::string>("scan_ompl_algorithm",         "RRTConnect");
  node->declare_parameter<double>     ("scan_pitch_search_range_deg", 90.0);
  node->declare_parameter<double>     ("scan_pitch_search_step_deg",  15.0);
  node->declare_parameter<double>     ("scan_pitch_xparallel_bias",    0.05);
  node->declare_parameter<double>     ("scan_ik_timeout",              0.2);
  node->declare_parameter<double>     ("scan_planning_time",           5.0);
  node->declare_parameter<int>        ("scan_planning_attempts",       1);

  const std::string global_frame    = node->get_parameter("global_frame").as_string();
  const std::string planning_group  = node->get_parameter("planning_group").as_string();
  const std::string ee_link         = node->get_parameter("end_effector_link").as_string();
  const std::string home_pose_name        = node->get_parameter("home_pose_name").as_string();
  const std::string lower_home_pose_name  = node->get_parameter("lower_home_pose_name").as_string();
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
  const bool        lock_pitch      = node->get_parameter("scan_lock_pitch").as_bool();
  const std::string planner_str     = node->get_parameter("scan_planner").as_string();
  const std::string ompl_algorithm  = node->get_parameter("scan_ompl_algorithm").as_string();
  const double      pitch_range_deg = node->get_parameter("scan_pitch_search_range_deg").as_double();
  const double      pitch_step_deg  = node->get_parameter("scan_pitch_search_step_deg").as_double();
  const double      pitch_bias      = node->get_parameter("scan_pitch_xparallel_bias").as_double();
  const double      ik_timeout      = node->get_parameter("scan_ik_timeout").as_double();
  const double      planning_time   = node->get_parameter("scan_planning_time").as_double();
  const int         planning_attempts = node->get_parameter("scan_planning_attempts").as_int();

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
  // In modalità FULL li generiamo come due chiamate separate (upper + lower) e
  // teniamo l'indice di confine: prima del primo waypoint del lower hemisphere
  // facciamo passare il robot dalla home, così il braccio si "ri-orienta" prima
  // di tuffarsi sotto l'equatore (evita transizioni cinematiche brutte tra
  // l'ultimo waypoint dell'upper e il polo sud).
  std::vector<geometry_msgs::msg::Pose> waypoints;
  size_t lower_start_index = 0;  // 0 = nessuna transizione (upper-only o lower-only)
  if (cfg.hemisphere == HEMISPHERE_FULL) {
    ScanConfig cfg_upper = cfg; cfg_upper.hemisphere = HEMISPHERE_UPPER;
    ScanConfig cfg_lower = cfg; cfg_lower.hemisphere = HEMISPHERE_LOWER;
    auto upper_pts = generate_waypoints(cfg_upper);
    auto lower_pts = generate_waypoints(cfg_lower);
    lower_start_index = upper_pts.size();
    waypoints.reserve(upper_pts.size() + lower_pts.size());
    waypoints.insert(waypoints.end(), upper_pts.begin(), upper_pts.end());
    waypoints.insert(waypoints.end(), lower_pts.begin(), lower_pts.end());
  } else {
    waypoints = generate_waypoints(cfg);
  }

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

  // --- Marker publisher unificato + initial gray markers ---
  // Un solo topic con un solo MarkerArray. I due marker per waypoint (sphere + arrow)
  // sono distinguibili dal namespace e occupano posizioni consecutive nell'array.
  auto markers_pub = node->create_publisher<MarkerArray>("/scan_waypoints_markers", 10);

  MarkerArray markers_array;
  markers_array.markers.reserve(2 * waypoints.size());
  for (size_t i = 0; i < waypoints.size(); ++i) {
    markers_array.markers.push_back(make_sphere_marker(static_cast<int>(i), waypoints[i], global_frame, COLOR_GRAY));
    markers_array.markers.push_back(make_arrow_marker( static_cast<int>(i), waypoints[i], cfg.center, global_frame, COLOR_GRAY));
  }
  for (int attempt = 0; attempt < 5; ++attempt) {
    publish_markers(markers_pub, markers_array);
    rclcpp::sleep_for(std::chrono::milliseconds(200));
  }

  // Pre-compute the pitch sampling sequence (used only when lock_pitch=false).
  const std::vector<double> pitch_offsets = build_pitch_offsets_rad(pitch_range_deg, pitch_step_deg);

  // --- MoveIt setup ---
  moveit::planning_interface::MoveGroupInterface move_group(node, planning_group);
  move_group.setEndEffectorLink(ee_link);
  move_group.setPoseReferenceFrame(global_frame);
  move_group.setMaxVelocityScalingFactor(scaling);
  move_group.setMaxAccelerationScalingFactor(scaling);
  move_group.setPlanningTime(planning_time);
  move_group.setNumPlanningAttempts(planning_attempts);
  // Workspace bounds AABB: size (1.5, 1.75, 1.5) m, center (0, 0.25, 0.5) in world.
  // NB: è solo un hint per il sampling cartesiano di OMPL, non un vincolo geometrico
  // sull'EE — con joint goal viene di fatto ignorato. Per vincolare davvero il TCP
  // servono PositionConstraint o collision objects nella PlanningScene.
  move_group.setWorkspace(-0.75, -0.625, -0.25,   // min x,y,z
                          +0.75, +1.125, +1.25);  // max x,y,z

  // Resolve `scan_planner` string to (pipeline_id, planner_id).
  // For OMPL the algorithm comes from `scan_ompl_algorithm` (e.g. RRTConnect,
  // RRTstar, PRM, …) and is suffixed with `kConfigDefault` per MoveIt convention.
  // Default fallback is OMPL/RRTConnect if the planner string is unknown.
  std::string pipeline_id = "ompl";
  std::string planner_id  = ompl_algorithm + "kConfigDefault";
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
      "WARNING: scan_planner='%s' unknown, falling back to ompl/%s.\n",
      planner_str.c_str(), ompl_algorithm.c_str());
  }
  move_group.setPlanningPipelineId(pipeline_id);
  move_group.setPlannerId(planner_id);
  const std::string planner_label = pipeline_id + " / " + planner_id;

  // Pilz pipelines do not honour setPathConstraints — the look-at constraint
  // used in free-pitch mode would be silently ignored. Warn the user so they
  // don't chase a phantom bug.
  if (!lock_pitch && pipeline_id == "pilz_industrial_motion_planner") {
    std::fprintf(stderr,
      "WARNING: planner='%s' does not support orientation path constraints; "
      "lock_pitch=false will behave as a plain position target (no pitch lock).\n",
      planner_str.c_str());
  }

  // (lo spinner che processa anche i service /start e /pause è già attivo, vedi sopra)

  // --- Interactive terminal: spacebar pause/resume + Q to quit ---
  setup_raw_terminal();
  std::printf("%s", ansi::HIDE_CURSOR);
  std::thread key_thread(key_reader_loop);

  // First render — il nodo parte in pausa, in attesa di SPACE o del service /start
  render_table(rows, lock_pitch, planner_label);

  // Aspetta il primo via libera (SPACE da TTY, oppure service /scan_executor/start)
  wait_while_paused(rows, lock_pitch, planner_label);

  // --- Visit each waypoint ---
  int successes      = 0;
  int failures       = 0;
  int ik_failures    = 0;   // nessun pitch ha trovato IK valida
  int plan_failures  = 0;   // IK ok ma OMPL non è riuscito a pianificare
  int exec_failures  = 0;   // plan ok ma il controller ha rifiutato l'esecuzione
  // Pose di recovery: home tradizionale per upper hemisphere, scan_lower_ready
  // per lower (più vicino ai waypoint del polo sud, riduce i fail di planning).
  // Inizializzata in base alla modalità di scan; in FULL viene aggiornata al
  // momento della transizione upper→lower.
  std::string recovery_pose_name =
    (cfg.hemisphere == HEMISPHERE_LOWER) ? lower_home_pose_name : home_pose_name;
  // Contatore di plan() consecutivi falliti. Quando plan() fallisce, l'idea è
  // NON andare a home (il robot non si è mosso, andare a home è solo tempo
  // sprecato): tentiamo subito il prossimo waypoint dalla posizione attuale.
  // Solo dopo `recovery_after_consecutive_fails` fallimenti di fila il robot
  // potrebbe essere finito in una configurazione "scomoda" da cui non si esce
  // più → recovery a home come reset, e il contatore riparte da zero.
  int consecutive_plan_fails = 0;
  const int recovery_after_consecutive_fails = 3;

  for (size_t i = 0; i < waypoints.size(); ++i) {
    if (g_quit.load() || !rclcpp::ok()) break;

    // Pause check: if paused mid-scan, finish the current iteration loop here
    wait_while_paused(rows, lock_pitch, planner_label);
    if (g_quit.load() || !rclcpp::ok()) break;

    // Transizione tra emisfero superiore e inferiore in modalità FULL: passiamo
    // per la pose `lower_home_pose_name` (es. scan_lower_ready) prima di iniziare
    // l'emisfero inferiore. Più vicina ai waypoint del polo sud rispetto a home,
    // quindi più probabilità di planning success per i primi waypoint lower.
    // Da qui in avanti anche le recovery dei fail useranno questa pose.
    if (lower_start_index > 0 && i == lower_start_index) {
      recovery_pose_name = lower_home_pose_name;
      go_home(move_group, recovery_pose_name, logger);
      consecutive_plan_fails = 0;  // partenza pulita per il lower hemisphere
    }

    rows[i].status = WpStatus::RUNNING;
    set_marker_color(markers_array, i, COLOR_YELLOW);
    publish_markers(markers_pub, markers_array);
    g_redraw_request = true;
    render_table(rows, lock_pitch, planner_label);

    // Always start each iteration from a clean constraint state
    move_group.clearPathConstraints();
    move_group.clearPoseTargets();

    bool ik_ok = true;
    double chosen_pitch_deg = 0.0;
    if (lock_pitch) {
      move_group.setPoseTarget(waypoints[i], ee_link);
    } else {
      // Pitch libero. Strategia:
      //   - Y_TCP della camera deve sempre puntare al centro → rotazione attorno
      //     all'asse Y_TCP locale (Ry post-moltiplicazione) la lascia invariata.
      //   - Per ogni offset ∈ pitch_offsets, IK con seed = stato corrente.
      //   - Tra gli offset con IK valida, scegliamo quello che minimizza
      //     joint_distance(soluzione, stato_corrente) + bias·|offset|.
      //   - Effetto: traiettoria minima nei giunti tra waypoint adiacenti
      //     (niente wrist flip / sbracciate), con preferenza moderata per
      //     X parallelo (offset 0) a parità di costo.
      const auto * jmg = move_group.getRobotModel()->getJointModelGroup(planning_group);
      const moveit::core::RobotState seed_state(*move_group.getCurrentState());

      Eigen::Quaterniond q_target(
        waypoints[i].orientation.w,
        waypoints[i].orientation.x,
        waypoints[i].orientation.y,
        waypoints[i].orientation.z);

      double best_cost = std::numeric_limits<double>::infinity();
      moveit::core::RobotState best_state(seed_state);
      double best_offset_rad = 0.0;
      ik_ok = false;

      for (double offset : pitch_offsets) {
        Eigen::Quaterniond q_offset(Eigen::AngleAxisd(offset, Eigen::Vector3d::UnitY()));
        Eigen::Quaterniond q_try = q_target * q_offset;  // rotazione attorno a Y locale

        geometry_msgs::msg::Pose pose_try;
        pose_try.position = waypoints[i].position;
        pose_try.orientation.w = q_try.w();
        pose_try.orientation.x = q_try.x();
        pose_try.orientation.y = q_try.y();
        pose_try.orientation.z = q_try.z();

        moveit::core::RobotState candidate(seed_state);  // IK seed = stato corrente
        if (!candidate.setFromIK(jmg, pose_try, ee_link, ik_timeout)) continue;

        const double dq   = joint_distance(seed_state, candidate, jmg);
        const double cost = dq + pitch_bias * std::abs(offset);

        if (cost < best_cost) {
          best_cost = cost;
          best_state = candidate;
          best_offset_rad = offset;
          ik_ok = true;
        }
      }

      if (ik_ok) {
        chosen_pitch_deg = best_offset_rad * 180.0 / M_PI;
        move_group.setJointValueTarget(best_state);
      }
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool plan_ok = ik_ok &&
      (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!lock_pitch && ik_ok) {
      RCLCPP_INFO(logger, "wp %zu: pitch chosen %+.1f deg", i, chosen_pitch_deg);
    }

    if (!plan_ok) {
      // plan() fallito: il robot NON si è mosso, niente go_home a vuoto.
      // Marca FAIL e prova subito il prossimo waypoint dalla stessa posizione.
      // Se accumuliamo troppi fail di fila → recovery a home come reset.
      ++failures;
      ++consecutive_plan_fails;
      // Distinguiamo IK-fail (nessun pitch ha trovato IK) da plan-fail
      // (IK ok ma OMPL non ha trovato un percorso): aiuta a capire dove
      // attaccare i miglioramenti successivi.
      if (!ik_ok) ++ik_failures;
      else        ++plan_failures;
      set_marker_color(markers_array, i, COLOR_RED);
      publish_markers(markers_pub, markers_array);

      if (consecutive_plan_fails >= recovery_after_consecutive_fails) {
        rows[i].status = WpStatus::HOMING;
        render_table(rows, lock_pitch, planner_label);
        go_home(move_group, recovery_pose_name, logger);
        consecutive_plan_fails = 0;  // reset dopo il recovery
      }
      rows[i].status = WpStatus::FAIL;
      render_table(rows, lock_pitch, planner_label);
      continue;
    }

    // plan() ok → contatore di fail consecutivi azzerato.
    consecutive_plan_fails = 0;

    bool exec_ok = (move_group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!exec_ok) {
      // execute() fallito: il robot potrebbe essersi fermato a metà traiettoria
      // in una posizione incerta. go_home come fallback di sicurezza.
      ++failures;
      ++exec_failures;
      rows[i].status = WpStatus::HOMING;
      set_marker_color(markers_array, i, COLOR_RED);
      publish_markers(markers_pub, markers_array);
      render_table(rows, lock_pitch, planner_label);
      go_home(move_group, recovery_pose_name, logger);
      rows[i].status = WpStatus::FAIL;
      render_table(rows, lock_pitch, planner_label);
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
    rows[i].reoriented = (!lock_pitch && diff_deg > 2.0);

    set_marker_color(markers_array, i, COLOR_GREEN);
    publish_markers(markers_pub, markers_array);
    render_table(rows, lock_pitch, planner_label);
  }

  // Final summary line, leaves table on screen
  std::printf("\n%sScan complete:%s %d / %zu reached, %d failed (%d IK, %d plan, %d execute).\n",
    ansi::BOLD, ansi::RESET, successes, waypoints.size(), failures,
    ik_failures, plan_failures, exec_failures);
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
