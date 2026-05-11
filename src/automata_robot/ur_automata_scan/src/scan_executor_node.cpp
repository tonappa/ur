#include <algorithm>
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
const Color COLOR_BLUE   = {0.0f, 0.4f, 1.0f};  // fallback waypoint

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
  FAIL,
  FALLBACK_ORIGIN  // original wp unreachable; a nearby fallback was executed instead
};

struct WpRow {
  geometry_msgs::msg::Pose target;
  WpStatus status = WpStatus::PENDING;
  bool actual_set = false;            // true if actual_pose has been filled
  geometry_msgs::msg::Pose actual;    // pose actually reached (may differ when lock_pitch=false)
  bool reoriented = false;            // true if actual orientation differs from target
  bool fallback_set = false;          // true if a nearby fallback pose was found
  geometry_msgs::msg::Pose fallback;  // fallback pose executed instead of target
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
    case WpStatus::PENDING:         return "PENDING";
    case WpStatus::RUNNING:         return "RUNNING";
    case WpStatus::DONE:            return "DONE";
    case WpStatus::HOMING:          return "HOMING";
    case WpStatus::FAIL:            return "FAIL";
    case WpStatus::FALLBACK_ORIGIN: return "FALLBACK";
  }
  return "?";
}

static const char * status_color(WpStatus s)
{
  switch (s) {
    case WpStatus::PENDING:         return ansi::GRAY;
    case WpStatus::RUNNING:         return ansi::YELLOW;
    case WpStatus::DONE:            return ansi::GREEN;
    case WpStatus::HOMING:          return ansi::YELLOW;
    case WpStatus::FAIL:            return ansi::RED;
    case WpStatus::FALLBACK_ORIGIN: return "\033[34m";  // blue
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

// Returns true if any arm link falls inside the camera's view cone toward the
// sphere center. We only check links that are between the TCP and the center
// (not behind the camera) and within threshold_rad of the camera-to-center axis.
// Links too close to the TCP (< 1 cm) are ignored — they are part of the wrist
// and can't physically occlude the scene.
static bool is_arm_occluding(
  const moveit::core::RobotState & state,
  const moveit::core::JointModelGroup * jmg,
  const std::string & ee_link,
  const Eigen::Vector3d & center,
  double threshold_rad)
{
  Eigen::Vector3d tcp_pos  = state.getGlobalLinkTransform(ee_link).translation();
  Eigen::Vector3d view_dir = (center - tcp_pos).normalized();
  double dist_to_center    = (center - tcp_pos).norm();

  for (const auto * link : jmg->getLinkModels()) {
    if (link->getName() == ee_link) continue;

    Eigen::Vector3d link_pos = state.getGlobalLinkTransform(link->getName()).translation();
    Eigen::Vector3d to_link  = link_pos - tcp_pos;
    double dist = to_link.norm();

    // Skip links that are too close to the TCP (wrist area) or behind the camera
    if (dist < 0.01) continue;

    double cos_angle = view_dir.dot(to_link / dist);
    double angle = std::acos(std::clamp(cos_angle, -1.0, 1.0));

    // Link is inside the view cone AND closer than the center (i.e. in the way)
    if (angle < threshold_rad && dist < dist_to_center) {
      return true;
    }
  }
  return false;
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

// Shrink the sphere marker for a FALLBACK_ORIGIN waypoint so it's visually
// distinct from normal markers (small gray dot instead of the usual size).
static void set_marker_small(MarkerArray & arr, size_t i)
{
  arr.markers[2 * i].scale.x = arr.markers[2 * i].scale.y = arr.markers[2 * i].scale.z = 0.010;
}

// Build a look-at pose: position at `pos`, Y-axis pointing toward `center`.
// Duplicates the logic in sphere_waypoint_generator.cpp so we don't need to
// expose it as a public symbol.
static geometry_msgs::msg::Pose make_lookat(
  const Eigen::Vector3d & pos, const Eigen::Vector3d & center)
{
  Eigen::Vector3d y_dir = (center - pos).normalized();
  Eigen::Vector3d ref   = (y_dir.cross(Eigen::Vector3d::UnitZ()).norm() > 1e-6)
                          ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitX();
  Eigen::Vector3d x_dir = y_dir.cross(ref).normalized();
  Eigen::Vector3d z_dir = x_dir.cross(y_dir);

  Eigen::Matrix3d rot;
  rot.col(0) = x_dir;
  rot.col(1) = y_dir;
  rot.col(2) = z_dir;
  Eigen::Quaterniond q(rot);

  geometry_msgs::msg::Pose pose;
  pose.position.x    = pos.x();
  pose.position.y    = pos.y();
  pose.position.z    = pos.z();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
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
  node->declare_parameter<double>     ("scan_equator_exclusion_upper_deg", 10.0);
  node->declare_parameter<double>     ("scan_equator_exclusion_lower_deg", 10.0);
  node->declare_parameter<bool>       ("scan_lock_pitch",                   true);
  node->declare_parameter<bool>       ("scan_stagger_rings",                false);
  node->declare_parameter<bool>       ("scan_adaptive_rings",               false);
  node->declare_parameter<bool>       ("scan_occlusion_check",              false);
  node->declare_parameter<double>     ("scan_occlusion_threshold_deg",      20.0);
  node->declare_parameter<bool>       ("scan_fallback_search",              false);
  node->declare_parameter<double>     ("scan_fallback_radius_mm",           20.0);
  node->declare_parameter<double>     ("scan_fallback_planning_time",        3.0);
  node->declare_parameter<int>        ("scan_fallback_max_plan_attempts",    3);
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
  const double      exclusion_upper_deg = node->get_parameter("scan_equator_exclusion_upper_deg").as_double();
  const double      exclusion_lower_deg = node->get_parameter("scan_equator_exclusion_lower_deg").as_double();
  const bool        lock_pitch          = node->get_parameter("scan_lock_pitch").as_bool();
  const bool        stagger_rings           = node->get_parameter("scan_stagger_rings").as_bool();
  const bool        adaptive_rings          = node->get_parameter("scan_adaptive_rings").as_bool();
  const bool        occlusion_check         = node->get_parameter("scan_occlusion_check").as_bool();
  const double      occlusion_threshold_rad =
    node->get_parameter("scan_occlusion_threshold_deg").as_double() * M_PI / 180.0;
  const bool        fallback_search     = node->get_parameter("scan_fallback_search").as_bool();
  const double      fallback_radius_mm  = node->get_parameter("scan_fallback_radius_mm").as_double();
  const double      fallback_planning_time = node->get_parameter("scan_fallback_planning_time").as_double();
  const int         fallback_max_plan_attempts = node->get_parameter("scan_fallback_max_plan_attempts").as_int();
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
  cfg.equator_exclusion_upper_rad = exclusion_upper_deg * M_PI / 180.0;
  cfg.equator_exclusion_lower_rad = exclusion_lower_deg * M_PI / 180.0;
  cfg.stagger_rings               = stagger_rings;
  cfg.adaptive_rings              = adaptive_rings;

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

  // Cronometro dall'inizio del primo waypoint alla fine del ciclo
  auto scan_start_time = std::chrono::steady_clock::now();

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
      RCLCPP_WARN(logger,
        "=== Transizione upper → lower: vado a '%s' prima del polo sud ===",
        lower_home_pose_name.c_str());
      recovery_pose_name = lower_home_pose_name;
      if (!go_home(move_group, recovery_pose_name, logger)) {
        RCLCPP_ERROR(logger,
          "Transizione FALLITA: '%s' non raggiungibile. Verifica che esista nel SRDF.",
          lower_home_pose_name.c_str());
      }
    }

    rows[i].status = WpStatus::RUNNING;
    set_marker_color(markers_array, i, COLOR_YELLOW);
    publish_markers(markers_pub, markers_array);
    g_redraw_request = true;
    render_table(rows, lock_pitch, planner_label);

    // Recovery strategy: ad ogni plan() fallito vai a home e ritenta lo STESSO
    // waypoint UNA volta. Se anche il secondo tentativo fallisce → FAIL.
    // home_retry_done tiene traccia se l'abbiamo già rifatto da home.
    bool home_retry_done = false;
    bool wp_resolved     = false;

    while (!wp_resolved) {
      if (g_quit.load() || !rclcpp::ok()) { wp_resolved = true; break; }

      // Always start each attempt from a clean constraint state
      move_group.clearPathConstraints();
      move_group.clearPoseTargets();

      bool ik_ok = true;
      double chosen_pitch_deg = 0.0;
      if (lock_pitch) {
        move_group.setPoseTarget(waypoints[i], ee_link);
      } else {
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
          Eigen::Quaterniond q_try = q_target * q_offset;

          geometry_msgs::msg::Pose pose_try;
          pose_try.position = waypoints[i].position;
          pose_try.orientation.w = q_try.w();
          pose_try.orientation.x = q_try.x();
          pose_try.orientation.y = q_try.y();
          pose_try.orientation.z = q_try.z();

          moveit::core::RobotState candidate(seed_state);
          if (!candidate.setFromIK(jmg, pose_try, ee_link, ik_timeout)) continue;
          // setFromIK aggiorna i giunti ma non sempre gli link transforms:
          // forzare update() prima di getGlobalLinkTransform per evitare
          // l'assertion 'checkLinkTransforms()' di MoveIt.
          candidate.update();

          if (occlusion_check && lower_start_index > 0 && i >= lower_start_index &&
              is_arm_occluding(candidate, jmg, ee_link, cfg.center, occlusion_threshold_rad)) {
            continue;
          }

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
        RCLCPP_INFO(logger, "wp %zu (try %d): pitch chosen %+.1f deg",
                    i, home_retry_done ? 2 : 1, chosen_pitch_deg);
      }

      if (!plan_ok) {
        if (!home_retry_done) {
          // Primo plan fallito → vai a home e ritenta lo stesso waypoint
          rows[i].status = WpStatus::HOMING;
          render_table(rows, lock_pitch, planner_label);
          go_home(move_group, recovery_pose_name, logger);
          home_retry_done = true;
          rows[i].status = WpStatus::RUNNING;
          render_table(rows, lock_pitch, planner_label);
          continue;  // ritenta lo stesso waypoint
        }
        // Già ritentato da home → prova la ricerca fallback nell'intorno
        bool fb_found = false;

        if (fallback_search) {
          RCLCPP_INFO(logger, "wp %zu: avvio ricerca fallback ...", i);

          // planning_time più basso e UN solo tentativo per il fallback:
          // se 1 attempt non basta, il candidato è probabilmente irraggiungibile,
          // meglio passare al prossimo candidato che non sprecare 3x il tempo.
          move_group.setPlanningTime(fallback_planning_time);
          move_group.setNumPlanningAttempts(1);

          const auto * jmg_fb = move_group.getRobotModel()->getJointModelGroup(planning_group);
          double fallback_angle_max = (fallback_radius_mm / 1000.0) / cfg.radius;

          Eigen::Vector3d wp_pos(
            waypoints[i].position.x, waypoints[i].position.y, waypoints[i].position.z);
          Eigen::Vector3d radial = (wp_pos - cfg.center).normalized();
          Eigen::Vector3d ref_ax = (std::abs(radial.dot(Eigen::Vector3d::UnitZ())) < 0.9)
                                   ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitX();
          Eigen::Vector3d tang_u = radial.cross(ref_ax).normalized();
          Eigen::Vector3d tang_v = radial.cross(tang_u);

          // Fase 1: raccogli TUTTI i candidati con IK valida (veloce, senza plan).
          // Ogni candidato è {pose desiderata, stato IK, distanza joint, mm di offset}.
          struct FbCandidate {
            geometry_msgs::msg::Pose pose;
            moveit::core::RobotState state;
            double joint_dist;
            double mm_off;
            FbCandidate(const geometry_msgs::msg::Pose & p,
                        const moveit::core::RobotState & s,
                        double dq, double mm)
              : pose(p), state(s), joint_dist(dq), mm_off(mm) {}
          };
          std::vector<FbCandidate> candidates;
          candidates.reserve(12);

          const moveit::core::RobotState seed_fb(*move_group.getCurrentState());

          // FAST mode: solo i pitch offset più piccoli (max 7: 0, ±step, ±2step, ±3step).
          // Riduce drasticamente le chiamate IK rispetto ai 25 offset del main loop.
          size_t max_pitch_fb = std::min<size_t>(7, pitch_offsets.size());

          // IK timeout più aggressivo (TRAC-IK risolve in <1ms se la pose è raggiungibile,
          // quindi 5ms è più che sufficiente; se non risolve in 5ms, è irraggiungibile).
          const double fb_ik_timeout = std::min(ik_timeout, 0.005);

          // 2 anelli × 6 direzioni = 12 punti candidato (era 24)
          for (int ring = 1; ring <= 2 && !g_quit.load(); ++ring) {
            double angle_off = fallback_angle_max * ring / 2.0;
            double mm_off    = angle_off * cfg.radius * 1000.0;

            for (int d = 0; d < 6 && !g_quit.load(); ++d) {
              double dir_angle = d * (2.0 * M_PI / 6.0);  // ogni 60°
              Eigen::Vector3d tangent = std::cos(dir_angle) * tang_u
                                      + std::sin(dir_angle) * tang_v;
              Eigen::Vector3d new_radial = std::cos(angle_off) * radial
                                         + std::sin(angle_off) * tangent;
              new_radial.normalize();
              Eigen::Vector3d cand_pos = cfg.center + cfg.radius * new_radial;
              geometry_msgs::msg::Pose cand_pose = make_lookat(cand_pos, cfg.center);

              // Pitch search per trovare la miglior IK su questo candidato
              double best_cost_fb = std::numeric_limits<double>::infinity();
              moveit::core::RobotState best_fb(seed_fb);
              bool ik_fb = false;

              Eigen::Quaterniond q_cand_base(
                cand_pose.orientation.w, cand_pose.orientation.x,
                cand_pose.orientation.y, cand_pose.orientation.z);

              for (size_t op = 0; op < max_pitch_fb; ++op) {
                double offset = pitch_offsets[op];
                Eigen::Quaterniond q_off(Eigen::AngleAxisd(offset, Eigen::Vector3d::UnitY()));
                Eigen::Quaterniond q_try = q_cand_base * q_off;
                geometry_msgs::msg::Pose pose_try = cand_pose;
                pose_try.orientation.w = q_try.w();
                pose_try.orientation.x = q_try.x();
                pose_try.orientation.y = q_try.y();
                pose_try.orientation.z = q_try.z();

                moveit::core::RobotState cand_state(seed_fb);
                if (!cand_state.setFromIK(jmg_fb, pose_try, ee_link, fb_ik_timeout)) continue;
                cand_state.update();

                if (occlusion_check && lower_start_index > 0 && i >= lower_start_index &&
                    is_arm_occluding(cand_state, jmg_fb, ee_link, cfg.center, occlusion_threshold_rad)) {
                  continue;
                }

                double dq   = joint_distance(seed_fb, cand_state, jmg_fb);
                double cost = dq + pitch_bias * std::abs(offset);
                if (cost < best_cost_fb) {
                  best_cost_fb = cost;
                  best_fb      = cand_state;
                  ik_fb        = true;
                }
              }

              if (ik_fb) {
                double dq_fb = joint_distance(seed_fb, best_fb, jmg_fb);
                candidates.emplace_back(cand_pose, best_fb, dq_fb, mm_off);
              }
            }
          }

          RCLCPP_INFO(logger,
            "  wp %zu fallback: %zu candidati con IK valida, provo plan() sui migliori %d",
            i, candidates.size(), fallback_max_plan_attempts);

          // Ordina per joint_distance crescente (più vicini al robot prima)
          std::sort(candidates.begin(), candidates.end(),
            [](const FbCandidate & a, const FbCandidate & b) {
              return a.joint_dist < b.joint_dist;
            });

          // Fase 2: prova plan() solo sui top N
          int attempts = std::min(static_cast<int>(candidates.size()), fallback_max_plan_attempts);
          for (int k = 0; k < attempts && !fb_found && !g_quit.load(); ++k) {
            const auto & c = candidates[k];
            RCLCPP_INFO(logger,
              "  wp %zu fallback: plan tentativo %d/%d (offset %.0f mm, dq %.3f)",
              i, k + 1, attempts, c.mm_off, c.joint_dist);

            move_group.clearPathConstraints();
            move_group.clearPoseTargets();
            move_group.setJointValueTarget(c.state);

            moveit::planning_interface::MoveGroupInterface::Plan fb_plan;
            if (move_group.plan(fb_plan) != moveit::core::MoveItErrorCode::SUCCESS) continue;

            if (move_group.execute(fb_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
              go_home(move_group, recovery_pose_name, logger);
              continue;
            }

            // Fallback raggiunto
            fb_found = true;
            ++successes;
            rows[i].fallback      = c.pose;
            rows[i].fallback_set  = true;
            rows[i].status        = WpStatus::FALLBACK_ORIGIN;

            set_marker_color(markers_array, i, COLOR_GRAY);
            set_marker_small(markers_array, i);

            int fb_id = static_cast<int>(waypoints.size()) * 2 + static_cast<int>(i) * 2;
            Marker fb_sphere = make_sphere_marker(fb_id,     c.pose, global_frame, COLOR_BLUE);
            Marker fb_arrow  = make_arrow_marker (fb_id + 1, c.pose, cfg.center,   global_frame, COLOR_BLUE);
            fb_sphere.ns = "fallback_waypoints";
            fb_arrow.ns  = "fallback_orientations";
            markers_array.markers.push_back(fb_sphere);
            markers_array.markers.push_back(fb_arrow);

            publish_markers(markers_pub, markers_array);
            render_table(rows, lock_pitch, planner_label);
          }

          // Ripristina i parametri di planning normali per i waypoint successivi
          move_group.setPlanningTime(planning_time);
          move_group.setNumPlanningAttempts(planning_attempts);
        }

        if (!fb_found) {
          // Niente da fare: né plan da home né fallback nell'intorno
          ++failures;
          if (!ik_ok) ++ik_failures;
          else        ++plan_failures;
          set_marker_color(markers_array, i, COLOR_RED);
          publish_markers(markers_pub, markers_array);
          rows[i].status = WpStatus::FAIL;
          render_table(rows, lock_pitch, planner_label);
        }

        wp_resolved = true;
        break;
      }

      moveit::core::MoveItErrorCode exec_code = move_group.execute(plan);
      bool exec_ok = (exec_code == moveit::core::MoveItErrorCode::SUCCESS);

      if (!exec_ok) {
        // execute() fallito: robot fermo a metà traiettoria → home per sicurezza.
        // Logghiamo il codice errore per capire perché (tolerance, goal_time, protective_stop...).
        RCLCPP_ERROR(logger,
          "wp %zu: execute() FALLITO con codice MoveIt = %d (%s). "
          "Verifica il terminale del bringup per il messaggio del controller (es. 'goal tolerance violated', 'protective stop').",
          i, static_cast<int>(exec_code.val), exec_code.message.c_str());

        ++failures;
        ++exec_failures;
        rows[i].status = WpStatus::HOMING;
        set_marker_color(markers_array, i, COLOR_RED);
        publish_markers(markers_pub, markers_array);
        render_table(rows, lock_pitch, planner_label);
        go_home(move_group, recovery_pose_name, logger);
        rows[i].status = WpStatus::FAIL;
        render_table(rows, lock_pitch, planner_label);
        wp_resolved = true;
        break;
      }

      // Reached
      ++successes;
      rows[i].status = WpStatus::DONE;

      auto current = move_group.getCurrentPose(ee_link);
      rows[i].actual = current.pose;
      rows[i].actual_set = true;

      double diff_deg = quat_angle_diff(rows[i].target.orientation, current.pose.orientation)
                        * 180.0 / M_PI;
      rows[i].reoriented = (!lock_pitch && diff_deg > 2.0);

      set_marker_color(markers_array, i, COLOR_GREEN);
      publish_markers(markers_pub, markers_array);
      render_table(rows, lock_pitch, planner_label);
      wp_resolved = true;
    }
  }


  // Calcolo tempo totale di scansione (dal primo waypoint all'ultimo)
  auto scan_end_time = std::chrono::steady_clock::now();
  double scan_duration_s = std::chrono::duration<double>(scan_end_time - scan_start_time).count();

  // Ritorno a home a fine scansione (se non killato)
  if (!g_quit.load() && rclcpp::ok()) {
    RCLCPP_WARN(logger, "=== Scan completato, ritorno a '%s' ===", home_pose_name.c_str());
    go_home(move_group, home_pose_name, logger);
  }

  // Conteggio dei fallback effettivamente trovati nel main loop
  int fallback_total = 0;
  for (const auto & r : rows) {
    if (r.status == WpStatus::FALLBACK_ORIGIN) ++fallback_total;
  }

  // Final summary line, leaves table on screen
  std::printf("\n%sScan complete:%s %d / %zu reached, %d failed (%d IK, %d plan, %d execute).",
    ansi::BOLD, ansi::RESET, successes, waypoints.size(), failures,
    ik_failures, plan_failures, exec_failures);
  if (fallback_total > 0) {
    std::printf("  Fallback: %d punti alternativi.", fallback_total);
  }
  std::printf("\n%sTempo scansione:%s %.1f s (%.1f min)\n",
    ansi::BOLD, ansi::RESET, scan_duration_s, scan_duration_s / 60.0);
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
