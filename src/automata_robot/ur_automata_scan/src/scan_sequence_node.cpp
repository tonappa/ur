// scan_sequence_node: evoluzione di scan_executor_node (che resta come backup).
// Differenze rispetto al nodo originale:
//   Step 1: ogni candidato IK viene controllato contro la planning scene
//           (collisioni con scena e auto-collisioni) PRIMA di chiamare plan().
//   Step 2: catena di planner (Pilz CIRC -> Pilz PTP -> STOMP -> OMPL): ogni
//           tratto prova i planner in ordine e si ferma al primo che riesce.
//   Step 3: prima di muovere il robot si enumerano TUTTE le configurazioni
//           valide di ogni waypoint (pitch x rami IK) e una programmazione
//           dinamica a strati sceglie la sequenza con il percorso minimo nei
//           giunti. I waypoint irraggiungibili si conoscono prima di partire.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <future>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <mutex>
#include <tuple>
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
#include <moveit/planning_scene/planning_scene.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/bodies.h>
#include <geometric_shapes/body_operations.h>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "ur_automata_scan/sphere_waypoint_generator.hpp"
#include "ur_automata_scan/scan_sequence_planner.hpp"

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
const Color COLOR_ORANGE = {1.0f, 0.5f, 0.0f};  // view blocked by a scene object

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
    // Resta in pausa: lo start arriva dal service /scan_sequence/start.
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
  UNREACHABLE,     // nessuna configurazione valida trovata in fase di enumerazione
  OCCLUDED,        // un oggetto della scena sta fra la camera e il centro: scartato
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
    case WpStatus::UNREACHABLE:     return "UNREACHABLE";
    case WpStatus::OCCLUDED:        return "OCCLUDED";
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
    case WpStatus::UNREACHABLE:     return ansi::RED;
    case WpStatus::OCCLUDED:        return "\033[35m";  // magenta
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
// sphere center. The arm is treated as a chain of segments between the origins
// of consecutive joints (shoulder -> elbow -> wrist 1 -> 2 -> 3); every segment
// is sampled in a few points and each point is tested against the cone, so a
// long link crossing the line of sight is caught even when its origin lies far
// off-axis (checking origins only let the forearm sit in front of the camera).
// Only points between the TCP and the center count (not behind the camera);
// points closer than 1 cm to the TCP belong to the wrist and are skipped.
// ----------------------------------------------------------------------------
// Linea di vista camera -> centro contro gli oggetti della scena.
// Segmento dal waypoint al centro: se attraversa un oggetto del mondo la foto
// da quel punto inquadra l'ostacolo (es. lo stelo della piattaforma), quindi
// il waypoint va scartato prima ancora di cercare le IK.
// Ignorati: il marker del centro, l'oggetto da scansionare, i keep-out di
// margine e le intersezioni entro 3 cm dal centro (il disco sotto l'oggetto:
// i raggi dell'emisfero inferiore lo attraversano a 4 mm dal centro).
// ponytail: un solo raggio lungo l'asse ottico; campionare un cono di raggi
// se conta l'apertura della camera.
// ----------------------------------------------------------------------------

// Moller-Trumbore: intersezione raggio (origin, dir unitario) / triangolo.
// Ritorna true e la distanza t lungo il raggio se il raggio colpisce il triangolo.
static bool ray_hits_triangle(
  const Eigen::Vector3d & origin, const Eigen::Vector3d & dir,
  const Eigen::Vector3d & v0, const Eigen::Vector3d & v1, const Eigen::Vector3d & v2,
  double & t_out)
{
  const double eps = 1e-9;
  Eigen::Vector3d e1 = v1 - v0;
  Eigen::Vector3d e2 = v2 - v0;
  Eigen::Vector3d p = dir.cross(e2);
  double det = e1.dot(p);
  if (std::abs(det) < eps) return false;   // raggio parallelo al triangolo
  double inv_det = 1.0 / det;
  Eigen::Vector3d s = origin - v0;
  double u = inv_det * s.dot(p);
  if (u < 0.0 || u > 1.0) return false;
  Eigen::Vector3d q = s.cross(e1);
  double v = inv_det * dir.dot(q);
  if (v < 0.0 || u + v > 1.0) return false;
  t_out = inv_det * e2.dot(q);
  return t_out > eps;
}

static bool is_scene_occluding(
  const planning_scene::PlanningScene & scene,
  const Eigen::Vector3d & from,
  const Eigen::Vector3d & center,
  std::string & hit_object)
{
  static const std::set<std::string> ignored_objects = {
    "support_center", "artefact", "platform_margin", "table_margin"};
  const double ignore_near_center = 0.03;   // m

  const Eigen::Vector3d seg = center - from;
  const double seg_len = seg.norm();
  if (seg_len < 1e-6) return false;
  const Eigen::Vector3d dir = seg / seg_len;
  // Conta solo le intersezioni sul segmento e non a ridosso del centro.
  auto hit_counts = [&](double t) { return t > 0.0 && t < seg_len - ignore_near_center; };

  collision_detection::WorldConstPtr world = scene.getWorld();
  for (const std::string & id : world->getObjectIds()) {
    if (ignored_objects.count(id) > 0) continue;
    collision_detection::World::ObjectConstPtr obj = world->getObject(id);
    if (!obj) continue;
    for (size_t k = 0; k < obj->shapes_.size(); ++k) {
      const shapes::Shape * shape = obj->shapes_[k].get();
      const Eigen::Isometry3d & pose = obj->global_shape_poses_[k];

      if (shape->type == shapes::MESH) {
        // Raggio portato nel frame della mesh: una trasformazione sola invece
        // di una per vertice.
        const auto * mesh = static_cast<const shapes::Mesh *>(shape);
        Eigen::Isometry3d inv = pose.inverse();
        Eigen::Vector3d o = inv * from;
        Eigen::Vector3d d = inv.linear() * dir;
        auto vertex = [&](unsigned int idx) {
          const double * v = mesh->vertices + 3 * idx;
          return Eigen::Vector3d(v[0], v[1], v[2]);
        };
        for (unsigned int tri = 0; tri < mesh->triangle_count; ++tri) {
          const unsigned int * idx = mesh->triangles + 3 * tri;
          double t = 0.0;
          if (ray_hits_triangle(o, d, vertex(idx[0]), vertex(idx[1]), vertex(idx[2]), t) &&
              hit_counts(t)) {
            hit_object = id;
            return true;
          }
        }
      } else {
        // Primitive (box, cilindro, sfera): geometric_shapes fa il raycast.
        std::unique_ptr<bodies::Body> body(bodies::createBodyFromShape(shape));
        if (!body) continue;
        body->setPose(pose);
        EigenSTL::vector_Vector3d points;
        if (!body->intersectsRay(from, dir, &points)) continue;
        for (const Eigen::Vector3d & p : points) {
          if (hit_counts((p - from).dot(dir))) {
            hit_object = id;
            return true;
          }
        }
      }
    }
  }
  return false;
}

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

  // Skeleton: origin of the child link of every active joint, in chain order.
  std::vector<Eigen::Vector3d> skeleton;
  for (const auto * joint : jmg->getActiveJointModels()) {
    skeleton.push_back(state.getGlobalLinkTransform(joint->getChildLinkModel()).translation());
  }

  const int samples = 5;   // points per segment, both ends included
  for (size_t i = 0; i + 1 < skeleton.size(); ++i) {
    for (int k = 0; k < samples; ++k) {
      double t = static_cast<double>(k) / (samples - 1);
      Eigen::Vector3d point = skeleton[i] + t * (skeleton[i + 1] - skeleton[i]);
      Eigen::Vector3d to_point = point - tcp_pos;
      double dist = to_point.norm();
      if (dist < 0.01) continue;

      double cos_angle = view_dir.dot(to_point / dist);
      double angle = std::acos(std::clamp(cos_angle, -1.0, 1.0));

      // Point is inside the view cone AND closer than the center (i.e. in the way)
      if (angle < threshold_rad && dist < dist_to_center) {
        return true;
      }
    }
  }
  return false;
}

// ============================================================================
// Render the full table in place
// ============================================================================
static std::mutex g_render_mutex;
static std::string g_sequence_summary;   // riepilogo della sequenza pianificata (fase 2)

static void render_table(const std::vector<WpRow> & rows, bool lock_pitch,
                         const std::string & planner_label)
{
  std::lock_guard<std::mutex> lock(g_render_mutex);

  std::printf("%s%s", ansi::CURSOR_HOME, ansi::CLEAR_SCREEN);

  // Header banner
  const char * mode_str = g_paused.load() ? "PAUSED " : "RUNNING";
  const char * mode_col = g_paused.load() ? ansi::YELLOW : ansi::GREEN;
  std::printf("%s%sUR_AUTOMATA SCAN (sequence)  [%s%s%s]%s   %sSPACE%s = pause/resume   %sQ%s = quit\n",
    ansi::BOLD, ansi::CYAN,
    mode_col, mode_str, ansi::CYAN,
    ansi::RESET,
    ansi::BOLD, ansi::RESET,
    ansi::BOLD, ansi::RESET);
  std::printf("Services: %s/scan_sequence_node/start%s   %s/scan_sequence_node/pause%s   (std_srvs/srv/Trigger)\n",
    ansi::BOLD, ansi::RESET, ansi::BOLD, ansi::RESET);
  std::printf("Planner: %s%s%s   Pitch mode: %s%s%s\n",
    ansi::BOLD, planner_label.c_str(), ansi::RESET,
    ansi::BOLD,
    lock_pitch ? "LOCKED  (X horizontal, no rotation around Y_TCP)" : "FREE    (MoveIt picks pitch around Y_TCP)",
    ansi::RESET);
  if (!g_sequence_summary.empty()) {
    std::printf("%s%s%s\n", ansi::CYAN, g_sequence_summary.c_str(), ansi::RESET);
  }
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

// ============================================================================
// Candidati IK e seed per i rami (Step 3)
// ============================================================================
// Una configurazione valida che raggiunge un waypoint (o il suo fallback).
struct Candidate {
  std::vector<double> joints;          // giunti del gruppo, nell'ordine del JointModelGroup
  double pitch_offset_rad = 0.0;       // rotazione attorno a Y_TCP applicata alla posa base
  geometry_msgs::msg::Pose pose;       // posa effettivamente usata (con il pitch applicato)
  bool is_fallback = false;            // true se la posa e' un punto vicino, non il waypoint
};

// True se `joints` coincide (entro 1e-3 rad) con un candidato gia' in `list`.
static bool is_duplicate(const std::vector<double> & joints, const std::vector<Candidate> & list)
{
  for (const auto & c : list) {
    double max_diff = 0.0;
    for (size_t k = 0; k < joints.size() && k < c.joints.size(); ++k) {
      max_diff = std::max(max_diff, std::abs(joints[k] - c.joints[k]));
    }
    if (max_diff < 1e-3) return true;
  }
  return false;
}

// Posizione dei sei giunti UR dentro il JointModelGroup (-1 = non trovato).
struct UrJointIndex { int pan = -1, lift = -1, elbow = -1, w1 = -1, w2 = -1, w3 = -1; };

static UrJointIndex find_ur_joint_indices(const moveit::core::JointModelGroup * jmg)
{
  UrJointIndex idx;
  const auto & names = jmg->getActiveJointModelNames();
  for (size_t i = 0; i < names.size(); ++i) {
    const std::string & n = names[i];
    int k = static_cast<int>(i);
    if      (n.find("shoulder_pan")  != std::string::npos) idx.pan   = k;
    else if (n.find("shoulder_lift") != std::string::npos) idx.lift  = k;
    else if (n.find("elbow")         != std::string::npos) idx.elbow = k;
    else if (n.find("wrist_1")       != std::string::npos) idx.w1    = k;
    else if (n.find("wrist_2")       != std::string::npos) idx.w2    = k;
    else if (n.find("wrist_3")       != std::string::npos) idx.w3    = k;
  }
  return idx;
}

// Riporta un angolo in (-pi, pi]: cosi' i seed restano dentro i limiti UR (+-2pi).
static double wrap_pi(double a)
{
  while (a >  M_PI) a -= 2.0 * M_PI;
  while (a <= -M_PI) a += 2.0 * M_PI;
  return a;
}

// Otto seed per TRAC-IK a partire da una configurazione: tutte le combinazioni
// di polso (dritto/ribaltato), gomito (su/giu') e spalla (sx/dx). Il polso
// ribaltato e' esatto; gomito e spalla sono approssimazioni geometriche.
// Servono solo a far partire il solver vicino a un ramo diverso: e' poi
// TRAC-IK a convergere sulla soluzione esatta di quel ramo. Se in futuro
// servono tutti gli 8 rami con certezza, sostituire con l'IK analitica UR.
static std::vector<std::vector<double>> make_branch_seeds(
  const std::vector<double> & base, const UrJointIndex & j)
{
  std::vector<std::vector<double>> seeds;
  seeds.push_back(base);
  if (j.pan < 0 || j.lift < 0 || j.elbow < 0 || j.w1 < 0 || j.w2 < 0 || j.w3 < 0) return seeds;

  // Rapporto avambraccio/braccio (a3/a2), ~0.92 per tutta la famiglia UR.
  const double r = 0.92;

  for (int mask = 1; mask < 8; ++mask) {
    std::vector<double> q = base;
    if (mask & 1) {   // polso ribaltato
      q[j.w1] += M_PI;
      q[j.w2]  = -q[j.w2];
      q[j.w3] += M_PI;
    }
    if (mask & 2) {   // gomito specchiato rispetto alla retta spalla-polso
      double e     = q[j.elbow];
      double gamma = std::atan2(r * std::sin(e), 1.0 + r * std::cos(e));
      q[j.lift]  += 2.0 * gamma;
      q[j.elbow]  = -e;
      q[j.w1]    += 2.0 * e - 2.0 * gamma;
    }
    if (mask & 4) {   // spalla dall'altro lato: base a 180 gradi, braccio specchiato
      q[j.pan]  += M_PI;
      q[j.lift]  = -M_PI - q[j.lift];
      q[j.elbow] = -q[j.elbow];
      q[j.w1]    = -M_PI - q[j.w1];
      q[j.w2]    = -q[j.w2];
    }
    for (double & v : q) v = wrap_pi(v);
    seeds.push_back(q);
  }
  return seeds;
}

// ============================================================================
// Planning scene locale (Step 1)
// ============================================================================
// Copia locale della planning scene di move_group (oggetti del mondo + matrice
// delle collisioni permesse). Serve per scartare le soluzioni IK in collisione
// PRIMA di chiamare plan(): TRAC-IK non conosce la scena, e un goal in
// collisione fa fallire OMPL solo dopo aver consumato tutto il planning_time.
// La scena e' statica durante lo scan, quindi la leggiamo una volta sola.
static planning_scene::PlanningScenePtr fetch_planning_scene(
  rclcpp::Node::SharedPtr node,
  const moveit::core::RobotModelConstPtr & robot_model,
  const rclcpp::Logger & logger)
{
  using GetScene   = moveit_msgs::srv::GetPlanningScene;
  using Components = moveit_msgs::msg::PlanningSceneComponents;

  auto client = node->create_client<GetScene>("/get_planning_scene");
  if (!client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(logger,
      "Service /get_planning_scene non disponibile: check collisioni lato client DISABILITATO.");
    return nullptr;
  }

  auto request = std::make_shared<GetScene::Request>();
  request->components.components =
    Components::SCENE_SETTINGS |
    Components::ROBOT_STATE |
    Components::ROBOT_STATE_ATTACHED_OBJECTS |
    Components::WORLD_OBJECT_NAMES |
    Components::WORLD_OBJECT_GEOMETRY |
    Components::TRANSFORMS |
    Components::ALLOWED_COLLISION_MATRIX |
    Components::LINK_PADDING_AND_SCALING;

  // Il nodo e' gia' spinnato dal thread executor: basta aspettare il future.
  auto future = client->async_send_request(request);
  if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
    RCLCPP_ERROR(logger,
      "Timeout su /get_planning_scene: check collisioni lato client DISABILITATO.");
    return nullptr;
  }

  auto scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
  scene->setPlanningSceneMsg(future.get()->scene);
  RCLCPP_WARN(logger, "Planning scene locale caricata: %zu oggetti nel mondo.",
              scene->getWorld()->size());
  return scene;
}

// ============================================================================
// Catena di planner (Step 2)
// ============================================================================
struct PlannerChoice {
  std::string pipeline;   // es. "pilz_industrial_motion_planner" oppure "ompl"
  std::string planner;    // es. "PTP" oppure "RRTConnectkConfigDefault"
  bool is_circ = false;   // Pilz CIRC: vuole il vincolo "center" e uno start sulla sfera
  std::string label() const { return pipeline + "/" + planner; }
};

struct PlannerSetup {
  std::vector<PlannerChoice> chain;           // provati in ordine, ci si ferma al primo che riesce
  moveit_msgs::msg::Constraints circ_center;  // vincolo di percorso richiesto da CIRC
};

// Quante volte ogni planner ha prodotto il piano poi eseguito. E' la metrica con
// cui si confrontano i run: l'obiettivo e' avere quasi tutto su CIRC/PTP e OMPL
// vicino a zero. Include anche le recovery e il ritorno a home.
static std::map<std::string, int> g_planner_hits;

// Traduce la stringa del YAML ("ompl", "pilz_ptp", "pilz_lin", "pilz_circ",
// "stomp") in (pipeline, planner_id). Ritorna false se non e' riconosciuta.
static bool resolve_planner(const std::string & name, const std::string & ompl_algorithm,
                            PlannerChoice & out)
{
  if (name == "ompl") {
    out = {"ompl", ompl_algorithm + "kConfigDefault"};
  } else if (name == "pilz_ptp") {
    out = {"pilz_industrial_motion_planner", "PTP"};
  } else if (name == "pilz_lin") {
    out = {"pilz_industrial_motion_planner", "LIN"};
  } else if (name == "pilz_circ") {
    out = {"pilz_industrial_motion_planner", "CIRC", true};
  } else if (name == "stomp") {
    out = {"stomp", "stomp"};
  } else {
    return false;
  }
  return true;
}

// Vincolo di percorso che Pilz CIRC usa come centro dell'arco: nome "center",
// una sola PositionConstraint sul link del TCP, il centro in primitive_poses.
// La constraint_region resta di proposito SENZA primitives: Pilz legge solo la
// posizione, mentre il response adapter ValidateSolution ricontrolla il percorso
// anche contro req.path_constraints. Con una regione vera (il TCP "dentro" il
// centro della sfera) ogni arco verrebbe scartato; senza regione il vincolo
// risulta disabilitato lato MoveIt e la validazione lo ignora.
static moveit_msgs::msg::Constraints make_circ_center_constraint(
  const std::string & frame, const std::string & link, const Eigen::Vector3d & center)
{
  moveit_msgs::msg::Constraints constraints;
  constraints.name = "center";

  moveit_msgs::msg::PositionConstraint pc;
  pc.header.frame_id = frame;
  pc.link_name = link;
  pc.weight = 1.0;

  geometry_msgs::msg::Pose center_pose;
  center_pose.position.x = center.x();
  center_pose.position.y = center.y();
  center_pose.position.z = center.z();
  center_pose.orientation.w = 1.0;
  pc.constraint_region.primitive_poses.push_back(center_pose);

  constraints.position_constraints.push_back(pc);
  return constraints;
}

// Nome leggibile del codice d'errore di plan(). I codici sono quelli di
// moveit_msgs/MoveItErrorCodes; quelli non elencati escono come numero.
static std::string plan_error_name(int code)
{
  using Codes = moveit_msgs::msg::MoveItErrorCodes;
  switch (code) {
    case Codes::FAILURE:                              return "FAILURE (generico: vedi il terminale del bringup)";
    case Codes::PLANNING_FAILED:                      return "PLANNING_FAILED";
    case Codes::INVALID_MOTION_PLAN:                  return "INVALID_MOTION_PLAN (scartato da ValidateSolution; con CIRC anche raggio o piano dell'arco non validi)";
    case Codes::TIMED_OUT:                            return "TIMED_OUT";
    case Codes::START_STATE_IN_COLLISION:             return "START_STATE_IN_COLLISION";
    case Codes::START_STATE_VIOLATES_PATH_CONSTRAINTS: return "START_STATE_VIOLATES_PATH_CONSTRAINTS";
    case Codes::GOAL_IN_COLLISION:                    return "GOAL_IN_COLLISION";
    case Codes::GOAL_VIOLATES_PATH_CONSTRAINTS:       return "GOAL_VIOLATES_PATH_CONSTRAINTS";
    case Codes::GOAL_CONSTRAINTS_VIOLATED:            return "GOAL_CONSTRAINTS_VIOLATED";
    case Codes::INVALID_GOAL_CONSTRAINTS:             return "INVALID_GOAL_CONSTRAINTS";
    case Codes::INVALID_ROBOT_STATE:                  return "INVALID_ROBOT_STATE";
    default:                                          return "codice " + std::to_string(code);
  }
}

// True se l'ultimo punto della traiettoria coincide con il target in giunti.
// Serve per CIRC: Pilz riceve il goal in joint space ma ricava i giunti lungo
// l'arco con l'IK, seme = campione precedente, e puo' finire su un ramo IK
// diverso da quello scelto dalla DP. In quel caso il tratto successivo
// partirebbe da uno stato non previsto, quindi il piano va scartato.
// ponytail: soglia fissa a 0.01 rad, i rami IK distano decimi di radiante;
// portarla in configurazione solo se comparissero scarti ingiustificati.
static bool trajectory_ends_at_target(
  const moveit::planning_interface::MoveGroupInterface::Plan & plan,
  const moveit::planning_interface::MoveGroupInterface & move_group)
{
  const auto & jt = plan.trajectory.joint_trajectory;
  if (jt.points.empty()) return false;

  const auto & last = jt.points.back().positions;
  if (last.size() != jt.joint_names.size()) return false;

  const moveit::core::JointModelGroup * jmg =
    move_group.getRobotModel()->getJointModelGroup(move_group.getName());
  if (jmg == nullptr) return false;

  // Il target impostato dal chiamante, associato per nome: l'ordine dei giunti
  // nella traiettoria non e' per forza quello delle variabili del gruppo.
  std::vector<double> target_values;
  move_group.getJointValueTarget(target_values);
  const std::vector<std::string> & target_names = jmg->getVariableNames();
  if (target_values.size() != target_names.size()) return false;

  std::map<std::string, double> target;
  for (size_t k = 0; k < target_names.size(); ++k) {
    target[target_names[k]] = target_values[k];
  }

  for (size_t k = 0; k < jt.joint_names.size(); ++k) {
    auto it = target.find(jt.joint_names[k]);
    if (it == target.end()) continue;   // giunto fuori dal gruppo: ignorato
    if (std::abs(wrap_pi(last[k] - it->second)) > 0.01) return false;
  }
  return true;
}

// plan() lungo la catena di planner: si prova un planner alla volta nell'ordine
// della catena e ci si ferma al primo che riesce. Il target (joint / pose /
// named) va impostato PRIMA dal chiamante.
// `allow_circ` = false salta Pilz CIRC: senza uno start gia' sulla sfera di
// scansione i due estremi dell'arco hanno raggi diversi e CIRC fallisce sempre
// (e' il caso di home e delle pose di recovery).
static bool plan_with_fallback(
  moveit::planning_interface::MoveGroupInterface & move_group,
  moveit::planning_interface::MoveGroupInterface::Plan & plan,
  const PlannerSetup & planners,
  bool allow_circ,
  const rclcpp::Logger & logger)
{
  for (const PlannerChoice & choice : planners.chain) {
    if (choice.is_circ && !allow_circ) continue;

    move_group.setPlanningPipelineId(choice.pipeline);
    move_group.setPlannerId(choice.planner);
    // Il vincolo "center" serve solo a CIRC: gli altri planner devono partire
    // senza path constraints, quindi si mette qui e si toglie subito dopo,
    // riuscito o fallito che sia il tentativo.
    if (choice.is_circ) move_group.setPathConstraints(planners.circ_center);

    moveit::core::MoveItErrorCode code = move_group.plan(plan);
    if (choice.is_circ) move_group.clearPathConstraints();

    if (code != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_WARN(logger, "plan con %s fallito: %s",
                  choice.label().c_str(), plan_error_name(code.val).c_str());
      continue;
    }
    if (choice.is_circ && !trajectory_ends_at_target(plan, move_group)) {
      RCLCPP_WARN(logger, "plan con %s scartato: l'arco termina su un ramo IK diverso dal target",
                  choice.label().c_str());
      continue;
    }

    ++g_planner_hits[choice.label()];
    return true;
  }
  return false;
}

static bool go_home(moveit::planning_interface::MoveGroupInterface & move_group,
                    const PlannerSetup & planners,
                    const std::string & home_pose_name,
                    const rclcpp::Logger & logger)
{
  RCLCPP_WARN(logger, "Recovery: moving back to '%s' ...", home_pose_name.c_str());

  // Drop any active path constraint (e.g. the look-at orientation constraint
  // used in free-pitch mode) — the home pose cannot satisfy it.
  move_group.clearPathConstraints();
  move_group.setNamedTarget(home_pose_name);

  moveit::planning_interface::MoveGroupInterface::Plan home_plan;
  // CIRC escluso: home e le pose di recovery non stanno sulla sfera di scansione.
  if (!plan_with_fallback(move_group, home_plan, planners, false, logger)) {
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

// Recovery a due livelli: prima la posa dell'emisfero corrente, poi `home`.
// Da certe configurazioni basse una delle due non si pianifica (es. la retta
// PTP passa per un'autocollisione e OMPL sfiora il disco), l'altra spesso si'.
static bool recover_to_safe_pose(moveit::planning_interface::MoveGroupInterface & move_group,
                                 const PlannerSetup & planners,
                                 const std::string & recovery_pose_name,
                                 const std::string & home_pose_name,
                                 const rclcpp::Logger & logger)
{
  if (go_home(move_group, planners, recovery_pose_name, logger)) return true;
  if (recovery_pose_name == home_pose_name) return false;
  RCLCPP_WARN(logger, "Recovery verso '%s' fallita, provo '%s'.",
              recovery_pose_name.c_str(), home_pose_name.c_str());
  return go_home(move_group, planners, home_pose_name, logger);
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
  auto node   = rclcpp::Node::make_shared("scan_sequence_node");
  auto logger = node->get_logger();

  // Quiet down ROS/MoveIt info-spam so the table layout stays readable.
  // WARN/ERROR/FATAL still pass through.
  rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_WARN);

  // --- Service interface: /scan_sequence/start e /scan_sequence/pause ---
  // Lo scan parte sempre in pausa. Per farlo partire (o riprendere dopo una pausa)
  //   ros2 service call /scan_sequence/start std_srvs/srv/Trigger {}
  // Per metterlo in pausa mid-scan
  //   ros2 service call /scan_sequence/pause std_srvs/srv/Trigger {}
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
  // Catena di planner, provati in ordine. Il default riproduce il comportamento
  // storico: retta in joint space, OMPL quando la retta collide.
  node->declare_parameter<std::vector<std::string>>("scan_planners",
                                                    std::vector<std::string>{"pilz_ptp", "ompl"});
  node->declare_parameter<std::string>("scan_ompl_algorithm",         "RRTConnect");
  node->declare_parameter<double>     ("scan_pitch_search_range_deg", 90.0);
  node->declare_parameter<double>     ("scan_pitch_search_step_deg",  15.0);
  node->declare_parameter<double>     ("scan_pitch_xparallel_bias",    0.05);
  node->declare_parameter<double>     ("scan_ik_timeout",              0.2);
  node->declare_parameter<double>     ("scan_enum_ik_timeout",         0.003);
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
  const std::vector<std::string> planner_names = node->get_parameter("scan_planners").as_string_array();
  const std::string ompl_algorithm  = node->get_parameter("scan_ompl_algorithm").as_string();
  const double      pitch_range_deg = node->get_parameter("scan_pitch_search_range_deg").as_double();
  const double      pitch_step_deg  = node->get_parameter("scan_pitch_search_step_deg").as_double();
  const double      pitch_bias      = node->get_parameter("scan_pitch_xparallel_bias").as_double();
  const double      enum_ik_timeout = node->get_parameter("scan_enum_ik_timeout").as_double();
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
  // In modalita' FULL li generiamo come due chiamate separate (upper + lower) e
  // teniamo l'indice di confine: da li' in poi cambiano la pose di recovery e
  // il check di occlusione. La transizione vera e propria la decide la DP.
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

  // Traduce la lista `scan_planners` del YAML in una catena ordinata di
  // (pipeline, planner_id): ogni tratto la percorre e si ferma al primo planner
  // che riesce. Vedi plan_with_fallback().
  PlannerSetup planners;
  for (const std::string & name : planner_names) {
    PlannerChoice choice;
    if (resolve_planner(name, ompl_algorithm, choice)) {
      planners.chain.push_back(choice);
    } else {
      std::fprintf(stderr, "WARNING: planner '%s' sconosciuto, ignorato.\n", name.c_str());
    }
  }
  if (planners.chain.empty()) {
    std::fprintf(stderr, "Nessun planner valido in `planners`: controllare automata_config.yaml.\n");
    rclcpp::shutdown();
    return 1;
  }
  planners.circ_center = make_circ_center_constraint(global_frame, ee_link, cfg.center);

  std::string planner_label;
  for (size_t k = 0; k < planners.chain.size(); ++k) {
    if (k > 0) planner_label += " > ";
    planner_label += planners.chain[k].label();
  }

  // Copia locale della planning scene per il check collisioni dei candidati IK.
  planning_scene::PlanningScenePtr scene =
    fetch_planning_scene(node, move_group.getRobotModel(), logger);

  // (lo spinner che processa anche i service /start e /pause e' gia' attivo, vedi sopra)

  int successes      = 0;
  int failures       = 0;
  int ik_failures    = 0;   // nessuna configurazione valida: waypoint irraggiungibile
  int plan_failures  = 0;   // configurazione valida ma nessun planner ha trovato il percorso
  int exec_failures  = 0;   // plan ok ma il controller ha rifiutato l'esecuzione

  // Pose di recovery: home per l'emisfero superiore, lower_scan_ready per
  // l'inferiore. In FULL viene aggiornata quando si passa sotto l'equatore.
  std::string recovery_pose_name =
    (cfg.hemisphere == HEMISPHERE_LOWER) ? lower_home_pose_name : home_pose_name;

  // Filtro comune dei candidati IK:
  //   1) collisione con la scena / auto-collisione (planning scene locale)
  //   2) occlusione camera->centro (solo emisfero inferiore)
  // Ritorna true se il candidato e' accettabile.
  int collision_rejects = 0;
  int occlusion_rejects = 0;
  int scene_occluded    = 0;   // waypoint scartati: oggetto della scena fra camera e centro
  const auto * jmg = move_group.getRobotModel()->getJointModelGroup(planning_group);
  auto candidate_ok = [&](const moveit::core::RobotState & st, size_t wp_index) -> bool {
    if (scene && scene->isStateColliding(st, planning_group)) {
      ++collision_rejects;
      return false;
    }
    if (occlusion_check && lower_start_index > 0 && wp_index >= lower_start_index &&
        is_arm_occluding(st, jmg, ee_link, cfg.center, occlusion_threshold_rad)) {
      ++occlusion_rejects;
      return false;
    }
    return true;
  };

  // ==========================================================================
  // FASE 1 - enumerazione offline. Per ogni waypoint si cercano TUTTE le
  // configurazioni valide (pitch x rami IK) senza muovere il robot.
  // ==========================================================================
  moveit::core::RobotState work_state(*move_group.getCurrentState());
  std::vector<double> start_joints;
  work_state.copyJointGroupPositions(jmg, start_joints);
  const UrJointIndex ur_idx = find_ur_joint_indices(jmg);

  // Con lock_pitch il pitch e' fisso: un solo offset (0).
  std::vector<double> enum_pitch_offsets = pitch_offsets;
  if (lock_pitch) enum_pitch_offsets = {0.0};
  // Nel fallback bastano gli offset piu' piccoli (0, +-step, +-2step, +-3step).
  std::vector<double> fb_pitch_offsets(
    enum_pitch_offsets.begin(),
    enum_pitch_offsets.begin() + std::min<size_t>(7, enum_pitch_offsets.size()));

  // Enumera le configurazioni valide per una posa e le aggiunge a `out`.
  // Per ogni pitch offset e per ogni seed chiama TRAC-IK (solve_type Speed:
  // converge sul ramo piu' vicino al seed), poi filtra per collisioni /
  // occlusione e scarta i duplicati.
  auto enumerate_pose = [&](const geometry_msgs::msg::Pose & base_pose, size_t wp_index,
                            bool is_fallback,
                            const std::vector<std::vector<double>> & seeds,
                            const std::vector<double> & pitch_list,
                            std::vector<Candidate> & out,
                            double ik_timeout_s)
  {
    Eigen::Quaterniond q_base(base_pose.orientation.w, base_pose.orientation.x,
                              base_pose.orientation.y, base_pose.orientation.z);
    for (double offset : pitch_list) {
      Eigen::Quaterniond q_try =
        q_base * Eigen::Quaterniond(Eigen::AngleAxisd(offset, Eigen::Vector3d::UnitY()));
      geometry_msgs::msg::Pose pose_try = base_pose;
      pose_try.orientation.w = q_try.w();
      pose_try.orientation.x = q_try.x();
      pose_try.orientation.y = q_try.y();
      pose_try.orientation.z = q_try.z();

      for (const auto & seed : seeds) {
        moveit::core::RobotState cand(work_state);
        cand.setJointGroupPositions(jmg, seed);
        cand.enforceBounds();
        if (!cand.setFromIK(jmg, pose_try, ee_link, ik_timeout_s)) continue;
        cand.update();
        if (!candidate_ok(cand, wp_index)) continue;

        Candidate c;
        cand.copyJointGroupPositions(jmg, c.joints);
        c.pitch_offset_rad = offset;
        c.pose = pose_try;
        c.is_fallback = is_fallback;
        if (is_duplicate(c.joints, out)) continue;
        out.push_back(c);
      }
    }
  };

  std::vector<std::vector<Candidate>> layers(waypoints.size());
  const std::vector<std::vector<double>> home_seeds = make_branch_seeds(start_joints, ur_idx);
  std::vector<std::vector<double>> prev_seeds;   // soluzioni del waypoint precedente
  size_t total_candidates = 0;
  auto enum_t0 = std::chrono::steady_clock::now();

  std::printf("Enumerazione candidati (%zu waypoint, %zu pitch x max 8 seed) ...\n",
              waypoints.size(), enum_pitch_offsets.size());

  for (size_t i = 0; i < waypoints.size(); ++i) {
    if (g_quit.load() || !rclcpp::ok()) break;

    // Seed: le soluzioni del waypoint precedente (vicino nello spazio e gia'
    // una per ramo); quelle dalla home solo finche' non c'e' un precedente.
    const std::vector<std::vector<double>> & seeds = prev_seeds.empty() ? home_seeds : prev_seeds;

    // Linea di vista: se fra il waypoint e il centro c'e' un oggetto della
    // scena la foto inquadrerebbe l'ostacolo. Scartato subito, senza IK ne'
    // fallback.
    if (scene) {
      Eigen::Vector3d wp_pos(waypoints[i].position.x, waypoints[i].position.y, waypoints[i].position.z);
      std::string hit_object;
      if (is_scene_occluding(*scene, wp_pos, cfg.center, hit_object)) {
        ++scene_occluded;
        rows[i].status = WpStatus::OCCLUDED;
        set_marker_color(markers_array, i, COLOR_ORANGE);
        double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - enum_t0).count();
        std::printf("  wp %3zu/%zu: vista coperta da '%s'  <-- SCARTATO   (%.1f s)\n",
                    i + 1, waypoints.size(), hit_object.c_str(), elapsed);
        std::fflush(stdout);
        continue;
      }
    }

    enumerate_pose(waypoints[i], i, false, seeds, enum_pitch_offsets, layers[i], enum_ik_timeout);

    // Vicino al limite dell'inviluppo (braccio quasi disteso) il solver
    // numerico puo' non trovare in 3 ms soluzioni che esistono: prima di
    // dichiarare il punto irraggiungibile ritento con un timeout 20x.
    const double slow_ik_timeout = enum_ik_timeout * 20.0;
    if (layers[i].empty()) {
      enumerate_pose(waypoints[i], i, false, seeds, enum_pitch_offsets, layers[i], slow_ik_timeout);
    }

    if (layers[i].empty() && fallback_search) {
      // Nessuna configurazione valida: cerco un punto vicino sulla sfera
      // (2 anelli x 6 direzioni entro fallback_radius_mm), sempre offline.
      double fallback_angle_max = (fallback_radius_mm / 1000.0) / cfg.radius;
      Eigen::Vector3d wp_pos(waypoints[i].position.x, waypoints[i].position.y, waypoints[i].position.z);
      Eigen::Vector3d radial = (wp_pos - cfg.center).normalized();
      Eigen::Vector3d ref_ax = (std::abs(radial.dot(Eigen::Vector3d::UnitZ())) < 0.9)
                               ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitX();
      Eigen::Vector3d tang_u = radial.cross(ref_ax).normalized();
      Eigen::Vector3d tang_v = radial.cross(tang_u);

      for (int ring = 1; ring <= 2; ++ring) {
        double angle_off = fallback_angle_max * ring / 2.0;
        for (int d = 0; d < 6; ++d) {
          double dir_angle = d * (2.0 * M_PI / 6.0);
          Eigen::Vector3d tangent = std::cos(dir_angle) * tang_u + std::sin(dir_angle) * tang_v;
          Eigen::Vector3d new_radial =
            (std::cos(angle_off) * radial + std::sin(angle_off) * tangent).normalized();
          Eigen::Vector3d cand_pos = cfg.center + cfg.radius * new_radial;
          std::string hit_object;
          if (scene && is_scene_occluding(*scene, cand_pos, cfg.center, hit_object)) continue;
          enumerate_pose(make_lookat(cand_pos, cfg.center), i, true, seeds, fb_pitch_offsets, layers[i], slow_ik_timeout);
        }
      }
    }

    if (layers[i].empty()) {
      // Diagnosi: nessuna IK (fuori portata / orientamento) oppure tutte le IK
      // in collisione o occluse? E con quali oggetti? Stessa ricerca
      // dell'enumerazione (con il timeout lungo), qui si contano i motivi.
      int ik_found = 0, colliding = 0, occluded = 0;
      std::map<std::string, int> contact_pairs;
      Eigen::Quaterniond q_base(waypoints[i].orientation.w, waypoints[i].orientation.x,
                                waypoints[i].orientation.y, waypoints[i].orientation.z);
      for (double offset : enum_pitch_offsets) {
        Eigen::Quaterniond q_try =
          q_base * Eigen::Quaterniond(Eigen::AngleAxisd(offset, Eigen::Vector3d::UnitY()));
        geometry_msgs::msg::Pose pose_try = waypoints[i];
        pose_try.orientation.w = q_try.w();
        pose_try.orientation.x = q_try.x();
        pose_try.orientation.y = q_try.y();
        pose_try.orientation.z = q_try.z();
        for (const auto & seed : seeds) {
          moveit::core::RobotState cand(work_state);
          cand.setJointGroupPositions(jmg, seed);
          cand.enforceBounds();
          if (!cand.setFromIK(jmg, pose_try, ee_link, slow_ik_timeout)) continue;
          ++ik_found;
          cand.update();
          if (scene) {
            collision_detection::CollisionRequest req;
            collision_detection::CollisionResult res;
            req.contacts     = true;
            req.max_contacts = 10;
            req.group_name   = planning_group;
            scene->checkCollision(req, res, cand);
            if (res.collision) {
              ++colliding;
              for (const auto & kv : res.contacts) {
                contact_pairs[kv.first.first + " <-> " + kv.first.second] += 1;
              }
              continue;
            }
          }
          if (occlusion_check && lower_start_index > 0 && i >= lower_start_index &&
              is_arm_occluding(cand, jmg, ee_link, cfg.center, occlusion_threshold_rad)) {
            ++occluded;
          }
        }
      }
      Eigen::Vector3d base_pos = work_state.getGlobalLinkTransform("base_link").translation();
      Eigen::Vector3d wp_pos(waypoints[i].position.x, waypoints[i].position.y, waypoints[i].position.z);
      std::printf("     diagnosi wp %zu: distanza TCP-base %.3f m | IK trovate %d | in collisione %d | occluse %d\n",
                  i, (wp_pos - base_pos).norm(), ik_found, colliding, occluded);
      for (const auto & kv : contact_pairs) {
        std::printf("       contatto %-45s x%d\n", kv.first.c_str(), kv.second);
      }
    } else {
      prev_seeds.clear();
      for (size_t k = 0; k < layers[i].size() && k < 8; ++k) prev_seeds.push_back(layers[i][k].joints);
    }
    total_candidates += layers[i].size();

    double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - enum_t0).count();
    std::printf("  wp %3zu/%zu: %3zu candidati%s   (%.1f s)\n",
                i + 1, waypoints.size(), layers[i].size(),
                layers[i].empty() ? "  <-- IRRAGGIUNGIBILE" : "", elapsed);
    std::fflush(stdout);
  }
  const double enum_seconds =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - enum_t0).count();

  // ==========================================================================
  // FASE 2 - scelta della sequenza: DP a strati, un candidato per waypoint,
  // minimizzando il percorso totale nei giunti (+ penalita' sul pitch).
  // ==========================================================================
  std::vector<std::vector<SeqCandidate>> dp_layers(waypoints.size());
  for (size_t i = 0; i < waypoints.size(); ++i) {
    for (const auto & c : layers[i]) {
      SeqCandidate sc;
      sc.joints     = c.joints;
      sc.extra_cost = pitch_bias * std::abs(c.pitch_offset_rad);
      dp_layers[i].push_back(sc);
    }
  }
  // Validazione lazy dei tratti. Controllare tutti gli archi costerebbe minuti;
  // si controllano solo quelli della catena scelta (retta in joint space, come
  // Pilz PTP): i tratti che attraversano un ostacolo vengono penalizzati e la
  // DP viene rifatta, finche' la catena e' pulita o si esauriscono le iterazioni.
  auto segment_is_free = [&](const std::vector<double> & a, const std::vector<double> & b) -> bool {
    if (!scene) return true;
    double max_diff = 0.0;
    for (size_t k = 0; k < a.size() && k < b.size(); ++k) {
      max_diff = std::max(max_diff, std::abs(a[k] - b[k]));
    }
    int steps = std::max(1, static_cast<int>(std::ceil(max_diff / 0.05)));   // un campione ogni ~3 gradi
    moveit::core::RobotState from(work_state), to(work_state), mid(work_state);
    from.setJointGroupPositions(jmg, a);
    to.setJointGroupPositions(jmg, b);
    for (int s = 1; s < steps; ++s) {
      from.interpolate(to, static_cast<double>(s) / steps, mid, jmg);
      mid.update();
      if (scene->isStateColliding(mid, planning_group)) return false;
    }
    return true;
  };

  // Cache dei tratti gia' controllati: true = libero, false = in collisione.
  // Un tratto mai controllato e' considerato libero (ottimismo), poi viene
  // verificato solo se la DP lo sceglie.
  std::map<std::tuple<int, int, int, int>, bool> edge_cache;   // (prev_layer, prev_c, layer, c)
  auto edge_ok = [&](int prev_layer, int prev_c, int layer, int c) -> bool {
    auto it = edge_cache.find(std::make_tuple(prev_layer, prev_c, layer, c));
    return (it == edge_cache.end()) ? true : it->second;
  };
  auto check_edge = [&](int prev_layer, int prev_c, int layer, int c,
                        const std::vector<double> & a, const std::vector<double> & b) -> bool {
    auto key = std::make_tuple(prev_layer, prev_c, layer, c);
    auto it = edge_cache.find(key);
    if (it != edge_cache.end()) return it->second;
    bool is_free = segment_is_free(a, b);
    edge_cache[key] = is_free;
    return is_free;
  };

  SeqResult seq;
  int dp_iterations = 0;
  auto dp_t0 = std::chrono::steady_clock::now();
  for (int iter = 0; iter < 200; ++iter) {
    seq = choose_sequence(start_joints, dp_layers, edge_ok);
    ++dp_iterations;

    const size_t cache_before = edge_cache.size();
    int new_blocked = 0;
    int prev_layer = -1;
    int prev_c = 0;
    const std::vector<double> * prev_joints = &start_joints;
    for (size_t i = 0; i < waypoints.size(); ++i) {
      if (seq.chosen[i] < 0) continue;
      const int c = seq.chosen[i];
      if (!check_edge(prev_layer, prev_c, static_cast<int>(i), c, *prev_joints, layers[i][c].joints)) {
        ++new_blocked;
        // Tratto bloccato: controllo subito anche gli altri candidati di questo
        // layer dallo stesso punto di partenza, cosi' alla prossima iterazione
        // la DP sa gia' quali alternative sono libere invece di provarle una
        // per volta.
        for (size_t c2 = 0; c2 < layers[i].size(); ++c2) {
          check_edge(prev_layer, prev_c, static_cast<int>(i), static_cast<int>(c2),
                     *prev_joints, layers[i][c2].joints);
        }
      }
      prev_layer  = static_cast<int>(i);
      prev_c      = c;
      prev_joints = &layers[i][c].joints;
    }
    std::printf("  DP iterazione %d: %d tratti in collisione sulla catena (%zu tratti controllati finora)\n",
                iter + 1, new_blocked, edge_cache.size());
    std::fflush(stdout);
    if (new_blocked == 0) break;
    // Nessun tratto nuovo controllato: tutte le alternative dei layer bloccati
    // sono gia' in cache e sono tutte in collisione. Un'altra iterazione
    // darebbe la stessa catena; il tratto rimasto lo gestisce il planner.
    if (edge_cache.size() == cache_before) {
      std::printf("  DP: %d tratti bloccati senza alternative libere, li lascio al planner.\n",
                  new_blocked);
      break;
    }
  }
  size_t blocked_edges_total = 0;
  for (const auto & kv : edge_cache) {
    if (!kv.second) ++blocked_edges_total;
  }
  const double dp_seconds =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - dp_t0).count();

  size_t reachable = 0, via_fallback = 0;
  for (size_t i = 0; i < waypoints.size(); ++i) {
    if (seq.chosen[i] < 0) {
      if (rows[i].status != WpStatus::OCCLUDED) {   // gia' marcato in fase 1
        rows[i].status = WpStatus::UNREACHABLE;
        set_marker_color(markers_array, i, COLOR_RED);
      }
      continue;
    }
    ++reachable;
    const Candidate & c = layers[i][seq.chosen[i]];
    if (c.is_fallback) {
      ++via_fallback;
      rows[i].fallback     = c.pose;
      rows[i].fallback_set = true;
      set_marker_color(markers_array, i, COLOR_GRAY);
      set_marker_small(markers_array, i);
      int fb_id = static_cast<int>(waypoints.size()) * 2 + static_cast<int>(i) * 2;
      Marker fb_sphere = make_sphere_marker(fb_id,     c.pose, global_frame, COLOR_BLUE);
      Marker fb_arrow  = make_arrow_marker (fb_id + 1, c.pose, cfg.center,   global_frame, COLOR_BLUE);
      fb_sphere.ns = "fallback_waypoints";
      fb_arrow.ns  = "fallback_orientations";
      markers_array.markers.push_back(fb_sphere);
      markers_array.markers.push_back(fb_arrow);
    }
  }
  publish_markers(markers_pub, markers_array);

  {
    char buf[384];
    std::snprintf(buf, sizeof(buf),
      "Sequenza: %zu/%zu raggiungibili (%zu via fallback), %zu irraggiungibili, %d vista coperta | "
      "%zu candidati, %d scartati per collisione, %d per occlusione | costo DP %.1f, %zu tratti bloccati su %zu controllati, %d iter | "
      "enumerazione %.1f s, DP %.1f s",
      reachable, waypoints.size(), via_fallback, waypoints.size() - reachable - scene_occluded, scene_occluded,
      total_candidates, collision_rejects, occlusion_rejects, seq.total_cost, blocked_edges_total, edge_cache.size(),
      dp_iterations, enum_seconds, dp_seconds);
    g_sequence_summary = buf;
  }

  // --- Interactive terminal: spacebar pause/resume + Q to quit ---
  setup_raw_terminal();
  std::printf("%s", ansi::HIDE_CURSOR);
  std::thread key_thread(key_reader_loop);

  // Primo render: tabella con la sequenza pianificata, in attesa di SPACE o
  // del service /start. Da qui in poi il robot si muove.
  render_table(rows, lock_pitch, planner_label);
  wait_while_paused(rows, lock_pitch, planner_label);

  // ==========================================================================
  // FASE 3 - esecuzione della sequenza scelta.
  // ==========================================================================
  auto scan_start_time = std::chrono::steady_clock::now();

  // True quando il robot e' fermo su un waypoint della sfera di scansione: solo
  // da li' Pilz CIRC puo' pianificare l'arco (start e goal allo stesso raggio).
  bool on_sphere = false;

  for (size_t i = 0; i < waypoints.size(); ++i) {
    if (g_quit.load() || !rclcpp::ok()) break;
    wait_while_paused(rows, lock_pitch, planner_label);
    if (g_quit.load() || !rclcpp::ok()) break;

    // Sotto l'equatore cambia la pose di recovery.
    if (lower_start_index > 0 && i == lower_start_index) {
      recovery_pose_name = lower_home_pose_name;
    }

    if (seq.chosen[i] < 0) {   // gia' marcato OCCLUDED (fase 1) o UNREACHABLE (fase 2)
      if (rows[i].status != WpStatus::OCCLUDED) {
        ++failures;
        ++ik_failures;
      }
      continue;
    }
    const Candidate & chosen = layers[i][seq.chosen[i]];

    rows[i].status = WpStatus::RUNNING;
    set_marker_color(markers_array, i, COLOR_YELLOW);
    publish_markers(markers_pub, markers_array);
    render_table(rows, lock_pitch, planner_label);

    work_state.setJointGroupPositions(jmg, chosen.joints);
    move_group.clearPathConstraints();
    move_group.clearPoseTargets();
    move_group.setJointValueTarget(work_state);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool plan_ok = plan_with_fallback(move_group, plan, planners, on_sphere, logger);

    if (!plan_ok) {
      // Nessun percorso da dove siamo: passo dalla pose di recovery e riprovo
      // una volta verso la STESSA configurazione.
      rows[i].status = WpStatus::HOMING;
      render_table(rows, lock_pitch, planner_label);
      if (!recover_to_safe_pose(move_group, planners, recovery_pose_name, home_pose_name, logger)) {
        // Neanche la recovery pianifica: quasi sempre stato di partenza
        // invalido (robot in collisione / fuori limiti). Continuare farebbe
        // solo fallire tutti i waypoint restanti: ci si ferma qui.
        RCLCPP_ERROR(logger,
          "wp %zu: recovery impossibile dallo stato attuale. SCAN INTERROTTO: "
          "riportare il robot in una posa valida e rilanciare.", i);
        rows[i].status = WpStatus::FAIL;
        ++failures; ++plan_failures;
        render_table(rows, lock_pitch, planner_label);
        break;
      }
      on_sphere = false;   // dopo la recovery il TCP non e' piu' su un waypoint
      rows[i].status = WpStatus::RUNNING;
      render_table(rows, lock_pitch, planner_label);
      move_group.setJointValueTarget(work_state);
      plan_ok = plan_with_fallback(move_group, plan, planners, on_sphere, logger);
    }

    if (!plan_ok) {
      ++failures;
      ++plan_failures;
      rows[i].status = WpStatus::FAIL;
      set_marker_color(markers_array, i, COLOR_RED);
      publish_markers(markers_pub, markers_array);
      render_table(rows, lock_pitch, planner_label);
      continue;
    }

    moveit::core::MoveItErrorCode exec_code = move_group.execute(plan);
    if (exec_code != moveit::core::MoveItErrorCode::SUCCESS) {
      // execute() fallito: robot fermo a meta' traiettoria -> home per sicurezza.
      RCLCPP_ERROR(logger,
        "wp %zu: execute() FALLITO con codice MoveIt = %d (%s). "
        "Verifica il terminale del bringup per il messaggio del controller.",
        i, static_cast<int>(exec_code.val), exec_code.message.c_str());
      ++failures;
      ++exec_failures;
      on_sphere = false;   // fermo a meta' traiettoria: posizione non nota
      rows[i].status = WpStatus::HOMING;
      set_marker_color(markers_array, i, COLOR_RED);
      publish_markers(markers_pub, markers_array);
      render_table(rows, lock_pitch, planner_label);
      bool recovered = recover_to_safe_pose(move_group, planners, recovery_pose_name, home_pose_name, logger);
      rows[i].status = WpStatus::FAIL;
      render_table(rows, lock_pitch, planner_label);
      if (!recovered) {
        RCLCPP_ERROR(logger,
          "wp %zu: recovery impossibile dopo execute fallito. SCAN INTERROTTO: "
          "riportare il robot in una posa valida e rilanciare.", i);
        break;
      }
      continue;
    }

    // Raggiunto
    ++successes;
    on_sphere = true;
    rows[i].status = chosen.is_fallback ? WpStatus::FALLBACK_ORIGIN : WpStatus::DONE;

    auto current = move_group.getCurrentPose(ee_link);
    rows[i].actual = current.pose;
    rows[i].actual_set = true;
    double diff_deg = quat_angle_diff(rows[i].target.orientation, current.pose.orientation)
                      * 180.0 / M_PI;
    rows[i].reoriented = (!lock_pitch && diff_deg > 2.0);

    if (!chosen.is_fallback) set_marker_color(markers_array, i, COLOR_GREEN);
    publish_markers(markers_pub, markers_array);
    render_table(rows, lock_pitch, planner_label);
  }

  // Calcolo tempo totale di scansione (dal primo waypoint all'ultimo)
  auto scan_end_time = std::chrono::steady_clock::now();
  double scan_duration_s = std::chrono::duration<double>(scan_end_time - scan_start_time).count();

  // Ritorno a home a fine scansione (se non killato)
  if (!g_quit.load() && rclcpp::ok()) {
    RCLCPP_WARN(logger, "=== Scan completato, ritorno a '%s' ===", home_pose_name.c_str());
    go_home(move_group, planners, home_pose_name, logger);
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
  if (scene_occluded > 0) {
    std::printf("  Vista coperta: %d waypoint scartati.", scene_occluded);
  }
  std::printf("\n%sPlanner usati:%s", ansi::BOLD, ansi::RESET);
  if (g_planner_hits.empty()) {
    std::printf(" nessuno");
  } else {
    for (const auto & kv : g_planner_hits) {
      std::printf("  %s %d", kv.first.c_str(), kv.second);
    }
  }
  std::printf("\n%sCandidati IK scartati:%s %d per collisione, %d per occlusione",
    ansi::BOLD, ansi::RESET, collision_rejects, occlusion_rejects);
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
