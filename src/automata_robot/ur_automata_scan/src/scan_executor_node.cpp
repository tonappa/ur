#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include "ur_automata_scan/sphere_waypoint_generator.hpp"

using namespace std::chrono_literals;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

// ── ANSI colors per i log ────────────────────────────────────────────────────
namespace ansi {
  constexpr const char * GREEN  = "\033[1;32m";
  constexpr const char * YELLOW = "\033[1;33m";
  constexpr const char * RED    = "\033[1;31m";
  constexpr const char * RESET  = "\033[0m";
}

// ── Stati WP → colore RGB (stessa palette per terminale e marker) ────────────
struct Rgb { float r, g, b; };
namespace col {
  constexpr Rgb GRAY   {0.5f, 0.5f, 0.5f};   // not yet attempted
  constexpr Rgb YELLOW {1.0f, 0.85f, 0.0f};  // currently planning/executing
  constexpr Rgb GREEN  {0.0f, 1.0f, 0.0f};   // success
  constexpr Rgb RED    {1.0f, 0.0f, 0.0f};   // fail
}

// ── Helpers marker ────────────────────────────────────────────────────────────

static Marker make_sphere_marker(
  int id,
  const geometry_msgs::msg::Pose & pose,
  const std::string & frame,
  const Rgb & c)
{
  Marker m;
  m.header.frame_id = frame;
  m.ns = "scan_waypoints";
  m.id = id;
  m.type = Marker::SPHERE;
  m.action = Marker::ADD;
  m.pose = pose;
  m.scale.x = m.scale.y = m.scale.z = 0.025;  // 2.5 cm
  m.color.r = c.r;
  m.color.g = c.g;
  m.color.b = c.b;
  m.color.a = 0.85f;
  return m;
}

/// Freccia da WP verso il centro della sfera = asse Y del TCP (asse ottico camera).
static Marker make_arrow_marker(
  int id,
  const geometry_msgs::msg::Pose & pose,
  const Eigen::Vector3d & center,
  const std::string & frame,
  const Rgb & c)
{
  Marker m;
  m.header.frame_id = frame;
  m.ns = "scan_orientations";
  m.id = id;
  m.type = Marker::ARROW;
  m.action = Marker::ADD;

  const Eigen::Vector3d wp_pos(pose.position.x, pose.position.y, pose.position.z);
  const Eigen::Vector3d dir = (center - wp_pos).normalized();
  const double length = 0.06;  // 6 cm
  const Eigen::Vector3d tip = wp_pos + length * dir;

  geometry_msgs::msg::Point start, end;
  start.x = wp_pos.x(); start.y = wp_pos.y(); start.z = wp_pos.z();
  end.x   = tip.x();    end.y   = tip.y();    end.z   = tip.z();
  m.points.push_back(start);
  m.points.push_back(end);

  // ARROW con points[]: scale.x=shaft Ø, scale.y=head Ø, scale.z=head length
  m.scale.x = 0.005;
  m.scale.y = 0.012;
  m.scale.z = 0.015;

  m.color.r = c.r;
  m.color.g = c.g;
  m.color.b = c.b;
  m.color.a = 0.9f;
  return m;
}

// ── Main ──────────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("scan_executor_node");
  auto logger = node->get_logger();

  // ── Lettura parametri ──────────────────────────────────────────────────────
  node->declare_parameter<std::string>("global_frame", "world");
  node->declare_parameter<std::string>("planning_group", "ur_manipulator");
  node->declare_parameter<std::string>("end_effector_link", "ee_automata_tcp");
  node->declare_parameter<std::string>("home_pose_name", "home");
  node->declare_parameter<double>("trajectory_scaling_factor", 0.1);
  node->declare_parameter<std::vector<double>>("scan_center", {0.0, 0.4, 0.5});
  node->declare_parameter<double>("scan_radius", 0.35);
  node->declare_parameter<std::string>("scan_hemisphere", "upper");
  node->declare_parameter<std::string>("scan_direction", "latitudinal");
  node->declare_parameter<int>("scan_num_rings", 4);
  node->declare_parameter<int>("scan_points_per_ring", 8);
  node->declare_parameter<double>("scan_equator_exclusion_deg", 10.0);

  const std::string global_frame = node->get_parameter("global_frame").as_string();
  const std::string planning_group = node->get_parameter("planning_group").as_string();
  const std::string ee_link = node->get_parameter("end_effector_link").as_string();
  const std::string home_pose_name = node->get_parameter("home_pose_name").as_string();
  const double scaling = node->get_parameter("trajectory_scaling_factor").as_double();
  const auto center_vec = node->get_parameter("scan_center").as_double_array();
  const double radius = node->get_parameter("scan_radius").as_double();
  const std::string hemi_str = node->get_parameter("scan_hemisphere").as_string();
  const std::string dir_str = node->get_parameter("scan_direction").as_string();
  const int num_rings = node->get_parameter("scan_num_rings").as_int();
  const int points_per_ring = node->get_parameter("scan_points_per_ring").as_int();
  const double equator_exclusion_deg = node->get_parameter("scan_equator_exclusion_deg").as_double();

  if (center_vec.size() != 3) {
    RCLCPP_FATAL(logger, "%sscan_center deve avere 3 elementi.%s", ansi::RED, ansi::RESET);
    rclcpp::shutdown();
    return 1;
  }

  // ── Configurazione scan ────────────────────────────────────────────────────
  ur_automata_scan::ScanConfig cfg;
  cfg.center = Eigen::Vector3d(center_vec[0], center_vec[1], center_vec[2]);
  cfg.radius = radius;
  cfg.num_rings = num_rings;
  cfg.points_per_ring = points_per_ring;
  cfg.equator_exclusion_rad = equator_exclusion_deg * M_PI / 180.0;

  if (hemi_str == "upper") {
    cfg.hemisphere = ur_automata_scan::Hemisphere::UPPER;
  } else if (hemi_str == "lower") {
    cfg.hemisphere = ur_automata_scan::Hemisphere::LOWER;
  } else if (hemi_str == "full") {
    cfg.hemisphere = ur_automata_scan::Hemisphere::FULL;
  } else {
    RCLCPP_FATAL(logger, "%sscan_hemisphere non valido: '%s'. Usa: upper|lower|full%s",
      ansi::RED, hemi_str.c_str(), ansi::RESET);
    rclcpp::shutdown();
    return 1;
  }

  if (dir_str == "latitudinal") {
    cfg.direction = ur_automata_scan::ScanDirection::LATITUDINAL;
  } else if (dir_str == "longitudinal") {
    cfg.direction = ur_automata_scan::ScanDirection::LONGITUDINAL;
  } else {
    RCLCPP_FATAL(logger, "%sscan_direction non valido: '%s'. Usa: latitudinal|longitudinal%s",
      ansi::RED, dir_str.c_str(), ansi::RESET);
    rclcpp::shutdown();
    return 1;
  }

  // ── Generazione waypoint ───────────────────────────────────────────────────
  const auto waypoints = ur_automata_scan::generate_waypoints(cfg);

  RCLCPP_INFO(logger,
    "%sScan config:%s frame=%s | group=%s | ee=%s | scaling=%.2f",
    ansi::GREEN, ansi::RESET,
    global_frame.c_str(), planning_group.c_str(), ee_link.c_str(), scaling);
  RCLCPP_INFO(logger,
    "%sSfera:%s center=[%.3f, %.3f, %.3f] | radius=%.3f m | exclusion=%.1f°",
    ansi::GREEN, ansi::RESET,
    center_vec[0], center_vec[1], center_vec[2], radius, equator_exclusion_deg);
  RCLCPP_INFO(logger,
    "%sGenerati %zu waypoint%s (hemi=%s, dir=%s, rings=%d, pts/ring=%d)",
    ansi::GREEN, waypoints.size(), ansi::RESET,
    hemi_str.c_str(), dir_str.c_str(), num_rings, points_per_ring);

  // ── Spinner: il nodo deve girare in un executor per MoveGroupInterface ─────
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  // ── Publishers (due topic separati: pallini e frecce) ─────────────────────
  auto sphere_pub = node->create_publisher<MarkerArray>("/scan_waypoints_markers", 10);
  auto arrow_pub  = node->create_publisher<MarkerArray>("/scan_waypoints_orientations", 10);

  // Costruisce gli array iniziali (tutti grigi = non ancora visitati)
  MarkerArray sphere_array, arrow_array;
  for (size_t i = 0; i < waypoints.size(); ++i) {
    sphere_array.markers.push_back(
      make_sphere_marker(static_cast<int>(i), waypoints[i], global_frame, col::GRAY));
    arrow_array.markers.push_back(
      make_arrow_marker(static_cast<int>(i), waypoints[i], cfg.center, global_frame, col::GRAY));
  }

  auto set_marker_color = [&](size_t i, const Rgb & c) {
    sphere_array.markers[i].color.r = c.r;
    sphere_array.markers[i].color.g = c.g;
    sphere_array.markers[i].color.b = c.b;
    arrow_array.markers[i].color.r  = c.r;
    arrow_array.markers[i].color.g  = c.g;
    arrow_array.markers[i].color.b  = c.b;
  };

  auto publish_markers = [&]() {
    sphere_pub->publish(sphere_array);
    arrow_pub->publish(arrow_array);
  };

  // Pubblica i marker e aspetta che RViz li riceva
  RCLCPP_INFO(logger, "Pubblico marker su /scan_waypoints_markers e /scan_waypoints_orientations ...");
  for (int attempt = 0; attempt < 5; ++attempt) {
    publish_markers();
    rclcpp::sleep_for(500ms);
  }

  // ── MoveIt setup ──────────────────────────────────────────────────────────
  moveit::planning_interface::MoveGroupInterface move_group(node, planning_group);
  move_group.setEndEffectorLink(ee_link);
  move_group.setPoseReferenceFrame(global_frame);
  move_group.setMaxVelocityScalingFactor(scaling);
  move_group.setMaxAccelerationScalingFactor(scaling);
  move_group.setPlanningTime(5.0);

  // ── Helper: torna in home ─────────────────────────────────────────────────
  auto go_home = [&]() -> bool {
    RCLCPP_WARN(logger, "%sRecovery: torno in '%s' ...%s",
      ansi::YELLOW, home_pose_name.c_str(), ansi::RESET);
    move_group.setNamedTarget(home_pose_name);
    moveit::planning_interface::MoveGroupInterface::Plan home_plan;
    if (move_group.plan(home_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(logger, "%sRecovery fallito: pianificazione verso '%s' fallita.%s",
        ansi::RED, home_pose_name.c_str(), ansi::RESET);
      return false;
    }
    if (move_group.execute(home_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(logger, "%sRecovery fallito: esecuzione verso '%s' fallita.%s",
        ansi::RED, home_pose_name.c_str(), ansi::RESET);
      return false;
    }
    RCLCPP_INFO(logger, "%sRecovery completato: in '%s'.%s",
      ansi::GREEN, home_pose_name.c_str(), ansi::RESET);
    return true;
  };

  // ── Esecuzione ────────────────────────────────────────────────────────────
  int successi = 0;
  int falliti = 0;
  for (size_t i = 0; i < waypoints.size(); ++i) {
    RCLCPP_INFO(logger, "WP[%zu/%zu] → pianificazione ...", i + 1, waypoints.size());

    // Marker giallo: WP corrente in lavorazione
    set_marker_color(i, col::YELLOW);
    publish_markers();

    move_group.setPoseTarget(waypoints[i], ee_link);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool plan_ok =
      (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!plan_ok) {
      RCLCPP_ERROR(logger, "%sWP[%zu] pianificazione fallita%s", ansi::RED, i, ansi::RESET);
      ++falliti;
      set_marker_color(i, col::RED);
      publish_markers();
      go_home();
      continue;
    }

    const bool exec_ok =
      (move_group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!exec_ok) {
      RCLCPP_ERROR(logger, "%sWP[%zu] esecuzione fallita%s", ansi::RED, i, ansi::RESET);
      ++falliti;
      set_marker_color(i, col::RED);
      publish_markers();
      go_home();
      continue;
    }

    // Successo: marker verde
    ++successi;
    set_marker_color(i, col::GREEN);
    publish_markers();

    RCLCPP_INFO(logger, "%sWP[%zu] ✓ raggiunto%s", ansi::GREEN, i, ansi::RESET);
  }

  RCLCPP_INFO(logger,
    "%sScan completato: %d/%zu raggiunti, %d falliti.%s",
    ansi::GREEN, successi, waypoints.size(), falliti, ansi::RESET);

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
