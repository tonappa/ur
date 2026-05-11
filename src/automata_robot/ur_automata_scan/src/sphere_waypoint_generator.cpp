#include "ur_automata_scan/sphere_waypoint_generator.hpp"

#include <cmath>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>

// ------------------------------------------------------------------
// Helper: given spherical coordinates, return a 3D cartesian point
// theta = polar angle   (0 = north pole, PI/2 = equator, PI = south pole)
// phi   = azimuth angle (0 .. 2*PI around the sphere)
// ------------------------------------------------------------------
static Eigen::Vector3d sphere_point(
  const Eigen::Vector3d & center, double radius, double theta, double phi)
{
  return center + Eigen::Vector3d(
    radius * std::sin(theta) * std::cos(phi),
    radius * std::sin(theta) * std::sin(phi),
    radius * std::cos(theta));
}

// ------------------------------------------------------------------
// Helper: build a Pose at `position` with the Y-axis pointing toward `center`.
// (The camera optical axis is the Y-axis of the TCP.)
// We use basic linear algebra (cross products) to build the rotation matrix.
// ------------------------------------------------------------------
static geometry_msgs::msg::Pose make_look_at_pose(
  const Eigen::Vector3d & position, const Eigen::Vector3d & center)
{
  // Y points toward the sphere center
  Eigen::Vector3d y_dir = (center - position).normalized();

  // Pick a reference vector to build the other two axes.
  // If y_dir is nearly parallel to the Z world axis, use X instead.
  Eigen::Vector3d ref = Eigen::Vector3d::UnitZ();
  if (y_dir.cross(ref).norm() < 1e-6) {
    ref = Eigen::Vector3d::UnitX();
  }

  // X is perpendicular to both Y and the reference
  Eigen::Vector3d x_dir = (y_dir.cross(ref)).normalized();
  // Z closes the right-handed frame
  Eigen::Vector3d z_dir = x_dir.cross(y_dir);

  // Build rotation matrix from the three axis vectors
  Eigen::Matrix3d rot;
  rot.col(0) = x_dir;
  rot.col(1) = y_dir;
  rot.col(2) = z_dir;

  // Convert rotation matrix to quaternion
  Eigen::Quaterniond q(rot);

  // Fill in the ROS Pose message
  geometry_msgs::msg::Pose pose;
  pose.position.x = position.x();
  pose.position.y = position.y();
  pose.position.z = position.z();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

// ------------------------------------------------------------------
// Generate waypoints for one hemisphere (upper or lower).
//
// upper = true  -> starts at north pole (theta=0),   goes toward equator
// upper = false -> starts at south pole (theta=PI),  goes toward equator
// ------------------------------------------------------------------
static std::vector<geometry_msgs::msg::Pose> generate_hemisphere(
  const Eigen::Vector3d & center,
  double radius,
  bool upper,
  int direction,
  int num_rings,
  int points_per_ring,
  int num_arc,
  int points_per_arc,
  double equator_exclusion,
  bool stagger_rings,
  bool adaptive_rings)
{
  std::vector<geometry_msgs::msg::Pose> poses;

  // Pole angle and the direction we move toward the equator
  double pole_theta = upper ? 0.0 : M_PI;
  double sign       = upper ? 1.0 : -1.0;  // +1 grows theta, -1 shrinks it

  // Total angular range from pole to (equator - exclusion band)
  double range = (M_PI / 2.0) - equator_exclusion;

  // First waypoint is always the pole itself
  poses.push_back(make_look_at_pose(sphere_point(center, radius, pole_theta, 0.0), center));

  if (direction == SCAN_LATITUDINAL) {
    // Ring-by-ring: num_rings horizontal circles between pole and equator.
    // Boustrophedon (serpentine): odd rings go forward (phi 0 -> 2pi),
    // even rings go backward (phi 2pi -> 0), so the robot doesn't have to
    // jump back to phi=0 at the start of each new ring.
    double d_theta = sign * range / num_rings;

    for (int ring = 1; ring <= num_rings; ++ring) {
      double theta = pole_theta + ring * d_theta;
      bool reverse = (ring % 2 == 0);

      // Adaptive: scale points by sin(theta) so coverage is roughly uniform.
      // points_per_ring is the reference density at the equator (theta = PI/2).
      // Near the pole sin(theta) is small -> fewer points; near equator -> more.
      int n_points = points_per_ring;
      if (adaptive_rings) {
        n_points = std::max(1, (int)std::round(points_per_ring * std::sin(theta)));
      }

      // Half step and stagger are computed per-ring because n_points may vary
      double half_step     = M_PI / n_points;
      double stagger_offset = (stagger_rings && ring % 2 == 0) ? half_step : 0.0;

      for (int pt = 0; pt < n_points; ++pt) {
        // If reverse, walk the points from last to first
        int actual_pt = reverse ? (n_points - 1 - pt) : pt;
        double phi = actual_pt * 2.0 * M_PI / n_points + stagger_offset;
        poses.push_back(make_look_at_pose(sphere_point(center, radius, theta, phi), center));
      }
    }

  } else {
    // Meridian-by-meridian: num_arc vertical slices, points_per_arc points each.
    // Boustrophedon: odd meridians go pole -> equator, even go equator -> pole.
    double d_theta = sign * range / points_per_arc;

    for (int meridian = 0; meridian < num_arc; ++meridian) {
      double phi = meridian * 2.0 * M_PI / num_arc;
      bool reverse = (meridian % 2 == 1);

      for (int pt = 1; pt <= points_per_arc; ++pt) {
        // If reverse, walk the arc points from equator end back toward the pole
        int actual_pt = reverse ? (points_per_arc + 1 - pt) : pt;
        double theta = pole_theta + actual_pt * d_theta;
        poses.push_back(make_look_at_pose(sphere_point(center, radius, theta, phi), center));
      }
    }
  }

  return poses;
}

// ------------------------------------------------------------------
// Public function: generate all waypoints according to the ScanConfig.
// ------------------------------------------------------------------
std::vector<geometry_msgs::msg::Pose> generate_waypoints(const ScanConfig & cfg)
{
  std::vector<geometry_msgs::msg::Pose> poses;

  // Add upper hemisphere waypoints if requested
  if (cfg.hemisphere == HEMISPHERE_UPPER || cfg.hemisphere == HEMISPHERE_FULL) {
    std::vector<geometry_msgs::msg::Pose> upper = generate_hemisphere(
      cfg.center, cfg.radius, true, cfg.direction,
      cfg.num_rings, cfg.points_per_ring,
      cfg.num_arc,   cfg.points_per_arc,
      cfg.equator_exclusion_upper_rad, cfg.stagger_rings, cfg.adaptive_rings);

    poses.insert(poses.end(), upper.begin(), upper.end());
  }

  // Add lower hemisphere waypoints if requested
  if (cfg.hemisphere == HEMISPHERE_LOWER || cfg.hemisphere == HEMISPHERE_FULL) {
    std::vector<geometry_msgs::msg::Pose> lower = generate_hemisphere(
      cfg.center, cfg.radius, false, cfg.direction,
      cfg.num_rings, cfg.points_per_ring,
      cfg.num_arc,   cfg.points_per_arc,
      cfg.equator_exclusion_lower_rad, cfg.stagger_rings, cfg.adaptive_rings);

    poses.insert(poses.end(), lower.begin(), lower.end());
  }

  return poses;
}
