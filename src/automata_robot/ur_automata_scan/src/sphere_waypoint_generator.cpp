#include "ur_automata_scan/sphere_waypoint_generator.hpp"

#include <cmath>
#include <stdexcept>

#include <geometry_msgs/msg/pose.hpp>

namespace ur_automata_scan {

namespace {

// ── Helpers ────────────────────────────────────────────────────────────────

/// Restituisce il punto sulla sfera in coordinate cartesiane.
/// theta: angolo polare (0 = polo nord, PI/2 = equatore, PI = polo sud)
/// phi:   angolo azimutale (0..2PI)
Eigen::Vector3d sphere_point(
  const Eigen::Vector3d & center, double radius, double theta, double phi)
{
  return center + Eigen::Vector3d(
    radius * std::sin(theta) * std::cos(phi),
    radius * std::sin(theta) * std::sin(phi),
    radius * std::cos(theta));
}

/// Costruisce una posa con l'asse Y del TCP puntante verso `center`.
/// Stessa tecnica Gram-Schmidt già usata in scan_scene_builder.cpp (Eigen).
geometry_msgs::msg::Pose make_look_at_pose(
  const Eigen::Vector3d & position, const Eigen::Vector3d & center)
{
  // Y punta verso il centro (asse ottico camera)
  Eigen::Vector3d y_dir = (center - position).normalized();

  // Vettore di riferimento per completare la base ortonormale.
  // Se y_dir è parallelo/antiparallelo a Z-world, usiamo X-world come ref.
  Eigen::Vector3d ref = Eigen::Vector3d::UnitZ();
  if (y_dir.cross(ref).norm() < 1e-6) {
    ref = Eigen::Vector3d::UnitX();
  }

  // X perpendicolare a Y e al ref; Z chiude il sistema destrorso (X × Y = Z)
  Eigen::Vector3d x_dir = (y_dir.cross(ref)).normalized();
  Eigen::Vector3d z_dir = x_dir.cross(y_dir);

  Eigen::Matrix3d rot;
  rot.col(0) = x_dir;
  rot.col(1) = y_dir;
  rot.col(2) = z_dir;
  Eigen::Quaterniond q(rot);

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

/// Genera i waypoint per un singolo emisfero.
/// upper=true  → polo nord (theta=0)        → ultimo anello a (PI/2 - exclusion)
/// upper=false → polo sud  (theta=PI)       → ultimo anello a (PI/2 + exclusion)
std::vector<geometry_msgs::msg::Pose> generate_hemisphere(
  const Eigen::Vector3d & center,
  double radius,
  bool upper,
  ScanDirection dir,
  int num_rings,
  int points_per_ring,
  double equator_exclusion)
{
  std::vector<geometry_msgs::msg::Pose> poses;

  const double pole_theta = upper ? 0.0 : M_PI;
  // sign > 0: andiamo da polo nord verso equatore (theta cresce)
  // sign < 0: andiamo da polo sud verso equatore (theta decresce)
  const double sign = upper ? 1.0 : -1.0;

  // Range angolare percorribile: dal polo fino al confine della banda equatoriale esclusa.
  const double range = (M_PI / 2.0) - equator_exclusion;

  // Primo punto: polo
  poses.push_back(make_look_at_pose(sphere_point(center, radius, pole_theta, 0.0), center));

  if (dir == ScanDirection::LATITUDINAL) {
    // Scansione per anelli orizzontali (paralleli)
    // num_rings anelli equidistanti tra polo e (equatore - exclusion), escluso polo
    const double d_theta = sign * range / num_rings;
    for (int k = 1; k <= num_rings; ++k) {
      const double theta = pole_theta + k * d_theta;
      for (int j = 0; j < points_per_ring; ++j) {
        const double phi = j * 2.0 * M_PI / points_per_ring;
        poses.push_back(make_look_at_pose(sphere_point(center, radius, theta, phi), center));
      }
    }
  } else {
    // Scansione per meridiani (longitudinale)
    // num_rings meridiani, points_per_ring punti lungo ciascuno (escluso polo)
    const double d_theta = sign * range / points_per_ring;
    for (int m = 0; m < num_rings; ++m) {
      const double phi = m * 2.0 * M_PI / num_rings;
      for (int k = 1; k <= points_per_ring; ++k) {
        const double theta = pole_theta + k * d_theta;
        poses.push_back(make_look_at_pose(sphere_point(center, radius, theta, phi), center));
      }
    }
  }

  return poses;
}

}  // namespace

// ── API pubblica ─────────────────────────────────────────────────────────────

std::vector<geometry_msgs::msg::Pose> generate_waypoints(const ScanConfig & cfg)
{
  std::vector<geometry_msgs::msg::Pose> poses;

  if (cfg.hemisphere == Hemisphere::UPPER || cfg.hemisphere == Hemisphere::FULL) {
    auto upper = generate_hemisphere(
      cfg.center, cfg.radius, true, cfg.direction,
      cfg.num_rings, cfg.points_per_ring, cfg.equator_exclusion_rad);
    poses.insert(poses.end(), upper.begin(), upper.end());
  }

  if (cfg.hemisphere == Hemisphere::LOWER || cfg.hemisphere == Hemisphere::FULL) {
    auto lower = generate_hemisphere(
      cfg.center, cfg.radius, false, cfg.direction,
      cfg.num_rings, cfg.points_per_ring, cfg.equator_exclusion_rad);
    poses.insert(poses.end(), lower.begin(), lower.end());
  }

  return poses;
}

}  // namespace ur_automata_scan
