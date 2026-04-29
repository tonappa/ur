#pragma once

#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>

namespace ur_automata_scan {

enum class Hemisphere { UPPER, LOWER, FULL };
enum class ScanDirection { LATITUDINAL, LONGITUDINAL };

struct ScanConfig {
  Eigen::Vector3d center;
  double radius;
  Hemisphere hemisphere;
  ScanDirection direction;
  int num_rings;        // latitudinale: anelli (escluso polo) | longitudinale: meridiani
  int points_per_ring;  // latitudinale: punti/anello | longitudinale: punti/meridiano (escluso polo)
  double equator_exclusion_rad;  // banda angolare attorno all'equatore esclusa dallo scan
};

/// Genera la lista ordinata di pose per lo scan sferico.
///
/// Orientamento: l'asse Y di ee_automata_tcp punta verso il centro della sfera
/// in ogni waypoint (asse ottico della camera = Y del TCP).
///
/// Ordine:
///   - Il primo punto di ogni emisfero è sempre il polo.
///   - Hemisphere::FULL → emisfero superiore completo, poi inferiore.
std::vector<geometry_msgs::msg::Pose> generate_waypoints(const ScanConfig & cfg);

}  // namespace ur_automata_scan
