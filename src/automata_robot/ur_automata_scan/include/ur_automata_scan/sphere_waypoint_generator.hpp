#pragma once

#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>

// Which half of the sphere to scan
#define HEMISPHERE_UPPER 0
#define HEMISPHERE_LOWER 1
#define HEMISPHERE_FULL  2

// Order in which waypoints are visited
#define SCAN_LATITUDINAL  0   // ring by ring (horizontal circles)
#define SCAN_LONGITUDINAL 1   // meridian by meridian (vertical slices)

// All the settings for one scan
struct ScanConfig {
  Eigen::Vector3d center;       // center of the sphere in 3D space
  double radius;                // radius of the sphere in meters
  int hemisphere;               // HEMISPHERE_UPPER / LOWER / FULL
  int direction;                // SCAN_LATITUDINAL or SCAN_LONGITUDINAL

  // latitudinal scan parameters
  int num_rings;                // number of rings between pole and equator
  int points_per_ring;          // points per ring

  // longitudinal scan parameters
  int num_arc;                  // number of meridians
  int points_per_arc;           // points per meridian (excluding pole)

  double equator_exclusion_rad; // angle (radians) to skip near the equator
};

// Generates the list of poses the robot should visit during the scan.
// At every pose the end-effector Y-axis points toward the sphere center.
std::vector<geometry_msgs::msg::Pose> generate_waypoints(const ScanConfig & cfg);
