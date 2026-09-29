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

  double equator_exclusion_upper_rad; // angle (radians) to skip near equator for upper hemisphere
  double equator_exclusion_lower_rad; // angle (radians) to skip near equator for lower hemisphere
  bool stagger_rings;                 // if true, even rings are offset by half angular step
  bool adaptive_rings;                // if true, scale points_per_ring by sin(theta) so coverage is uniform
};

// Generates the list of poses the robot should visit during the scan.
// At every pose the end-effector Y-axis points toward the sphere center.
std::vector<geometry_msgs::msg::Pose> generate_waypoints(const ScanConfig & cfg);

// A sector is an azimuth slice of one hemisphere. Waypoints are visited sector
// by sector instead of ring by ring: the points of one slice are close to each
// other and can be reached with similar arm configurations, instead of walking
// each ring all the way around the object (on 2026-09-29 this halved the
// segments whose straight joint-space line collides and needs OMPL).
struct SectorBlock {
  size_t first_wp = 0;        // index (in visit order) of the first waypoint of the sector
  int sector = 0;             // 0 = front (robot side), then counterclockwise seen from above
  bool upper = true;          // hemisphere
};

// Reorders the waypoints of ONE hemisphere by sectors and appends them to `out`.
// phi_front = azimuth (rad) of the direction center -> robot base: sector 0 is
// centered there. Inside a sector the rings go from the pole to the equator
// (the other way round in odd sectors, so the next sector starts close to where
// the previous one ended) and consecutive rings are walked in opposite
// directions (serpentine).
void order_by_sectors(const std::vector<geometry_msgs::msg::Pose> & pts,
                      const Eigen::Vector3d & center, double phi_front,
                      int num_sectors, bool upper,
                      std::vector<geometry_msgs::msg::Pose> & out,
                      std::vector<SectorBlock> & blocks);
