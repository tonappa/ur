#include <gtest/gtest.h>

#include <cmath>

#include "ur_automata_scan/sphere_waypoint_generator.hpp"

// Same sphere as the reference scan: center C, radius 0.30, full sphere 4 x 15
// with staggered and adaptive rings, robot base at the origin.
static ScanConfig reference_config()
{
  ScanConfig cfg;
  cfg.center = Eigen::Vector3d(0.133, 0.375, 0.455);
  cfg.radius = 0.30;
  cfg.direction = SCAN_LATITUDINAL;
  cfg.num_rings = 4;
  cfg.points_per_ring = 15;
  cfg.num_arc = 5;
  cfg.points_per_arc = 4;
  cfg.equator_exclusion_upper_rad = 15.0 * M_PI / 180.0;
  cfg.equator_exclusion_lower_rad = 20.0 * M_PI / 180.0;
  cfg.stagger_rings = true;
  cfg.adaptive_rings = true;
  return cfg;
}

static Eigen::Vector3d pos(const geometry_msgs::msg::Pose & p)
{
  return Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
}

// Every waypoint is kept exactly once, there are 4 sectors per hemisphere, the
// lower hemisphere starts right after the upper one, and inside a sector two
// consecutive points are neighbours on the sphere (no jump across it).
TEST(Sectors, KeepsAllPointsAndGroupsThemBySector)
{
  ScanConfig cfg = reference_config();
  ScanConfig upper = cfg;
  upper.hemisphere = HEMISPHERE_UPPER;
  ScanConfig lower = cfg;
  lower.hemisphere = HEMISPHERE_LOWER;
  std::vector<geometry_msgs::msg::Pose> up = generate_waypoints(upper);
  std::vector<geometry_msgs::msg::Pose> lo = generate_waypoints(lower);

  const double phi_front = std::atan2(-cfg.center.y(), -cfg.center.x());
  std::vector<geometry_msgs::msg::Pose> out;
  std::vector<SectorBlock> blocks;
  order_by_sectors(up, cfg.center, phi_front, 4, true, out, blocks);
  order_by_sectors(lo, cfg.center, phi_front, 4, false, out, blocks);

  ASSERT_EQ(out.size(), up.size() + lo.size());
  std::vector<geometry_msgs::msg::Pose> all = up;
  all.insert(all.end(), lo.begin(), lo.end());
  for (const auto & p : all) {
    int found = 0;
    for (const auto & q : out) {
      if ((pos(p) - pos(q)).norm() < 1e-9) ++found;
    }
    EXPECT_EQ(found, 1);
  }

  ASSERT_EQ(blocks.size(), 8u);
  EXPECT_EQ(blocks[0].first_wp, 0u);
  EXPECT_EQ(blocks[4].first_wp, up.size());
  for (size_t b = 0; b < blocks.size(); ++b) {
    EXPECT_EQ(blocks[b].upper, b < 4);
    EXPECT_EQ(blocks[b].sector, static_cast<int>(b % 4));
    size_t end = (b + 1 < blocks.size()) ? blocks[b + 1].first_wp : out.size();
    for (size_t i = blocks[b].first_wp + 1; i < end; ++i) {
      EXPECT_LT((pos(out[i]) - pos(out[i - 1])).norm(), 0.6 * cfg.radius);
    }
  }
}

// Sector 0 faces the robot: all its points (except the pole) are on the base
// side of the center, within +-45 degrees of the center -> base direction.
TEST(Sectors, FirstSectorFacesTheRobot)
{
  ScanConfig cfg = reference_config();
  cfg.hemisphere = HEMISPHERE_UPPER;
  std::vector<geometry_msgs::msg::Pose> up = generate_waypoints(cfg);

  const double phi_front = std::atan2(-cfg.center.y(), -cfg.center.x());
  std::vector<geometry_msgs::msg::Pose> out;
  std::vector<SectorBlock> blocks;
  order_by_sectors(up, cfg.center, phi_front, 4, true, out, blocks);

  ASSERT_GE(blocks.size(), 2u);
  Eigen::Vector3d to_base = -cfg.center;
  to_base.z() = 0.0;
  to_base.normalize();
  for (size_t i = blocks[0].first_wp; i < blocks[1].first_wp; ++i) {
    Eigen::Vector3d v = pos(out[i]) - cfg.center;
    v.z() = 0.0;
    if (v.norm() < 1e-6) continue;   // the pole
    EXPECT_GE(v.normalized().dot(to_base), std::cos(M_PI / 4.0) - 1e-9);
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
