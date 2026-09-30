#include <gtest/gtest.h>

#include "ur_automata_scan/frustum_markers.hpp"

// One frustum per waypoint segment (not for "posa home"), gray at first,
// green only for the segment that was reached.
TEST(FrustumMarkers, OnePerWaypointAndGreenWhenReached)
{
  RecordedSegment home;
  home.label = "posa home";

  RecordedSegment wp;
  wp.label = "wp 7";
  wp.waypoint = 7;
  wp.camera_pose = {0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0};

  ScanRecording rec;
  rec.segments = {home, wp, wp};

  visualization_msgs::msg::MarkerArray arr = make_frustum_markers(rec, "world");
  ASSERT_EQ(arr.markers.size(), 2u);
  EXPECT_EQ(arr.markers[0].id, 1);
  EXPECT_EQ(arr.markers[1].id, 2);
  EXPECT_EQ(arr.markers[0].header.frame_id, "world");
  EXPECT_EQ(arr.markers[0].points.size(), 16u);   // 4 rays + 4 sides, 2 points each
  EXPECT_DOUBLE_EQ(arr.markers[0].pose.position.z, 0.3);
  EXPECT_FLOAT_EQ(arr.markers[0].color.g, 0.5f);

  set_frustum_reached(arr, 0);   // "posa home": nothing changes
  EXPECT_FLOAT_EQ(arr.markers[0].color.g, 0.5f);

  set_frustum_reached(arr, 2);
  EXPECT_FLOAT_EQ(arr.markers[0].color.g, 0.5f);
  EXPECT_FLOAT_EQ(arr.markers[1].color.g, 1.0f);
  EXPECT_FLOAT_EQ(arr.markers[1].color.r, 0.0f);
}
