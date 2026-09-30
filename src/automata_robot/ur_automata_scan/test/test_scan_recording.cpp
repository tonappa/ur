#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>

#include "ur_automata_scan/scan_recording.hpp"

static ScanRecording sample_recording()
{
  ScanRecording rec;
  rec.created = "2026-09-29 15:30:00";
  rec.planning_group = "ur_manipulator";
  rec.end_effector_link = "ee_automata_tcp";
  rec.global_frame = "world";
  rec.center = {0.133, 0.375, 0.455};
  rec.radius = 0.30;
  rec.waypoints_total = 82;
  rec.waypoints_reached = 77;
  rec.recoveries = 1;

  RecordedSegment wp;
  wp.label = "wp 0";
  wp.waypoint = 0;
  wp.joint_names = {"a", "b"};
  wp.camera_pose = {0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0};
  RecordedPoint p0;
  p0.time = 0.0;
  p0.positions = {0.0, -1.5707963};
  p0.velocities = {0.0, 0.0};
  p0.accelerations = {0.0, 0.0};
  RecordedPoint p1;
  p1.time = 0.123456789;
  p1.positions = {0.25, -1.2};
  wp.points = {p0, p1};

  RecordedSegment home;
  home.label = "posa home";
  home.joint_names = {"a", "b"};
  home.points = {p1};

  rec.segments = {wp, home};
  return rec;
}

// What is saved comes back the same: header, segments, optional fields,
// numbers with full precision.
TEST(ScanRecording, SaveAndLoadRoundTrip)
{
  const std::string path = "/tmp/test_scan_recording.yaml";
  std::string error;
  ASSERT_TRUE(save_recording(path, sample_recording(), error)) << error;

  ScanRecording rec;
  ASSERT_TRUE(load_recording(path, rec, error)) << error;
  std::remove(path.c_str());

  EXPECT_EQ(rec.created, "2026-09-29 15:30:00");
  EXPECT_EQ(rec.end_effector_link, "ee_automata_tcp");
  ASSERT_EQ(rec.center.size(), 3u);
  EXPECT_DOUBLE_EQ(rec.center[1], 0.375);
  EXPECT_DOUBLE_EQ(rec.radius, 0.30);
  EXPECT_EQ(rec.waypoints_reached, 77);
  EXPECT_EQ(rec.recoveries, 1);

  ASSERT_EQ(rec.segments.size(), 2u);
  const RecordedSegment & wp = rec.segments[0];
  EXPECT_EQ(wp.label, "wp 0");
  EXPECT_EQ(wp.waypoint, 0);
  EXPECT_EQ(wp.joint_names, (std::vector<std::string>{"a", "b"}));
  ASSERT_EQ(wp.camera_pose.size(), 7u);
  ASSERT_EQ(wp.points.size(), 2u);
  EXPECT_NEAR(wp.points[0].positions[1], -1.5707963, 1e-9);
  EXPECT_NEAR(wp.points[1].time, 0.123456789, 1e-9);
  EXPECT_EQ(wp.points[0].accelerations.size(), 2u);
  EXPECT_TRUE(wp.points[1].velocities.empty());

  EXPECT_EQ(rec.segments[1].waypoint, -1);
  EXPECT_TRUE(rec.segments[1].camera_pose.empty());
}

// Half speed: twice the time, half the velocity, a quarter of the acceleration,
// same positions.
TEST(ScanRecording, ScaleSegmentSpeed)
{
  RecordedSegment seg;
  RecordedPoint p;
  p.time = 2.0;
  p.positions = {0.25, -1.2};
  p.velocities = {0.4, -0.8};
  p.accelerations = {1.0, -2.0};
  seg.points = {p};

  RecordedSegment slow = scale_segment_speed(seg, 0.5);
  EXPECT_DOUBLE_EQ(slow.points[0].time, 4.0);
  EXPECT_DOUBLE_EQ(slow.points[0].positions[1], -1.2);
  EXPECT_DOUBLE_EQ(slow.points[0].velocities[1], -0.4);
  EXPECT_DOUBLE_EQ(slow.points[0].accelerations[1], -0.5);

  RecordedSegment same = scale_segment_speed(seg, 1.0);
  EXPECT_DOUBLE_EQ(same.points[0].time, 2.0);
  EXPECT_DOUBLE_EQ(same.points[0].velocities[0], 0.4);
}

// A missing file or a point with the wrong number of joints is an error, not
// a partial recording.
TEST(ScanRecording, RejectsBadFiles)
{
  ScanRecording rec;
  std::string error;
  EXPECT_FALSE(load_recording("/tmp/does_not_exist_scan.yaml", rec, error));

  const std::string path = "/tmp/test_scan_recording_bad.yaml";
  ScanRecording bad = sample_recording();
  bad.segments[0].points[1].positions = {0.1};   // 1 value for 2 joints
  ASSERT_TRUE(save_recording(path, bad, error)) << error;
  EXPECT_FALSE(load_recording(path, rec, error));
  std::remove(path.c_str());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
