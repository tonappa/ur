#include <gtest/gtest.h>

#include <cstdio>

#include "ur_automata_scan_tsp/plan_file.hpp"

static ScanPlan sample_plan()
{
  ScanPlan plan;
  ScanRecording & rec = plan.recording;
  rec.created = "2026-09-29 18:00:00";
  rec.planning_group = "ur_manipulator";
  rec.end_effector_link = "ee_automata_tcp";
  rec.global_frame = "world";
  rec.center = {0.133, 0.375, 0.455};
  rec.radius = 0.30;
  rec.waypoints_total = 2;
  rec.waypoints_reached = 1;

  RecordedSegment wp;
  wp.label = "wp 1";
  wp.waypoint = 1;
  wp.joint_names = {"a", "b"};
  wp.camera_pose = {0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0};
  RecordedPoint p0;
  p0.positions = {0.0, 0.0};
  RecordedPoint p1;
  p1.time = 1.25;
  p1.positions = {0.5, -0.5};
  wp.points = {p0, p1};
  rec.segments = {wp};

  PlanData & d = plan.data;
  d.method = "tsp";
  d.joint_names = {"a", "b"};
  d.start_joints = {0.0, 0.0};
  d.visit_order = {1};
  d.waypoints = {{0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0}, {0.4, 0.5, 0.6, 0.0, 0.0, 0.0, 1.0}};
  d.waypoint_params = {{"scan_num_rings", "4"}, {"scan_hemisphere", "full"}};
  d.scaling = 0.4;
  d.max_velocity = {3.14, 3.14};
  d.max_acceleration = {5.0, 5.0};
  d.scene = {{"platform", {0.0, 0.1, 0.2, 0.3, 0.4, 0.5}}};
  d.metrics = {{"trajectory_s", 1.25}};
  return plan;
}

// Plan section and recording come back the same; the file is still a valid
// recording for scan_replay_node (load_recording ignores the plan section).
TEST(PlanFile, RoundTripAndStillARecording)
{
  const std::string path = "/tmp/test_scan_tsp_plan.yaml";
  std::string error;
  ASSERT_TRUE(save_plan(path, sample_plan(), error)) << error;

  ScanPlan plan;
  ASSERT_TRUE(load_plan(path, plan, error)) << error;
  EXPECT_EQ(plan.data.method, "tsp");
  EXPECT_EQ(plan.data.visit_order, (std::vector<int>{1}));
  ASSERT_EQ(plan.data.waypoints.size(), 2u);
  EXPECT_DOUBLE_EQ(plan.data.waypoints[1][2], 0.6);
  EXPECT_EQ(plan.data.waypoint_params.at("scan_num_rings"), "4");
  EXPECT_DOUBLE_EQ(plan.data.scaling, 0.4);
  ASSERT_EQ(plan.data.scene.size(), 1u);
  EXPECT_EQ(plan.data.scene[0].id, "platform");
  EXPECT_DOUBLE_EQ(plan.data.scene[0].box[4], 0.4);
  EXPECT_DOUBLE_EQ(plan.data.metrics.at("trajectory_s"), 1.25);
  ASSERT_EQ(plan.recording.segments.size(), 1u);
  EXPECT_NEAR(plan.recording.segments[0].points[1].time, 1.25, 1e-9);

  ScanRecording rec;
  EXPECT_TRUE(load_recording(path, rec, error)) << error;
  std::remove(path.c_str());
}

// A plain recording of scan_sequence_node has no plan section: refused.
TEST(PlanFile, RejectsPlainRecording)
{
  const std::string path = "/tmp/test_scan_tsp_recording.yaml";
  std::string error;
  ASSERT_TRUE(save_recording(path, sample_plan().recording, error)) << error;
  ScanPlan plan;
  EXPECT_FALSE(load_plan(path, plan, error));
  EXPECT_NE(error.find("no 'plan' section"), std::string::npos);
  std::remove(path.c_str());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
