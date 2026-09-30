#pragma once

// Shared by scan_replay_node and scan_tsp_node (package ur_automata_scan_tsp).

#include <chrono>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "ur_automata_scan/scan_recording.hpp"

// ============================================================================
// RViz markers of a recorded scan being played back
// ============================================================================
// One small camera frustum (wireframe pyramid, like the camera of a 3D editor)
// at every recorded waypoint pose: gray = not reached yet, green = reached.
// The waypoints the calibration could not reach are not in the recording, so
// they are not drawn. Topic: /scan_waypoints_markers, the one scan_sequence_node
// uses, already shown by the RViz config.

// Size of the drawn frustum (m). Only a drawing, not the real field of view of
// the camera: change these to make it match the lens.
const double FRUSTUM_DEPTH       = 0.05;   // along the optical axis (Y of the TCP)
const double FRUSTUM_HALF_WIDTH  = 0.03;   // along X of the TCP
const double FRUSTUM_HALF_HEIGHT = 0.02;   // along Z of the TCP

inline geometry_msgs::msg::Point make_point(double x, double y, double z)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

// One gray frustum per recorded waypoint. The marker id is the index of the
// segment in the recording.
inline visualization_msgs::msg::MarkerArray make_frustum_markers(
  const ScanRecording & rec, const std::string & frame)
{
  using visualization_msgs::msg::Marker;

  // Frustum in the camera frame: the apex is the camera, the rectangle is in
  // front of it along +Y.
  const geometry_msgs::msg::Point apex = make_point(0.0, 0.0, 0.0);
  const geometry_msgs::msg::Point corners[4] = {
    make_point(+FRUSTUM_HALF_WIDTH, FRUSTUM_DEPTH, +FRUSTUM_HALF_HEIGHT),
    make_point(-FRUSTUM_HALF_WIDTH, FRUSTUM_DEPTH, +FRUSTUM_HALF_HEIGHT),
    make_point(-FRUSTUM_HALF_WIDTH, FRUSTUM_DEPTH, -FRUSTUM_HALF_HEIGHT),
    make_point(+FRUSTUM_HALF_WIDTH, FRUSTUM_DEPTH, -FRUSTUM_HALF_HEIGHT)};

  visualization_msgs::msg::MarkerArray arr;
  for (size_t k = 0; k < rec.segments.size(); ++k) {
    const RecordedSegment & seg = rec.segments[k];
    if (seg.waypoint < 0 || seg.camera_pose.size() != 7) continue;

    Marker m;
    m.header.frame_id = frame;
    m.ns     = "scan_frustums";
    m.id     = static_cast<int>(k);
    m.type   = Marker::LINE_LIST;
    m.action = Marker::ADD;
    m.pose.position.x    = seg.camera_pose[0];
    m.pose.position.y    = seg.camera_pose[1];
    m.pose.position.z    = seg.camera_pose[2];
    m.pose.orientation.x = seg.camera_pose[3];
    m.pose.orientation.y = seg.camera_pose[4];
    m.pose.orientation.z = seg.camera_pose[5];
    m.pose.orientation.w = seg.camera_pose[6];
    // LINE_LIST: every pair of points is one line.
    for (int c = 0; c < 4; ++c) {
      m.points.push_back(apex);                  // ray from the camera to a corner
      m.points.push_back(corners[c]);
      m.points.push_back(corners[c]);            // side of the rectangle
      m.points.push_back(corners[(c + 1) % 4]);
    }
    m.scale.x = 0.001;   // line width
    m.color.r = 0.5f;
    m.color.g = 0.5f;
    m.color.b = 0.5f;
    m.color.a = 0.9f;
    arr.markers.push_back(m);
  }
  return arr;
}

// Turns green the frustum of segment `segment_index` (nothing happens if that
// segment is not a waypoint).
inline void set_frustum_reached(visualization_msgs::msg::MarkerArray & arr, size_t segment_index)
{
  for (visualization_msgs::msg::Marker & m : arr.markers) {
    if (m.id == static_cast<int>(segment_index)) {
      m.color.r = 0.0f;
      m.color.g = 1.0f;
      m.color.b = 0.0f;
      m.scale.x = 0.002;
    }
  }
}

// First publication: removes the markers left by a previous run, then shows
// the frustums. Repeated a few times because RViz may not be connected to a
// publisher that was just created.
inline void publish_first_frustums(
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub,
  const visualization_msgs::msg::MarkerArray & arr)
{
  visualization_msgs::msg::Marker delete_all;
  delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
  visualization_msgs::msg::MarkerArray clear;
  clear.markers.push_back(delete_all);
  for (int attempt = 0; attempt < 5; ++attempt) {
    pub->publish(clear);
    pub->publish(arr);
    rclcpp::sleep_for(std::chrono::milliseconds(200));
  }
}
