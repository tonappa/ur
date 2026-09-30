#pragma once

#include <string>
#include <vector>

// A recorded scan: every motion the robot executed during a calibration run,
// in order, with its full timing. scan_sequence_node writes it (record:=true),
// scan_replay_node plays it back without planning anything.

// One point of a joint trajectory.
struct RecordedPoint {
  double time = 0.0;                   // time from the start of the motion (s)
  std::vector<double> positions;       // rad
  std::vector<double> velocities;      // rad/s, may be empty
  std::vector<double> accelerations;   // rad/s^2, may be empty
};

// One executed motion.
struct RecordedSegment {
  std::string label;                   // e.g. "wp 12", "posa home"
  int waypoint = -1;                   // waypoint reached at the end, -1 = not a waypoint
  std::vector<std::string> joint_names;
  // Camera pose reached at the end (x y z qx qy qz qw, planning frame), only
  // for waypoints: the replay checks that the current robot model still puts
  // the camera there.
  std::vector<double> camera_pose;
  std::vector<RecordedPoint> points;
};

struct ScanRecording {
  std::string created;                 // date and time of the calibration run
  std::string planning_group;
  std::string end_effector_link;
  std::string global_frame;
  std::vector<double> center;          // scan center (m)
  double radius = 0.0;                 // scan radius (m)
  int waypoints_total = 0;
  int waypoints_reached = 0;
  int recoveries = 0;                  // recovery motions in the calibration run
  std::vector<RecordedSegment> segments;
};

// The same motion played at `speed` times the recorded speed (0.5 = half speed,
// twice the time). The joint positions do not change, so the path is the same:
// only times, velocities and accelerations are rescaled. `speed` must be > 0.
RecordedSegment scale_segment_speed(const RecordedSegment & seg, double speed);

// Writes the recording as YAML. Returns false (and fills `error`) on failure.
bool save_recording(const std::string & path, const ScanRecording & rec, std::string & error);

// Reads a recording written by save_recording. Returns false (and fills
// `error`) if the file is missing or malformed.
bool load_recording(const std::string & path, ScanRecording & rec, std::string & error);
