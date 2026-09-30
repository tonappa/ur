#pragma once

#include <map>
#include <string>
#include <vector>

#include "ur_automata_scan/scan_recording.hpp"

// A saved scan plan of scan_tsp_node. The file is a scan recording of
// ur_automata_scan (same keys, so scan_replay_node can also play it) plus a
// `plan:` section with what is needed to recognize the waypoints, the robot
// setup and the scene the trajectory was planned for.

// Axis-aligned box that holds one object of the planning scene (world frame).
struct SceneObjectBox {
  std::string id;
  std::vector<double> box;   // min x, y, z, max x, y, z (m)
};

struct PlanData {
  std::string method;                          // visit order: tsp | gtsp | sectors | alternate
  std::vector<std::string> joint_names;        // planning group order
  std::vector<double> start_joints;            // robot state the plan starts from
  std::vector<int> visit_order;                // waypoint indices, in visit order
  std::vector<std::vector<double>> waypoints;  // every waypoint: x y z qx qy qz qw
  std::map<std::string, std::string> waypoint_params;  // scan settings that decide the waypoints
  double scaling = 0.0;                        // trajectory_scaling_factor
  std::vector<double> max_velocity;            // joint limits of the model (unscaled)
  std::vector<double> max_acceleration;
  std::vector<SceneObjectBox> scene;
  std::map<std::string, double> metrics;       // planning time, trajectory time, ...
};

struct ScanPlan {
  ScanRecording recording;
  PlanData data;
};

// Writes the plan. Returns false (and fills `error`) on failure.
bool save_plan(const std::string & path, const ScanPlan & plan, std::string & error);

// Reads a plan written by save_plan. A plain recording without the `plan:`
// section is refused.
bool load_plan(const std::string & path, ScanPlan & plan, std::string & error);
