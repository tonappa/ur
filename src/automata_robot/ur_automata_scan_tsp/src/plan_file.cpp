#include "ur_automata_scan_tsp/plan_file.hpp"

#include <fstream>

#include <yaml-cpp/yaml.h>

// Writes a list of numbers on one line: [a, b, c].
static void emit_numbers(YAML::Emitter & out, const std::vector<double> & values)
{
  out << YAML::Flow << YAML::BeginSeq;
  for (double v : values) out << v;
  out << YAML::EndSeq;
}

bool save_plan(const std::string & path, const ScanPlan & plan, std::string & error)
{
  // Header and segments with the writer of ur_automata_scan, then the plan
  // section appended as one more top-level key.
  if (!save_recording(path, plan.recording, error)) return false;

  const PlanData & d = plan.data;
  YAML::Emitter out;
  out.SetDoublePrecision(9);
  out << YAML::BeginMap;
  out << YAML::Key << "plan" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "method" << YAML::Value << d.method;
  out << YAML::Key << "scaling" << YAML::Value << d.scaling;
  out << YAML::Key << "joint_names" << YAML::Value << YAML::Flow << d.joint_names;
  out << YAML::Key << "start_joints" << YAML::Value;
  emit_numbers(out, d.start_joints);
  out << YAML::Key << "max_velocity" << YAML::Value;
  emit_numbers(out, d.max_velocity);
  out << YAML::Key << "max_acceleration" << YAML::Value;
  emit_numbers(out, d.max_acceleration);
  out << YAML::Key << "visit_order" << YAML::Value << YAML::Flow << d.visit_order;

  out << YAML::Key << "waypoint_params" << YAML::Value << YAML::BeginMap;
  for (const auto & kv : d.waypoint_params) out << YAML::Key << kv.first << YAML::Value << kv.second;
  out << YAML::EndMap;

  out << YAML::Key << "metrics" << YAML::Value << YAML::BeginMap;
  for (const auto & kv : d.metrics) out << YAML::Key << kv.first << YAML::Value << kv.second;
  out << YAML::EndMap;

  out << YAML::Key << "scene" << YAML::Value << YAML::BeginSeq;
  for (const SceneObjectBox & obj : d.scene) {
    out << YAML::Flow << YAML::BeginMap;
    out << YAML::Key << "id" << YAML::Value << obj.id;
    out << YAML::Key << "box" << YAML::Value;
    emit_numbers(out, obj.box);
    out << YAML::EndMap;
  }
  out << YAML::EndSeq;

  out << YAML::Key << "waypoints" << YAML::Value << YAML::BeginSeq;
  for (const std::vector<double> & wp : d.waypoints) emit_numbers(out, wp);
  out << YAML::EndSeq;

  out << YAML::EndMap;
  out << YAML::EndMap;
  if (!out.good()) {
    error = "YAML emitter: " + out.GetLastError();
    return false;
  }
  std::ofstream file(path, std::ios::app);
  if (!file) {
    error = "cannot write " + path;
    return false;
  }
  file << out.c_str() << "\n";
  return static_cast<bool>(file);
}

bool load_plan(const std::string & path, ScanPlan & plan, std::string & error)
{
  plan = ScanPlan();
  if (!load_recording(path, plan.recording, error)) return false;
  try {
    YAML::Node root = YAML::LoadFile(path);
    YAML::Node p = root["plan"];
    if (!p) {
      error = path + ": no 'plan' section (a recording of scan_sequence_node, not a plan of scan_tsp_node)";
      return false;
    }
    PlanData & d = plan.data;
    d.method           = p["method"].as<std::string>();
    d.scaling          = p["scaling"].as<double>();
    d.joint_names      = p["joint_names"].as<std::vector<std::string>>();
    d.start_joints     = p["start_joints"].as<std::vector<double>>();
    d.max_velocity     = p["max_velocity"].as<std::vector<double>>();
    d.max_acceleration = p["max_acceleration"].as<std::vector<double>>();
    d.visit_order      = p["visit_order"].as<std::vector<int>>();
    d.waypoint_params  = p["waypoint_params"].as<std::map<std::string, std::string>>();
    d.metrics          = p["metrics"].as<std::map<std::string, double>>();
    for (const YAML::Node & obj : p["scene"]) {
      SceneObjectBox box;
      box.id  = obj["id"].as<std::string>();
      box.box = obj["box"].as<std::vector<double>>();
      d.scene.push_back(box);
    }
    for (const YAML::Node & wp : p["waypoints"]) {
      d.waypoints.push_back(wp.as<std::vector<double>>());
      if (d.waypoints.back().size() != 7) {
        error = path + ": a waypoint does not have 7 values";
        return false;
      }
    }
    if (d.start_joints.size() != d.joint_names.size()) {
      error = path + ": start_joints and joint_names have different sizes";
      return false;
    }
  } catch (const std::exception & e) {
    error = path + ": " + e.what();
    return false;
  }
  return true;
}
