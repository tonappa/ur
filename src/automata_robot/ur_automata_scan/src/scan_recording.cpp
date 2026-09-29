#include "ur_automata_scan/scan_recording.hpp"

#include <fstream>

#include <yaml-cpp/yaml.h>

// Writes a list of numbers on one line: [a, b, c].
static void emit_numbers(YAML::Emitter & out, const std::vector<double> & values)
{
  out << YAML::Flow << YAML::BeginSeq;
  for (double v : values) out << v;
  out << YAML::EndSeq;
}

bool save_recording(const std::string & path, const ScanRecording & rec, std::string & error)
{
  YAML::Emitter out;
  out.SetDoublePrecision(9);
  out << YAML::Comment("Scan recording written by scan_sequence_node (record:=true). "
                       "Play it back with scan_replay.launch.py.");
  out << YAML::BeginMap;
  out << YAML::Key << "created"           << YAML::Value << rec.created;
  out << YAML::Key << "planning_group"    << YAML::Value << rec.planning_group;
  out << YAML::Key << "end_effector_link" << YAML::Value << rec.end_effector_link;
  out << YAML::Key << "global_frame"      << YAML::Value << rec.global_frame;
  out << YAML::Key << "center"            << YAML::Value;
  emit_numbers(out, rec.center);
  out << YAML::Key << "radius"            << YAML::Value << rec.radius;
  out << YAML::Key << "waypoints_total"   << YAML::Value << rec.waypoints_total;
  out << YAML::Key << "waypoints_reached" << YAML::Value << rec.waypoints_reached;
  out << YAML::Key << "recoveries"        << YAML::Value << rec.recoveries;

  out << YAML::Key << "segments" << YAML::Value << YAML::BeginSeq;
  for (const RecordedSegment & seg : rec.segments) {
    out << YAML::BeginMap;
    out << YAML::Key << "label"    << YAML::Value << seg.label;
    out << YAML::Key << "waypoint" << YAML::Value << seg.waypoint;
    out << YAML::Key << "joint_names" << YAML::Value << YAML::Flow << seg.joint_names;
    if (!seg.camera_pose.empty()) {
      out << YAML::Key << "camera_pose" << YAML::Value;
      emit_numbers(out, seg.camera_pose);
    }
    out << YAML::Key << "points" << YAML::Value << YAML::BeginSeq;
    for (const RecordedPoint & p : seg.points) {
      out << YAML::Flow << YAML::BeginMap;
      out << YAML::Key << "t" << YAML::Value << p.time;
      out << YAML::Key << "p" << YAML::Value;
      emit_numbers(out, p.positions);
      if (!p.velocities.empty()) {
        out << YAML::Key << "v" << YAML::Value;
        emit_numbers(out, p.velocities);
      }
      if (!p.accelerations.empty()) {
        out << YAML::Key << "a" << YAML::Value;
        emit_numbers(out, p.accelerations);
      }
      out << YAML::EndMap;
    }
    out << YAML::EndSeq;
    out << YAML::EndMap;
  }
  out << YAML::EndSeq;
  out << YAML::EndMap;

  if (!out.good()) {
    error = "YAML emitter: " + out.GetLastError();
    return false;
  }
  std::ofstream file(path);
  if (!file) {
    error = "cannot write " + path;
    return false;
  }
  file << out.c_str() << "\n";
  return static_cast<bool>(file);
}

bool load_recording(const std::string & path, ScanRecording & rec, std::string & error)
{
  try {
    YAML::Node root = YAML::LoadFile(path);
    rec = ScanRecording();
    rec.created           = root["created"].as<std::string>();
    rec.planning_group    = root["planning_group"].as<std::string>();
    rec.end_effector_link = root["end_effector_link"].as<std::string>();
    rec.global_frame      = root["global_frame"].as<std::string>();
    rec.center            = root["center"].as<std::vector<double>>();
    rec.radius            = root["radius"].as<double>();
    rec.waypoints_total   = root["waypoints_total"].as<int>();
    rec.waypoints_reached = root["waypoints_reached"].as<int>();
    rec.recoveries        = root["recoveries"].as<int>();

    for (const YAML::Node & s : root["segments"]) {
      RecordedSegment seg;
      seg.label       = s["label"].as<std::string>();
      seg.waypoint    = s["waypoint"].as<int>();
      seg.joint_names = s["joint_names"].as<std::vector<std::string>>();
      if (s["camera_pose"]) seg.camera_pose = s["camera_pose"].as<std::vector<double>>();
      for (const YAML::Node & p : s["points"]) {
        RecordedPoint point;
        point.time      = p["t"].as<double>();
        point.positions = p["p"].as<std::vector<double>>();
        if (p["v"]) point.velocities    = p["v"].as<std::vector<double>>();
        if (p["a"]) point.accelerations = p["a"].as<std::vector<double>>();
        if (point.positions.size() != seg.joint_names.size()) {
          error = "segment '" + seg.label + "': a point has the wrong number of joints";
          return false;
        }
        seg.points.push_back(point);
      }
      if (seg.points.empty()) {
        error = "segment '" + seg.label + "' has no points";
        return false;
      }
      rec.segments.push_back(seg);
    }
  } catch (const std::exception & e) {
    error = path + ": " + e.what();
    return false;
  }
  if (rec.segments.empty()) {
    error = path + ": no segments";
    return false;
  }
  return true;
}
