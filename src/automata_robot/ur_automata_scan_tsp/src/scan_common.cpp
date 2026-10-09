// Copied from ur_automata_scan/src/scan_sequence_node.cpp (see scan_common.hpp).

#include "ur_automata_scan_tsp/scan_common.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <map>
#include <memory>
#include <set>

#include <geometric_shapes/bodies.h>
#include <geometric_shapes/body_operations.h>
#include <geometric_shapes/shapes.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>

std::vector<double> build_pitch_offsets_rad(double range_deg, double step_deg)
{
  std::vector<double> offsets;
  offsets.push_back(0.0);
  if (step_deg <= 0.0 || range_deg <= 0.0) return offsets;
  const double range_rad = range_deg * M_PI / 180.0;
  const double step_rad  = step_deg  * M_PI / 180.0;
  for (double k = step_rad; k <= range_rad + 1e-9; k += step_rad) {
    offsets.push_back(+k);
    offsets.push_back(-k);
  }
  return offsets;
}

// ---------------------------------------------------------------------------
// Line of sight camera -> center against the scene objects. If the segment
// crosses an object the photo shows the obstacle (e.g. the platform stem): the
// waypoint is dropped before any IK. Ignored: the center marker, the scanned
// object, the keep-out margins and cylinders and hits within 3 cm of the center.
// ---------------------------------------------------------------------------

// Moller-Trumbore ray / triangle intersection.
static bool ray_hits_triangle(
  const Eigen::Vector3d & origin, const Eigen::Vector3d & dir,
  const Eigen::Vector3d & v0, const Eigen::Vector3d & v1, const Eigen::Vector3d & v2,
  double & t_out)
{
  const double eps = 1e-9;
  Eigen::Vector3d e1 = v1 - v0;
  Eigen::Vector3d e2 = v2 - v0;
  Eigen::Vector3d p = dir.cross(e2);
  double det = e1.dot(p);
  if (std::abs(det) < eps) return false;
  double inv_det = 1.0 / det;
  Eigen::Vector3d s = origin - v0;
  double u = inv_det * s.dot(p);
  if (u < 0.0 || u > 1.0) return false;
  Eigen::Vector3d q = s.cross(e1);
  double v = inv_det * dir.dot(q);
  if (v < 0.0 || u + v > 1.0) return false;
  t_out = inv_det * e2.dot(q);
  return t_out > eps;
}

static bool is_scene_occluding(
  const planning_scene::PlanningScene & scene,
  const Eigen::Vector3d & from,
  const Eigen::Vector3d & center,
  std::string & hit_object)
{
  static const std::set<std::string> ignored_objects = {
    "support_center", "artefact", "table_margin",
    "keepout_stem_base", "keepout_stem", "keepout_disk", "keepout_support"};
  const double ignore_near_center = 0.03;   // m

  const Eigen::Vector3d seg = center - from;
  const double seg_len = seg.norm();
  if (seg_len < 1e-6) return false;
  const Eigen::Vector3d dir = seg / seg_len;
  auto hit_counts = [&](double t) { return t > 0.0 && t < seg_len - ignore_near_center; };

  collision_detection::WorldConstPtr world = scene.getWorld();
  for (const std::string & id : world->getObjectIds()) {
    if (ignored_objects.count(id) > 0) continue;
    collision_detection::World::ObjectConstPtr obj = world->getObject(id);
    if (!obj) continue;
    for (size_t k = 0; k < obj->shapes_.size(); ++k) {
      const shapes::Shape * shape = obj->shapes_[k].get();
      const Eigen::Isometry3d & pose = obj->global_shape_poses_[k];

      if (shape->type == shapes::MESH) {
        // Ray moved into the mesh frame: one transform instead of one per vertex.
        const auto * mesh = static_cast<const shapes::Mesh *>(shape);
        Eigen::Isometry3d inv = pose.inverse();
        Eigen::Vector3d o = inv * from;
        Eigen::Vector3d d = inv.linear() * dir;
        auto vertex = [&](unsigned int idx) {
          const double * v = mesh->vertices + 3 * idx;
          return Eigen::Vector3d(v[0], v[1], v[2]);
        };
        for (unsigned int tri = 0; tri < mesh->triangle_count; ++tri) {
          const unsigned int * idx = mesh->triangles + 3 * tri;
          double t = 0.0;
          if (ray_hits_triangle(o, d, vertex(idx[0]), vertex(idx[1]), vertex(idx[2]), t) &&
              hit_counts(t)) {
            hit_object = id;
            return true;
          }
        }
      } else {
        // Primitives (box, cylinder, sphere): raycast from geometric_shapes.
        std::unique_ptr<bodies::Body> body(bodies::createBodyFromShape(shape));
        if (!body) continue;
        body->setPose(pose);
        EigenSTL::vector_Vector3d points;
        if (!body->intersectsRay(from, dir, &points)) continue;
        for (const Eigen::Vector3d & p : points) {
          if (hit_counts((p - from).dot(dir))) {
            hit_object = id;
            return true;
          }
        }
      }
    }
  }
  return false;
}

// ---------------------------------------------------------------------------
// Platform visibility: sight lines from the camera to the disk center and 24
// rim points, as thin cylinders in a separate collision world, checked against
// the real arm collision meshes (end-effector links ignored).
// ---------------------------------------------------------------------------
static planning_scene::PlanningScenePtr make_sight_scene(const moveit::core::RobotModelConstPtr & model)
{
  auto sight_scene = std::make_shared<planning_scene::PlanningScene>(model);
  collision_detection::AllowedCollisionMatrix & acm = sight_scene->getAllowedCollisionMatrixNonConst();
  for (const std::string & link : model->getLinkModelNamesWithCollisionGeometry()) {
    if (link.find("ee_automata") != std::string::npos) acm.setEntry("sight", link, true);
  }
  return sight_scene;
}

static void set_sight_lines(planning_scene::PlanningScene & sight_scene,
                            const Eigen::Vector3d & camera, const Eigen::Vector3d & center,
                            double disk_radius, double margin)
{
  const int rim_points = 24;
  const double skip_near_camera = 0.02;   // m: the first 2 cm are inside the camera housing
  std::vector<shapes::ShapeConstPtr> lines;
  EigenSTL::vector_Isometry3d poses;
  for (int k = -1; k < rim_points; ++k) {   // k = -1: the disk center
    Eigen::Vector3d target = center;
    if (k >= 0) {
      double angle = 2.0 * M_PI * k / rim_points;
      target += disk_radius * Eigen::Vector3d(std::cos(angle), std::sin(angle), 0.0);
    }
    Eigen::Vector3d dir = target - camera;
    double length = dir.norm() - skip_near_camera;
    if (length < 0.01) continue;
    dir.normalize();
    lines.push_back(std::make_shared<shapes::Cylinder>(margin, length));
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = camera + dir * (skip_near_camera + length / 2.0);
    pose.linear() = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), dir).toRotationMatrix();
    poses.push_back(pose);
  }
  collision_detection::WorldPtr world = sight_scene.getWorldNonConst();
  world->removeObject("sight");
  world->addToObject("sight", lines, poses);
}

static bool is_platform_hidden(const planning_scene::PlanningScene & sight_scene,
                               const moveit::core::RobotState & state)
{
  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  sight_scene.getCollisionEnv()->checkRobotCollision(req, res, state, sight_scene.getAllowedCollisionMatrix());
  return res.collision;
}

geometry_msgs::msg::Pose make_lookat(const Eigen::Vector3d & pos, const Eigen::Vector3d & center)
{
  Eigen::Vector3d y_dir = (center - pos).normalized();
  Eigen::Vector3d ref   = (y_dir.cross(Eigen::Vector3d::UnitZ()).norm() > 1e-6)
                          ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitX();
  Eigen::Vector3d x_dir = y_dir.cross(ref).normalized();
  Eigen::Vector3d z_dir = x_dir.cross(y_dir);

  Eigen::Matrix3d rot;
  rot.col(0) = x_dir;
  rot.col(1) = y_dir;
  rot.col(2) = z_dir;
  Eigen::Quaterniond q(rot);

  geometry_msgs::msg::Pose pose;
  pose.position.x    = pos.x();
  pose.position.y    = pos.y();
  pose.position.z    = pos.z();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

// ---------------------------------------------------------------------------
// IK branch seeds
// ---------------------------------------------------------------------------
static bool is_duplicate(const std::vector<double> & joints, const std::vector<Candidate> & list)
{
  for (const auto & c : list) {
    double max_diff = 0.0;
    for (size_t k = 0; k < joints.size() && k < c.joints.size(); ++k) {
      max_diff = std::max(max_diff, std::abs(joints[k] - c.joints[k]));
    }
    if (max_diff < 1e-3) return true;
  }
  return false;
}

struct UrJointIndex { int pan = -1, lift = -1, elbow = -1, w1 = -1, w2 = -1, w3 = -1; };

static UrJointIndex find_ur_joint_indices(const moveit::core::JointModelGroup * jmg)
{
  UrJointIndex idx;
  const auto & names = jmg->getActiveJointModelNames();
  for (size_t i = 0; i < names.size(); ++i) {
    const std::string & n = names[i];
    int k = static_cast<int>(i);
    if      (n.find("shoulder_pan")  != std::string::npos) idx.pan   = k;
    else if (n.find("shoulder_lift") != std::string::npos) idx.lift  = k;
    else if (n.find("elbow")         != std::string::npos) idx.elbow = k;
    else if (n.find("wrist_1")       != std::string::npos) idx.w1    = k;
    else if (n.find("wrist_2")       != std::string::npos) idx.w2    = k;
    else if (n.find("wrist_3")       != std::string::npos) idx.w3    = k;
  }
  return idx;
}

static double wrap_pi(double a)
{
  while (a >  M_PI) a -= 2.0 * M_PI;
  while (a <= -M_PI) a += 2.0 * M_PI;
  return a;
}

// Eight TRAC-IK seeds from one configuration: wrist flipped / not, elbow up /
// down, shoulder left / right. Only starting points: TRAC-IK converges on the
// exact solution of the branch.
static std::vector<std::vector<double>> make_branch_seeds(
  const std::vector<double> & base, const UrJointIndex & j)
{
  std::vector<std::vector<double>> seeds;
  seeds.push_back(base);
  if (j.pan < 0 || j.lift < 0 || j.elbow < 0 || j.w1 < 0 || j.w2 < 0 || j.w3 < 0) return seeds;

  const double r = 0.92;   // forearm / upper arm ratio (a3/a2), about the same for all UR

  for (int mask = 1; mask < 8; ++mask) {
    std::vector<double> q = base;
    if (mask & 1) {   // wrist flipped
      q[j.w1] += M_PI;
      q[j.w2]  = -q[j.w2];
      q[j.w3] += M_PI;
    }
    if (mask & 2) {   // elbow mirrored about the shoulder-wrist line
      double e     = q[j.elbow];
      double gamma = std::atan2(r * std::sin(e), 1.0 + r * std::cos(e));
      q[j.lift]  += 2.0 * gamma;
      q[j.elbow]  = -e;
      q[j.w1]    += 2.0 * e - 2.0 * gamma;
    }
    if (mask & 4) {   // shoulder on the other side
      q[j.pan]  += M_PI;
      q[j.lift]  = -M_PI - q[j.lift];
      q[j.elbow] = -q[j.elbow];
      q[j.w1]    = -M_PI - q[j.w1];
      q[j.w2]    = -q[j.w2];
    }
    for (double & v : q) v = wrap_pi(v);
    seeds.push_back(q);
  }
  return seeds;
}

// ---------------------------------------------------------------------------
// Candidate enumeration (FASE 1 of scan_sequence_node)
// ---------------------------------------------------------------------------
EnumResult enumerate_candidates(const EnumSettings & s,
                                const std::vector<geometry_msgs::msg::Pose> & waypoints,
                                const moveit::core::RobotState & start_state,
                                const planning_scene::PlanningScenePtr & scene)
{
  EnumResult result;
  result.layers.resize(waypoints.size());
  result.view_blocked_by.resize(waypoints.size());
  const auto t0 = std::chrono::steady_clock::now();

  const moveit::core::JointModelGroup * jmg = start_state.getJointModelGroup(s.group);
  moveit::core::RobotState work_state(start_state);
  std::vector<double> start_joints;
  work_state.copyJointGroupPositions(jmg, start_joints);
  const UrJointIndex ur_idx = find_ur_joint_indices(jmg);

  // Sight lines depend only on the camera position: rebuilt when the TCP moves.
  planning_scene::PlanningScenePtr sight_scene = make_sight_scene(start_state.getRobotModel());
  Eigen::Vector3d sight_camera(1e9, 1e9, 1e9);
  auto platform_hidden = [&](const moveit::core::RobotState & st) -> bool {
    Eigen::Vector3d camera = st.getGlobalLinkTransform(s.ee_link).translation();
    if ((camera - sight_camera).norm() > 1e-3) {
      set_sight_lines(*sight_scene, camera, s.center, s.occlusion_disk_radius, s.occlusion_margin);
      sight_camera = camera;
    }
    return is_platform_hidden(*sight_scene, st);
  };
  auto candidate_ok = [&](const moveit::core::RobotState & st) -> bool {
    if (scene && scene->isStateColliding(st, s.group)) {
      ++result.collision_rejects;
      return false;
    }
    if (s.occlusion_check && platform_hidden(st)) {
      ++result.occlusion_rejects;
      return false;
    }
    return true;
  };

  // In the fallback search the smallest offsets are enough (0, +-1, +-2, +-3 steps).
  std::vector<double> fb_pitch_offsets(
    s.pitch_offsets.begin(),
    s.pitch_offsets.begin() + std::min<size_t>(7, s.pitch_offsets.size()));

  auto enumerate_pose = [&](const geometry_msgs::msg::Pose & base_pose, bool is_fallback,
                            const std::vector<std::vector<double>> & seeds,
                            const std::vector<double> & pitch_list,
                            std::vector<Candidate> & out, double ik_timeout_s)
  {
    Eigen::Quaterniond q_base(base_pose.orientation.w, base_pose.orientation.x,
                              base_pose.orientation.y, base_pose.orientation.z);
    for (double offset : pitch_list) {
      Eigen::Quaterniond q_try =
        q_base * Eigen::Quaterniond(Eigen::AngleAxisd(offset, Eigen::Vector3d::UnitY()));
      geometry_msgs::msg::Pose pose_try = base_pose;
      pose_try.orientation.w = q_try.w();
      pose_try.orientation.x = q_try.x();
      pose_try.orientation.y = q_try.y();
      pose_try.orientation.z = q_try.z();

      for (const auto & seed : seeds) {
        moveit::core::RobotState cand(work_state);
        cand.setJointGroupPositions(jmg, seed);
        cand.enforceBounds();
        if (!cand.setFromIK(jmg, pose_try, s.ee_link, ik_timeout_s)) continue;
        cand.update();
        if (!candidate_ok(cand)) continue;

        Candidate c;
        cand.copyJointGroupPositions(jmg, c.joints);
        c.pitch_offset_rad = offset;
        c.pose = pose_try;
        c.is_fallback = is_fallback;
        if (is_duplicate(c.joints, out)) continue;
        out.push_back(c);
      }
    }
  };

  const std::vector<std::vector<double>> home_seeds = make_branch_seeds(start_joints, ur_idx);
  std::vector<std::vector<double>> prev_seeds;   // solutions of the previous waypoint

  std::printf("Enumerazione candidati (%zu waypoint, %zu pitch x max 8 seed) ...\n",
              waypoints.size(), s.pitch_offsets.size());
  for (size_t i = 0; i < waypoints.size(); ++i) {
    if (!rclcpp::ok()) break;
    const std::vector<std::vector<double>> & seeds = prev_seeds.empty() ? home_seeds : prev_seeds;
    const Eigen::Vector3d wp_pos(waypoints[i].position.x, waypoints[i].position.y, waypoints[i].position.z);

    if (scene) {
      std::string hit_object;
      if (is_scene_occluding(*scene, wp_pos, s.center, hit_object)) {
        result.view_blocked_by[i] = hit_object;
        std::printf("  wp %3zu: vista coperta da '%s'  <-- SCARTATO\n", i, hit_object.c_str());
        continue;
      }
    }

    std::vector<Candidate> & layer = result.layers[i];
    enumerate_pose(waypoints[i], false, seeds, s.pitch_offsets, layer, s.ik_timeout);

    // Near the reach limit the solver may miss existing solutions in 3 ms:
    // retry with a 20x timeout, also from the 8 branches of the start pose.
    const double slow_ik_timeout = s.ik_timeout * 20.0;
    std::vector<std::vector<double>> retry_seeds = seeds;
    if (!prev_seeds.empty()) retry_seeds.insert(retry_seeds.end(), home_seeds.begin(), home_seeds.end());
    if (layer.empty()) {
      enumerate_pose(waypoints[i], false, retry_seeds, s.pitch_offsets, layer, slow_ik_timeout);
    }

    if (layer.empty() && s.fallback_search) {
      // No valid configuration: nearby points on the sphere (2 rings x 6 directions).
      double fallback_angle_max = (s.fallback_radius_mm / 1000.0) / s.radius;
      Eigen::Vector3d radial = (wp_pos - s.center).normalized();
      Eigen::Vector3d ref_ax = (std::abs(radial.dot(Eigen::Vector3d::UnitZ())) < 0.9)
                               ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitX();
      Eigen::Vector3d tang_u = radial.cross(ref_ax).normalized();
      Eigen::Vector3d tang_v = radial.cross(tang_u);
      for (int ring = 1; ring <= 2; ++ring) {
        double angle_off = fallback_angle_max * ring / 2.0;
        for (int d = 0; d < 6; ++d) {
          double dir_angle = d * (2.0 * M_PI / 6.0);
          Eigen::Vector3d tangent = std::cos(dir_angle) * tang_u + std::sin(dir_angle) * tang_v;
          Eigen::Vector3d new_radial =
            (std::cos(angle_off) * radial + std::sin(angle_off) * tangent).normalized();
          Eigen::Vector3d cand_pos = s.center + s.radius * new_radial;
          std::string hit_object;
          if (scene && is_scene_occluding(*scene, cand_pos, s.center, hit_object)) continue;
          enumerate_pose(make_lookat(cand_pos, s.center), true, retry_seeds, fb_pitch_offsets,
                         layer, slow_ik_timeout);
        }
      }
    }

    if (!layer.empty()) {
      prev_seeds.clear();
      for (size_t k = 0; k < layer.size() && k < 8; ++k) prev_seeds.push_back(layer[k].joints);
    }
    result.total_candidates += layer.size();
    if (layer.empty()) std::printf("  wp %3zu: nessuna configurazione valida  <-- IRRAGGIUNGIBILE\n", i);
  }
  result.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  std::fflush(stdout);
  return result;
}

bool segment_is_free(const planning_scene::PlanningScenePtr & scene,
                     const moveit::core::RobotState & reference,
                     const moveit::core::JointModelGroup * jmg,
                     const std::vector<double> & a, const std::vector<double> & b)
{
  if (!scene) return true;
  double max_diff = 0.0;
  for (size_t k = 0; k < a.size() && k < b.size(); ++k) {
    max_diff = std::max(max_diff, std::abs(a[k] - b[k]));
  }
  int steps = std::max(1, static_cast<int>(std::ceil(max_diff / 0.05)));
  moveit::core::RobotState from(reference), to(reference), mid(reference);
  from.setJointGroupPositions(jmg, a);
  to.setJointGroupPositions(jmg, b);
  for (int k = 1; k < steps; ++k) {
    from.interpolate(to, static_cast<double>(k) / steps, mid, jmg);
    mid.update();
    if (scene->isStateColliding(mid, jmg->getName())) return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
// Planner chain
// ---------------------------------------------------------------------------
bool resolve_planner(const std::string & name, const std::string & ompl_algorithm, PlannerChoice & out)
{
  if (name == "ompl") {
    out = {"ompl", ompl_algorithm + "kConfigDefault"};
  } else if (name == "pilz_ptp") {
    out = {"pilz_industrial_motion_planner", "PTP"};
  } else if (name == "pilz_lin") {
    out = {"pilz_industrial_motion_planner", "LIN"};
  } else if (name == "pilz_circ") {
    out = {"pilz_industrial_motion_planner", "CIRC", true};
  } else if (name == "stomp") {
    out = {"stomp", "stomp"};
  } else {
    return false;
  }
  return true;
}

// Pilz CIRC reads only the position of the "center" constraint; the region is
// left without primitives on purpose, so ValidateSolution ignores it.
moveit_msgs::msg::Constraints make_circ_center_constraint(
  const std::string & frame, const std::string & link, const Eigen::Vector3d & center)
{
  moveit_msgs::msg::Constraints constraints;
  constraints.name = "center";

  moveit_msgs::msg::PositionConstraint pc;
  pc.header.frame_id = frame;
  pc.link_name = link;
  pc.weight = 1.0;

  geometry_msgs::msg::Pose center_pose;
  center_pose.position.x = center.x();
  center_pose.position.y = center.y();
  center_pose.position.z = center.z();
  center_pose.orientation.w = 1.0;
  pc.constraint_region.primitive_poses.push_back(center_pose);

  constraints.position_constraints.push_back(pc);
  return constraints;
}

static std::string plan_error_name(int code)
{
  using Codes = moveit_msgs::msg::MoveItErrorCodes;
  switch (code) {
    case Codes::FAILURE:                              return "FAILURE (generico: vedi il terminale del bringup)";
    case Codes::PLANNING_FAILED:                      return "PLANNING_FAILED";
    case Codes::INVALID_MOTION_PLAN:                  return "INVALID_MOTION_PLAN";
    case Codes::TIMED_OUT:                            return "TIMED_OUT";
    case Codes::START_STATE_IN_COLLISION:             return "START_STATE_IN_COLLISION";
    case Codes::START_STATE_VIOLATES_PATH_CONSTRAINTS: return "START_STATE_VIOLATES_PATH_CONSTRAINTS";
    case Codes::GOAL_IN_COLLISION:                    return "GOAL_IN_COLLISION";
    case Codes::GOAL_VIOLATES_PATH_CONSTRAINTS:       return "GOAL_VIOLATES_PATH_CONSTRAINTS";
    case Codes::GOAL_CONSTRAINTS_VIOLATED:            return "GOAL_CONSTRAINTS_VIOLATED";
    case Codes::INVALID_GOAL_CONSTRAINTS:             return "INVALID_GOAL_CONSTRAINTS";
    case Codes::INVALID_ROBOT_STATE:                  return "INVALID_ROBOT_STATE";
    default:                                          return "codice " + std::to_string(code);
  }
}

// True if the last trajectory point is the joint target (CIRC may end on a
// different IK branch than the one chosen by the DP).
static bool trajectory_ends_at_target(
  const moveit::planning_interface::MoveGroupInterface::Plan & plan,
  const moveit::planning_interface::MoveGroupInterface & move_group)
{
  const auto & jt = plan.trajectory.joint_trajectory;
  if (jt.points.empty()) return false;
  const auto & last = jt.points.back().positions;
  if (last.size() != jt.joint_names.size()) return false;

  const moveit::core::JointModelGroup * jmg =
    move_group.getRobotModel()->getJointModelGroup(move_group.getName());
  if (jmg == nullptr) return false;

  std::vector<double> target_values;
  move_group.getJointValueTarget(target_values);
  const std::vector<std::string> & target_names = jmg->getVariableNames();
  if (target_values.size() != target_names.size()) return false;

  std::map<std::string, double> target;
  for (size_t k = 0; k < target_names.size(); ++k) target[target_names[k]] = target_values[k];

  for (size_t k = 0; k < jt.joint_names.size(); ++k) {
    auto it = target.find(jt.joint_names[k]);
    if (it == target.end()) continue;
    if (std::abs(wrap_pi(last[k] - it->second)) > 0.01) return false;
  }
  return true;
}

bool plan_with_fallback(moveit::planning_interface::MoveGroupInterface & move_group,
                        moveit::planning_interface::MoveGroupInterface::Plan & plan,
                        const PlannerSetup & planners, bool allow_circ,
                        const rclcpp::Logger & logger, std::string * used_label)
{
  for (const PlannerChoice & choice : planners.chain) {
    if (choice.is_circ && !allow_circ) continue;

    move_group.setPlanningPipelineId(choice.pipeline);
    move_group.setPlannerId(choice.planner);
    if (choice.is_circ) move_group.setPathConstraints(planners.circ_center);

    moveit::core::MoveItErrorCode code = move_group.plan(plan);
    if (choice.is_circ) move_group.clearPathConstraints();

    if (code != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_WARN(logger, "plan con %s fallito: %s",
                  choice.label().c_str(), plan_error_name(code.val).c_str());
      continue;
    }
    if (choice.is_circ && !trajectory_ends_at_target(plan, move_group)) {
      RCLCPP_WARN(logger, "plan con %s scartato: l'arco termina su un ramo IK diverso dal target",
                  choice.label().c_str());
      continue;
    }
    if (used_label) *used_label = choice.label();
    return true;
  }
  return false;
}
