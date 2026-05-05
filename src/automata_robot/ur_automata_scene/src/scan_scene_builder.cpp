#include "ur_automata_scene/scan_scene_builder.hpp"

#include <cmath>
#include <vector>

#include <Eigen/Geometry>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/object_color.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

namespace ur_automata_scene {

namespace {

// ------------------------------------------------------------------
// Build a quaternion that rotates the Z axis (0,0,1) so that it points
// from p1 toward p2. We use this to orient cylinder primitives, which
// in MoveIt are aligned along their local Z axis.
// ------------------------------------------------------------------
static geometry_msgs::msg::Quaternion rotation_z_to(const Eigen::Vector3d &p1,
                                                    const Eigen::Vector3d &p2) {
  Eigen::Vector3d direction = (p2 - p1).normalized();
  Eigen::Vector3d z_axis(0.0, 0.0, 1.0);
  double cos_angle = z_axis.dot(direction);

  geometry_msgs::msg::Quaternion result;
  if (cos_angle > 1.0 - 1e-9) {
    result.x = 0.0;
    result.y = 0.0;
    result.z = 0.0;
    result.w = 1.0;
    return result;
  }
  if (cos_angle < -1.0 + 1e-9) {
    result.x = 1.0;
    result.y = 0.0;
    result.z = 0.0;
    result.w = 0.0;
    return result;
  }

  Eigen::Vector3d axis = z_axis.cross(direction).normalized();
  double angle = std::acos(cos_angle);
  Eigen::AngleAxisd axis_angle(angle, axis);
  Eigen::Quaterniond q(axis_angle);

  result.x = q.x();
  result.y = q.y();
  result.z = q.z();
  result.w = q.w();
  return result;
}

// ------------------------------------------------------------------
// Identity quaternion: no rotation at all.
// ------------------------------------------------------------------
static geometry_msgs::msg::Quaternion identity_quat() {
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = 0.0;
  q.w = 1.0;
  return q;
}

// ------------------------------------------------------------------
// Wrap a SolidPrimitive into a CollisionObject
// ------------------------------------------------------------------
static moveit_msgs::msg::CollisionObject
make_obj(const std::string &id, const shape_msgs::msg::SolidPrimitive &shape,
         const Eigen::Vector3d &position,
         const geometry_msgs::msg::Quaternion &orientation,
         const std::string &frame, const rclcpp::Time &stamp) {
  moveit_msgs::msg::CollisionObject obj;
  obj.header.frame_id = frame;
  obj.header.stamp = stamp;
  obj.id = id;
  obj.operation = moveit_msgs::msg::CollisionObject::ADD;

  geometry_msgs::msg::Pose pose;
  pose.position.x = position.x();
  pose.position.y = position.y();
  pose.position.z = position.z();
  pose.orientation = orientation;

  obj.primitives.push_back(shape);
  obj.primitive_poses.push_back(pose);
  return obj;
}

// ------------------------------------------------------------------
// Load a mesh from a file path
// ------------------------------------------------------------------
static shape_msgs::msg::Mesh load_mesh_msg(const std::string &file_path) {
  shapes::Mesh *m = shapes::createMeshFromResource(file_path);
  shape_msgs::msg::Mesh mesh_msg;
  shapes::ShapeMsg shape_msg;
  if (m) {
    shapes::constructMsgFromShape(m, shape_msg);
    mesh_msg = boost::get<shape_msgs::msg::Mesh>(shape_msg);
    delete m;
  }
  return mesh_msg;
}

// ------------------------------------------------------------------
// Helper to build an ObjectColor entry.
// ------------------------------------------------------------------
static moveit_msgs::msg::ObjectColor
make_color(const std::string &id, float r, float g, float b, float a = 1.0f) {
  moveit_msgs::msg::ObjectColor oc;
  oc.id = id;
  oc.color.r = r;
  oc.color.g = g;
  oc.color.b = b;
  oc.color.a = a;
  return oc;
}

} // namespace

moveit_msgs::msg::PlanningScene
build_scan_scene(const std::string &global_frame, const Eigen::Vector3d &center,
                 const rclcpp::Time &stamp) {
  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;

  // ---------------- Table ----------------
  shape_msgs::msg::SolidPrimitive table_shape;
  table_shape.type = shape_msgs::msg::SolidPrimitive::BOX;
  table_shape.dimensions = {1.5, 1.5, 0.01};
  Eigen::Vector3d table_pos(0.0, 0.0, -0.02);
  moveit_msgs::msg::CollisionObject table = make_obj(
      "table", table_shape, table_pos, identity_quat(), global_frame, stamp);

  // ---------------- Left wall ----------------
  shape_msgs::msg::SolidPrimitive leftwall_shape;
  leftwall_shape.type = shape_msgs::msg::SolidPrimitive::BOX;
  leftwall_shape.dimensions = {0.01, 1.5, 1.5};
  Eigen::Vector3d leftwall_pos(-0.85, 0, 0.73);
  moveit_msgs::msg::CollisionObject leftwall =
      make_obj("leftwall", leftwall_shape, leftwall_pos, identity_quat(),
               global_frame, stamp);

  // ---------------- Right wall ----------------
  shape_msgs::msg::SolidPrimitive rightwall_shape;
  rightwall_shape.type = shape_msgs::msg::SolidPrimitive::BOX;
  rightwall_shape.dimensions = {0.01, 1.5, 1.5};
  Eigen::Vector3d rightwall_pos(0.85, 0, 0.73);
  moveit_msgs::msg::CollisionObject rightwall =
      make_obj("rightwall", rightwall_shape, rightwall_pos, identity_quat(),
               global_frame, stamp);

  // ---------------- Back wall ----------------
  shape_msgs::msg::SolidPrimitive backwall_shape;
  backwall_shape.type = shape_msgs::msg::SolidPrimitive::BOX;
  backwall_shape.dimensions = {1.5, 0.01, 1.5};
  Eigen::Vector3d backwall_pos(0, -0.50, 0.73);
  moveit_msgs::msg::CollisionObject backwall =
      make_obj("backwall", backwall_shape, backwall_pos, identity_quat(),
               global_frame, stamp);

  // ---------------- Support disk ----------------
  shape_msgs::msg::SolidPrimitive disk_shape;
  disk_shape.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  disk_shape.dimensions = {0.008, 0.15};
  moveit_msgs::msg::CollisionObject support = make_obj(
      "support", disk_shape, center, identity_quat(), global_frame, stamp);

  // ---------------- Target sphere (internal marker) ----------------
  shape_msgs::msg::SolidPrimitive sphere_shape;
  sphere_shape.type = shape_msgs::msg::SolidPrimitive::SPHERE;
  sphere_shape.dimensions = {0.01};
  moveit_msgs::msg::CollisionObject target =
      make_obj("support_center", sphere_shape, center, identity_quat(),
               global_frame, stamp);

  // ---------------- Support legs parameters ----------------
  const double leg_height = 0.36;
  const double leg1_height = 0.23;
  const double leg_radius = 0.02;
  const double table_z = -0.015;

  Eigen::Vector3d leg1_bottom(-0.40, 0.60, table_z);
  Eigen::Vector3d leg1_top(-0.40, 0.60, table_z + leg1_height);
  Eigen::Vector3d leg1_mid = (leg1_bottom + leg1_top) * 0.5;

  Eigen::Vector3d leg3_start(center.x(), center.y() + 0.15, center.z());
  double angle_45 = 45.0 * M_PI / 180.0;
  double angle_down = 5.0 * M_PI / 180.0;
  Eigen::Vector3d leg3_dir(-std::sin(angle_45) * std::cos(angle_down),
                           std::cos(angle_45) * std::cos(angle_down),
                           -std::sin(angle_down));
  Eigen::Vector3d leg3_end = leg3_start + leg_height * leg3_dir;
  Eigen::Vector3d leg3_mid = (leg3_start + leg3_end) * 0.5;

  // ---------------- Leg 1 ----------------
  shape_msgs::msg::SolidPrimitive leg1_shape;
  leg1_shape.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  leg1_shape.dimensions = {leg1_height, leg_radius};
  moveit_msgs::msg::CollisionObject leg1 =
      make_obj("leg1", leg1_shape, leg1_mid,
               rotation_z_to(leg1_bottom, leg1_top), global_frame, stamp);

  // ---------------- Leg 2 ----------------
  double leg2_length = (leg1_top - leg3_end).norm();
  shape_msgs::msg::SolidPrimitive leg2_shape;
  leg2_shape.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  leg2_shape.dimensions = {leg2_length, leg_radius};
  Eigen::Vector3d leg2_mid = (leg1_top + leg3_end) * 0.5;
  moveit_msgs::msg::CollisionObject leg2 =
      make_obj("leg2", leg2_shape, leg2_mid, rotation_z_to(leg3_end, leg1_top),
               global_frame, stamp);

  // ---------------- Leg 3 ----------------
  shape_msgs::msg::SolidPrimitive leg3_shape;
  leg3_shape.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  leg3_shape.dimensions = {leg_height, leg_radius};
  moveit_msgs::msg::CollisionObject leg3 =
      make_obj("leg3", leg3_shape, leg3_mid,
               rotation_z_to(leg3_start, leg3_end), global_frame, stamp);

  // ---------------- Mesh Artefact (OBJ) ----------------------
  std::string package_path =
      ament_index_cpp::get_package_share_directory("ur_automata_scene");
  std::string mesh_path =
      "file://" + package_path + "/meshes/ceramic_model.obj";

  moveit_msgs::msg::CollisionObject artefact;
  artefact.header.frame_id = global_frame;
  artefact.header.stamp = stamp;
  artefact.id = "artefact";
  artefact.operation = moveit_msgs::msg::CollisionObject::ADD;

  shape_msgs::msg::Mesh mesh_msg = load_mesh_msg(mesh_path);
  artefact.meshes.push_back(mesh_msg);

  geometry_msgs::msg::Pose mesh_pose;
  mesh_pose.position.x = center.x();
  mesh_pose.position.y = center.y();
  mesh_pose.position.z = center.z();
  mesh_pose.orientation = identity_quat();
  artefact.mesh_poses.push_back(mesh_pose);

  // ---------------- Assemble the scene ----------------
  scene.world.collision_objects.push_back(table);
  scene.world.collision_objects.push_back(support);
  scene.world.collision_objects.push_back(target);
  scene.world.collision_objects.push_back(leg1);
  scene.world.collision_objects.push_back(leg2);
  scene.world.collision_objects.push_back(leg3);
  scene.world.collision_objects.push_back(artefact);
  // scene.world.collision_objects.push_back(leftwall);
  // scene.world.collision_objects.push_back(rightwall);
  // scene.world.collision_objects.push_back(backwall);

  // ---------------- Colors ----------------
  scene.object_colors.push_back(make_color("table", 1.0f, 1.0f, 1.0f));
  scene.object_colors.push_back(make_color("support", 1.0f, 1.0f, 1.0f));
  scene.object_colors.push_back(make_color("support_center", 1.0f, 0.0f, 0.0f));
  scene.object_colors.push_back(make_color("leg1", 1.0f, 1.0f, 1.0f));
  scene.object_colors.push_back(make_color("leg2", 1.0f, 1.0f, 1.0f));
  scene.object_colors.push_back(make_color("leg3", 1.0f, 1.0f, 1.0f));
  scene.object_colors.push_back(
      make_color("artefact", 0.75f, 0.75f, 0.75f, 0.7f));
  scene.object_colors.push_back(make_color("leftwall", 1.0f, 1.0f, 1.0f));
  scene.object_colors.push_back(make_color("rightwall", 1.0f, 1.0f, 1.0f));
  scene.object_colors.push_back(make_color("backwall", 1.0f, 1.0f, 1.0f));

  return scene;
}

} // namespace ur_automata_scene
