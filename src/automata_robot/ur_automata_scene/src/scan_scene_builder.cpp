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
static shape_msgs::msg::Mesh load_mesh_msg(const std::string &file_path,
                                           double scale = 1.0) {
  Eigen::Vector3d scale_vec(scale, scale, scale);
  shapes::Mesh *m = shapes::createMeshFromResource(file_path, scale_vec);
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
                 const rclcpp::Time &stamp, bool platform_sim) {
  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;

  // ---------------- Table ----------------
  shape_msgs::msg::SolidPrimitive table_shape;
  table_shape.type = shape_msgs::msg::SolidPrimitive::BOX;
  // Spessore 0.10 m, centro a z=-0.05 → faccia superiore a z=0.0 (piano base
  // del robot). Blocca il gomito che nelle pose laterali basse scende sotto
  // z=0.
  table_shape.dimensions = {1.5, 1.5, 0.10};
  Eigen::Vector3d table_pos(0.0, 0.0, -0.05);
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

  // ---------------- Target sphere (internal marker) ----------------
  shape_msgs::msg::SolidPrimitive sphere_shape;
  sphere_shape.type = shape_msgs::msg::SolidPrimitive::SPHERE;
  sphere_shape.dimensions = {0.01};
  moveit_msgs::msg::CollisionObject target =
      make_obj("support_center", sphere_shape, center, identity_quat(),
               global_frame, stamp);

  // Package share path usato sia per platform STL che per artefact OBJ
  std::string package_path =
      ament_index_cpp::get_package_share_directory("ur_automata_scene");

  // ---------------- Platform (modalità SIM = disco + 3 gambe) -----------
  // Variabili dichiarate qui per essere visibili nella sezione "Assemble".
  moveit_msgs::msg::CollisionObject support, leg1, leg2, leg3;
  // ---------------- Platform (modalità STL = mesh reale) ----------------
  moveit_msgs::msg::CollisionObject platform;

  if (platform_sim) {
    // Disco di supporto al centro
    shape_msgs::msg::SolidPrimitive disk_shape;
    disk_shape.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    disk_shape.dimensions = {0.008, 0.15};
    support = make_obj("support", disk_shape, center, identity_quat(),
                       global_frame, stamp);

    // Parametri delle 3 gambe
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

    shape_msgs::msg::SolidPrimitive leg1_shape;
    leg1_shape.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    leg1_shape.dimensions = {leg1_height, leg_radius};
    leg1 = make_obj("leg1", leg1_shape, leg1_mid,
                    rotation_z_to(leg1_bottom, leg1_top), global_frame, stamp);

    double leg2_length = (leg1_top - leg3_end).norm();
    shape_msgs::msg::SolidPrimitive leg2_shape;
    leg2_shape.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    leg2_shape.dimensions = {leg2_length, leg_radius};
    Eigen::Vector3d leg2_mid = (leg1_top + leg3_end) * 0.5;
    leg2 = make_obj("leg2", leg2_shape, leg2_mid,
                    rotation_z_to(leg3_end, leg1_top), global_frame, stamp);

    shape_msgs::msg::SolidPrimitive leg3_shape;
    leg3_shape.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    leg3_shape.dimensions = {leg_height, leg_radius};
    leg3 = make_obj("leg3", leg3_shape, leg3_mid,
                    rotation_z_to(leg3_start, leg3_end), global_frame, stamp);
  } else {
    // Mesh STL della piattaforma reale, posizionata in modo che la faccia
    // superiore del disco coincida con `center` (vedi platform_pose sotto).
    std::string platform_mesh_path =
        "file://" + package_path + "/meshes/disk.stl";

    platform.header.frame_id = global_frame;
    platform.header.stamp = stamp;
    platform.id = "platform";
    platform.operation = moveit_msgs::msg::CollisionObject::ADD;

    // STL esportato in mm → scale 0.001 per convertire in metri
    shape_msgs::msg::Mesh platform_mesh =
        load_mesh_msg(platform_mesh_path, 0.001);
    platform.meshes.push_back(platform_mesh);

    // Rotazione composta: +90° attorno X, poi -90° attorno Z (entrambi nel
    // frame world). Con questa rotazione, e l'STL scalato in metri, il disco
    // (Ø 300 mm, spessore 4 mm) risulta centrato in (x, y - 0.20) e con la
    // faccia superiore a z + 0.504 rispetto all'origine della mesh. La posa e'
    // quindi ricavata da `center` cosi' che il piano di appoggio coincida con
    // il centro di scansione: spostare la piattaforma = cambiare scan.center.
    geometry_msgs::msg::Pose platform_pose;
    platform_pose.position.x = center.x();
    platform_pose.position.y = center.y() + 0.20;
    platform_pose.position.z = center.z() - 0.504;
    Eigen::Quaterniond q_rot =
        Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX());
    platform_pose.orientation.x = q_rot.x();
    platform_pose.orientation.y = q_rot.y();
    platform_pose.orientation.z = q_rot.z();
    platform_pose.orientation.w = q_rot.w();
    platform.mesh_poses.push_back(platform_pose);
  }

  // ---------------- Mesh Artefact (OBJ) ----------------------
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
  scene.world.collision_objects.push_back(target);
  scene.world.collision_objects.push_back(artefact);

  if (platform_sim) {
    scene.world.collision_objects.push_back(support);
    scene.world.collision_objects.push_back(leg1);
    scene.world.collision_objects.push_back(leg2);
    scene.world.collision_objects.push_back(leg3);
  } else {
    scene.world.collision_objects.push_back(platform);
  }
  // scene.world.collision_objects.push_back(leftwall);
  // scene.world.collision_objects.push_back(rightwall);
  // scene.world.collision_objects.push_back(backwall);

  // ---------------- Colors ----------------
  scene.object_colors.push_back(make_color("table", 1.0f, 1.0f, 1.0f));
  scene.object_colors.push_back(make_color("support_center", 1.0f, 0.0f, 0.0f));
  scene.object_colors.push_back(
      make_color("artefact", 0.75f, 0.75f, 0.75f, 0.7f));
  if (platform_sim) {
    scene.object_colors.push_back(make_color("support", 1.0f, 1.0f, 1.0f));
    scene.object_colors.push_back(make_color("leg1", 1.0f, 1.0f, 1.0f));
    scene.object_colors.push_back(make_color("leg2", 1.0f, 1.0f, 1.0f));
    scene.object_colors.push_back(make_color("leg3", 1.0f, 1.0f, 1.0f));
  } else {
    scene.object_colors.push_back(make_color("platform", 0.8f, 0.8f, 0.85f));
  }
  scene.object_colors.push_back(make_color("leftwall", 1.0f, 1.0f, 1.0f));
  scene.object_colors.push_back(make_color("rightwall", 1.0f, 1.0f, 1.0f));
  scene.object_colors.push_back(make_color("backwall", 1.0f, 1.0f, 1.0f));

  return scene;
}

} // namespace ur_automata_scene
