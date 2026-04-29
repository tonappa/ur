#include "ur_automata_scene/scan_scene_builder.hpp"

#include <vector>

#include <Eigen/Geometry>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene_world.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

namespace ur_automata_scene {

namespace {

geometry_msgs::msg::Quaternion identity_quaternion()
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = 0.0;
  q.w = 1.0;
  return q;
}

moveit_msgs::msg::CollisionObject make_collision_object(
    const std::string & id,
    const shape_msgs::msg::SolidPrimitive & primitive,
    const Eigen::Vector3d & position,
    const std::string & frame,
    const rclcpp::Time & stamp,
    const geometry_msgs::msg::Quaternion & orientation)
{
  moveit_msgs::msg::CollisionObject co;
  co.header.frame_id = frame;
  co.header.stamp = stamp;
  co.id = id;

  co.primitives.push_back(primitive);

  geometry_msgs::msg::Pose pose;
  pose.position.x = position.x();
  pose.position.y = position.y();
  pose.position.z = position.z();
  pose.orientation = orientation;
  co.primitive_poses.push_back(pose);

  co.operation = moveit_msgs::msg::CollisionObject::ADD;
  return co;
}

}  // namespace

moveit_msgs::msg::PlanningScene build_scan_scene(
    const std::string & global_frame,
    const Eigen::Vector3d & center,
    const rclcpp::Time & stamp)
{
  // 1) Tavolo
  shape_msgs::msg::SolidPrimitive table_box;
  table_box.type = shape_msgs::msg::SolidPrimitive::BOX;
  table_box.dimensions = {1.5, 1.5, 0.01};
  auto table = make_collision_object(
      "table", table_box, Eigen::Vector3d(0.0, 0.0, -0.02),
      global_frame, stamp, identity_quaternion());

  // 2) Disco di supporto sotto il target dello scan.
  shape_msgs::msg::SolidPrimitive support_cyl;
  support_cyl.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  support_cyl.dimensions = {0.004, 0.15};  // [altezza, raggio]
  auto support = make_collision_object(
      "support", support_cyl, center,
      global_frame, stamp, identity_quaternion());

  // 3) Gamba 1 del supporto, verticale.
  shape_msgs::msg::SolidPrimitive support_leg1;
  support_leg1.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  support_leg1.dimensions = {0.24, 0.011};  // [altezza, raggio]
  const double leg1_x = -0.38;
  const double leg1_y = 0.535 - 0.09;
  const double leg1_z = -0.02 + 0.12;
  Eigen::Vector3d leg1_position(leg1_x, leg1_y, leg1_z);
  auto support_leg1_obj = make_collision_object(
      "support_leg1", support_leg1, leg1_position,
      global_frame, stamp, identity_quaternion());

  // 4) Gamba 2: dal top di leg1 al punto piu' vicino sulla circonferenza del disco.
  Eigen::Vector3d p1(leg1_x, leg1_y, leg1_z + support_leg1.dimensions[0] / 2.0);

  Eigen::Vector3d v_proj(p1.x() - center.x(), p1.y() - center.y(), 0.0);
  const double v_proj_len = v_proj.norm();
  const double radius = support_cyl.dimensions[1];
  Eigen::Vector3d q_closest;
  if (v_proj_len > 1e-6) {
    q_closest = center + (v_proj / v_proj_len) * radius;
  } else {
    q_closest = center + Eigen::Vector3d(radius, 0.0, 0.0);
  }

  Eigen::Vector3d vec = q_closest - p1;
  Eigen::Vector3d z_dir = vec.normalized();

  const double leg2_len = 0.38;
  shape_msgs::msg::SolidPrimitive support_leg2;
  support_leg2.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  support_leg2.dimensions = {leg2_len, 0.011};

  Eigen::Vector3d leg2_center = p1 + z_dir * (leg2_len / 2.0);

  // Costruzione della rotazione: allinea l'asse Z del cilindro a z_dir.
  Eigen::Vector3d x_dir = Eigen::Vector3d(0.0, 1.0, 0.0).cross(z_dir);
  if (x_dir.norm() < 1e-6) {
    x_dir = Eigen::Vector3d(1.0, 0.0, 0.0).cross(z_dir);
  }
  x_dir.normalize();
  Eigen::Vector3d y_dir = z_dir.cross(x_dir);

  Eigen::Matrix3d rot_mat;
  rot_mat.col(0) = x_dir;
  rot_mat.col(1) = y_dir;
  rot_mat.col(2) = z_dir;
  Eigen::Quaterniond q(rot_mat);

  geometry_msgs::msg::Quaternion leg2_ori;
  leg2_ori.x = q.x();
  leg2_ori.y = q.y();
  leg2_ori.z = q.z();
  leg2_ori.w = q.w();

  auto support_leg2_obj = make_collision_object(
      "support_leg2", support_leg2, leg2_center,
      global_frame, stamp, leg2_ori);

  // 5) Sfera marker al centro dello scan (oggetto da ispezionare).
  shape_msgs::msg::SolidPrimitive center_sphere;
  center_sphere.type = shape_msgs::msg::SolidPrimitive::SPHERE;
  center_sphere.dimensions = {0.02};  // [raggio]
  auto support_center = make_collision_object(
      "support_center", center_sphere, center,
      global_frame, stamp, identity_quaternion());

  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;
  scene.world.collision_objects = {
      table, support, support_leg1_obj, support_leg2_obj, support_center};

  return scene;
}

}  // namespace ur_automata_scene
