#pragma once

// Shared by scan_sequence_node and scan_replay_node.

#include <chrono>
#include <future>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>

// ============================================================================
// Planning scene locale
// ============================================================================
// Copia locale della planning scene di move_group (oggetti del mondo + matrice
// delle collisioni permesse). scan_sequence_node la usa per scartare le
// soluzioni IK in collisione PRIMA di chiamare plan() (TRAC-IK non conosce la
// scena); scan_replay_node per controllare i movimenti registrati prima di
// eseguirli. La scena e' statica durante lo scan: si legge una volta sola.
inline planning_scene::PlanningScenePtr fetch_planning_scene(
  rclcpp::Node::SharedPtr node,
  const moveit::core::RobotModelConstPtr & robot_model,
  const rclcpp::Logger & logger)
{
  using GetScene   = moveit_msgs::srv::GetPlanningScene;
  using Components = moveit_msgs::msg::PlanningSceneComponents;

  auto client = node->create_client<GetScene>("/get_planning_scene");
  if (!client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(logger,
      "Service /get_planning_scene non disponibile: check collisioni lato client DISABILITATO.");
    return nullptr;
  }

  auto request = std::make_shared<GetScene::Request>();
  request->components.components =
    Components::SCENE_SETTINGS |
    Components::ROBOT_STATE |
    Components::ROBOT_STATE_ATTACHED_OBJECTS |
    Components::WORLD_OBJECT_NAMES |
    Components::WORLD_OBJECT_GEOMETRY |
    Components::TRANSFORMS |
    Components::ALLOWED_COLLISION_MATRIX |
    Components::LINK_PADDING_AND_SCALING;

  // Il nodo e' gia' spinnato dal thread executor: basta aspettare il future.
  auto future = client->async_send_request(request);
  if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
    RCLCPP_ERROR(logger,
      "Timeout su /get_planning_scene: check collisioni lato client DISABILITATO.");
    return nullptr;
  }

  auto scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
  scene->setPlanningSceneMsg(future.get()->scene);
  RCLCPP_WARN(logger, "Planning scene locale caricata: %zu oggetti nel mondo.",
              scene->getWorld()->size());
  return scene;
}
