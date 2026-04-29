#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>

#include "ur_automata_scene/scan_scene_builder.hpp"

using namespace std::chrono_literals;

class ScenePublisherNode : public rclcpp::Node
{
public:
  ScenePublisherNode()
  : Node("scene_publisher_node")
  {
    declare_parameter<std::string>("global_frame", "world");
    declare_parameter<std::vector<double>>("scan_center", {0.0, 0.4, 0.5});

    client_ = create_client<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene");
  }

  bool run()
  {
    const std::string global_frame = get_parameter("global_frame").as_string();
    const std::vector<double> center_vec = get_parameter("scan_center").as_double_array();

    if (center_vec.size() != 3) {
      RCLCPP_FATAL(get_logger(), "scan_center deve avere 3 elementi, ne ha %zu", center_vec.size());
      return false;
    }

    const Eigen::Vector3d center(center_vec[0], center_vec[1], center_vec[2]);

    RCLCPP_INFO(get_logger(), "Attendo il service /apply_planning_scene ...");
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "Interrotto durante l'attesa del service.");
        return false;
      }
      RCLCPP_INFO(get_logger(), "Service non disponibile, riprovo ...");
    }

    auto scene = ur_automata_scene::build_scan_scene(global_frame, center, now());

    auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
    request->scene = scene;

    RCLCPP_INFO(
      get_logger(),
      "Applico la scena: %zu oggetti nel frame '%s', centro [%.3f, %.3f, %.3f]",
      scene.world.collision_objects.size(), global_frame.c_str(),
      center.x(), center.y(), center.z());

    auto future = client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(get_node_base_interface(), future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Chiamata al service /apply_planning_scene fallita.");
      return false;
    }

    if (!future.get()->success) {
      RCLCPP_ERROR(get_logger(), "/apply_planning_scene ha risposto success=false.");
      return false;
    }

    RCLCPP_INFO(get_logger(), "Scena applicata correttamente.");
    return true;
  }

private:
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ScenePublisherNode>();
  const bool ok = node->run();
  rclcpp::shutdown();
  return ok ? 0 : 1;
}
