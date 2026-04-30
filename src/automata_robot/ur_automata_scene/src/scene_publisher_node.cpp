#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>

#include "ur_automata_scene/scan_scene_builder.hpp"

// ------------------------------------------------------------------
// ROS 2 node that builds a planning scene and applies it to MoveIt
// using the /apply_planning_scene service.
// ------------------------------------------------------------------
class ScenePublisherNode : public rclcpp::Node
{
public:
  ScenePublisherNode()
  : Node("scene_publisher_node")
  {
    // Parameter: frame in which the scene is expressed (default: "world")
    declare_parameter<std::string>("global_frame", "world");

    // Parameter: scan center (xyz), used to position the support and the target
    std::vector<double> default_center = {0.0, 0.4, 0.5};
    declare_parameter<std::vector<double>>("scan_center", default_center);

    // Create the service client we will call later
    client_ = create_client<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene");
  }

  // Build the scene and push it to MoveIt. Returns true on success.
  bool run()
  {
    // ---------------- Read parameters ----------------
    std::string global_frame = get_parameter("global_frame").as_string();
    std::vector<double> center_vec = get_parameter("scan_center").as_double_array();

    // The scan_center parameter must contain exactly 3 numbers (x, y, z)
    if (center_vec.size() != 3) {
      RCLCPP_FATAL(get_logger(),
        "scan_center must contain 3 values, got %zu", center_vec.size());
      return false;
    }

    Eigen::Vector3d center(center_vec[0], center_vec[1], center_vec[2]);

    // ---------------- Wait for the MoveIt service ----------------
    RCLCPP_INFO(get_logger(), "Waiting for /apply_planning_scene service ...");
    std::chrono::seconds wait_period(1);
    while (!client_->wait_for_service(wait_period)) {
      // If ROS is shutting down while we wait, abort
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service.");
        return false;
      }
      RCLCPP_INFO(get_logger(), "Service not available yet, retrying ...");
    }

    // ---------------- Build the scene ----------------
    moveit_msgs::msg::PlanningScene scene =
        ur_automata_scene::build_scan_scene(global_frame, center, now());

    // ---------------- Build the service request ----------------
    std::shared_ptr<moveit_msgs::srv::ApplyPlanningScene::Request> request =
        std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
    request->scene = scene;

    RCLCPP_INFO(
      get_logger(),
      "Applying scene: %zu objects in frame '%s', center [%.3f, %.3f, %.3f]",
      scene.world.collision_objects.size(), global_frame.c_str(),
      center.x(), center.y(), center.z());

    // ---------------- Call the service and wait for the response ----------------
    auto future = client_->async_send_request(request);
    rclcpp::FutureReturnCode wait_result =
        rclcpp::spin_until_future_complete(get_node_base_interface(), future);

    if (wait_result != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Call to /apply_planning_scene failed.");
      return false;
    }

    // The service replies with a success flag — check it
    if (!future.get()->success) {
      RCLCPP_ERROR(get_logger(), "/apply_planning_scene returned success=false.");
      return false;
    }

    RCLCPP_INFO(get_logger(), "Scene applied successfully.");
    return true;
  }

private:
  // Service client used to push the scene to MoveIt
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr client_;
};

// ------------------------------------------------------------------
// Entry point: init ROS, run the node once, shutdown.
// ------------------------------------------------------------------
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<ScenePublisherNode> node = std::make_shared<ScenePublisherNode>();
  bool ok = node->run();
  rclcpp::shutdown();
  return ok ? 0 : 1;
}
