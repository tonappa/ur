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

    // Parameter: true = disco + 3 gambe (simulato), false = mesh STL della piattaforma reale
    declare_parameter<bool>("platform_sim", true);

    // Parameters: keep-out di margine in metri (0 = nessun oggetto)
    declare_parameter<double>("platform_margin", 0.0);
    declare_parameter<double>("table_margin", 0.0);

    // Parameters: muri della cella in metri nel frame globale (0 = nessun muro)
    declare_parameter<double>("wall_back_y", 0.0);
    declare_parameter<double>("wall_left_x", 0.0);
    declare_parameter<double>("wall_right_x", 0.0);
    // Posa della base (robot.base_xyz / base_rpy): se non e' zero, parete di montaggio.
    declare_parameter<std::vector<double>>("base_xyz", std::vector<double>{0.0, 0.0, 0.0});
    declare_parameter<std::vector<double>>("base_rpy", std::vector<double>{0.0, 0.0, 0.0});

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
    ur_automata_scene::SceneOptions opt;
    opt.platform_sim    = get_parameter("platform_sim").as_bool();
    opt.platform_margin = get_parameter("platform_margin").as_double();
    opt.table_margin    = get_parameter("table_margin").as_double();
    opt.wall_back_y     = get_parameter("wall_back_y").as_double();
    opt.wall_left_x     = get_parameter("wall_left_x").as_double();
    opt.wall_right_x    = get_parameter("wall_right_x").as_double();
    std::vector<double> base_xyz = get_parameter("base_xyz").as_double_array();
    std::vector<double> base_rpy = get_parameter("base_rpy").as_double_array();
    if (base_xyz.size() != 3 || base_rpy.size() != 3) {
      RCLCPP_FATAL(get_logger(), "base_xyz and base_rpy must contain 3 values each");
      return false;
    }
    opt.base_xyz = Eigen::Vector3d(base_xyz[0], base_xyz[1], base_xyz[2]);
    opt.base_rpy = Eigen::Vector3d(base_rpy[0], base_rpy[1], base_rpy[2]);

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
        ur_automata_scene::build_scan_scene(global_frame, center, now(), opt);

    // ---------------- Build the service request ----------------
    std::shared_ptr<moveit_msgs::srv::ApplyPlanningScene::Request> request =
        std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
    request->scene = scene;

    RCLCPP_INFO(
      get_logger(),
      "Applying scene: %zu objects in frame '%s', center [%.3f, %.3f, %.3f], "
      "margins platform %.3f / table %.3f m, walls back %.2f left %.2f right %.2f, "
      "base [%.3f, %.3f, %.3f] rpy [%.3f, %.3f, %.3f]%s",
      scene.world.collision_objects.size(), global_frame.c_str(),
      center.x(), center.y(), center.z(), opt.platform_margin, opt.table_margin,
      opt.wall_back_y, opt.wall_left_x, opt.wall_right_x,
      opt.base_xyz.x(), opt.base_xyz.y(), opt.base_xyz.z(),
      opt.base_rpy.x(), opt.base_rpy.y(), opt.base_rpy.z(),
      (opt.base_xyz.isZero() && opt.base_rpy.isZero()) ? "" : " (parete di montaggio)");

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
