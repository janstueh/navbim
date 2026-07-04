#ifndef NAVBIM_GPP_BIM__ROOM_PLANNER_SERVER_HPP_
#define NAVBIM_GPP_BIM__ROOM_PLANNER_SERVER_HPP_

#include <chrono>
#include <string>
#include <memory>
#include <vector>
#include <unordered_map>
#include <mutex>
#include <functional>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/simple_action_server.hpp"

#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/create_timer_ros.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "navbim_multi_costmap_2d/multi_costmap_2d_ros.hpp"
#include "nav2_util/node_thread.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_core/global_planner.hpp"
#include "navbim_msgs/action/navbim_compute_path_to_pose_in_room.hpp"
#include "navbim_msgs/srv/smooth_path_with_floor_costmaps.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_core/planner_exceptions.hpp"
#include "nav2_core/smoother.hpp"
#include "navbim_gpp_bim/navfn_planner_wrapper.hpp"
#include "navbim_gpp_bim/theta_star_planner_wrapper.hpp"
#include "navbim_gpp_bim/ompl_planner_wrapper.hpp"
#include "navbim_smoother/simple_smoother.hpp"
#include "navbim_smoother/savitzky_golay_smoother.hpp"

namespace navbim_gpp_bim
{
/**
 * @class navbim_gpp_bim::RoomPlannerServer
 * @brief An action server that handles multi-room path planning using different
 * costmaps for each room, with support for various navigation algorithms.
 */
class RoomPlannerServer : public nav2_util::LifecycleNode
{
public:
  /**
   * @brief A constructor for navbim_gpp_bim::RoomPlannerServer
   * @param options Additional options to control creation of the node.
   */
  explicit RoomPlannerServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /**
   * @brief A destructor for navbim_gpp_bim::RoomPlannerServer
   */
  ~RoomPlannerServer();

  /**
   * @brief Set the costmap pointer (called after both nodes are loaded in composition mode)
   * @param costmap_ros Shared pointer to the MultiCostmap2DROS node
   */
  void setCostmap(std::shared_ptr<navbim_multi_costmap_2d::MultiCostmap2DROS> costmap_ros);

  using PlannerMap = std::unordered_map<std::string, nav2_core::GlobalPlanner::Ptr>;

  /**
   * @brief Method to get plan from the desired plugin
   * @param start starting pose
   * @param goal goal request
   * @param planner_id The planner to plan with
   * @param cancel_checker A function to check if the action has been canceled
   * @return Path
   */
  nav_msgs::msg::Path getPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const std::string & planner_id,
    std::function<bool()> cancel_checker);

protected:
  /**
   * @brief Configure member variables and initializes planner
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  
  /**
   * @brief Service callback for floor smoothing
   * @param request Service request with path and floor names
   * @param response Service response with smoothed path
   */
  void smoothPathWithFloorCostmaps(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<navbim_msgs::srv::SmoothPathWithFloorCostmaps::Request> request,
    std::shared_ptr<navbim_msgs::srv::SmoothPathWithFloorCostmaps::Response> response);
  
  /**
   * @brief Smooth path segments on specified floors
   * @param path Path to smooth
   * @param floor_segments Vector of (floor_name, start_idx, end_idx) for each floor segment
   * @return Smoothed path, or original if smoothing fails
   */
  nav_msgs::msg::Path smoothFloorSegments(
    const nav_msgs::msg::Path & path,
    const std::vector<std::tuple<std::string, size_t, size_t>> & floor_segments);
  /**
   * @brief Activate member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Deactivate member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Reset member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called when in shutdown state
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  using ActionToPoseInRoom = navbim_msgs::action::NavbimComputePathToPoseInRoom;
  using ActionToPoseInRoomResult = ActionToPoseInRoom::Result;
  using ActionServerToPoseInRoom = nav2_util::SimpleActionServer<ActionToPoseInRoom>;

  /**
   * @brief Check if an action server is valid / active
   * @param action_server Action server to test
   * @return SUCCESS or FAILURE
   */
  template<typename T>
  bool isServerInactive(std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server);

  /**
   * @brief Check if an action server has a cancellation request pending
   * @param action_server Action server to test
   * @return SUCCESS or FAILURE
   */
  template<typename T>
  bool isCancelRequested(std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server);

  /**
   * @brief Check if an action server has a preemption request and replaces the goal
   * with the new preemption goal.
   * @param action_server Action server to get updated goal if required
   * @param goal Goal to overwrite
   */
  template<typename T>
  void getPreemptedGoalIfRequested(
    std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
    typename std::shared_ptr<const typename T::Goal> goal);

  /**
   * @brief The action server callback for room-based path planning
   * ComputePathToPoseInRoom
   */
  void computePlanInRoom();

  /**
   * @brief Publish a path for visualization purposes
   * @param path Reference to Global Path
   */
  void publishPlan(const nav_msgs::msg::Path & path);

  /**
   * @brief Calculate the Euclidean distance along a path
   * @param path The path to measure
   * @return Total Euclidean distance in meters
   */
  double calculatePathDistance(const nav_msgs::msg::Path & path);

  /**
   * @brief Calculate the effective traversal distance considering costmap costs
   * Uses Bresenham line iteration to sample ALL cells between consecutive poses,
   * then computes an "effective distance" that increases with costmap costs.
   * The result is comparable to path distance and represents traversal difficulty.
   * @param path The path to evaluate
   * @param costmap The costmap to sample costs from
   * @return Effective distance in meters: actual_distance * (1 + avg_cost/252)
   *         - Free space (cost=0): returns actual path distance
   *         - Inscribed obstacles (cost=252): returns 2× path distance
   *         - Returns -1.0 if path is invalid
   */
  double calculatePathCost(
    const nav_msgs::msg::Path & path,
    nav2_costmap_2d::Costmap2D * costmap);

  void exceptionWarning(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const std::string & planner_id,
    const std::string & floor_name,
    const std::string & room_name,
    const std::exception & ex,
    std::string & msg);

  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  std::unique_ptr<ActionServerToPoseInRoom> action_server_room_;

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  std::mutex dynamic_params_lock_;

  // Planner
  PlannerMap planners_;
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> gp_loader_;
  std::vector<std::string> default_ids_;
  std::vector<std::string> default_types_;
  std::vector<std::string> planner_ids_;
  std::vector<std::string> planner_types_;
  double max_planner_duration_;
  rclcpp::Duration costmap_update_timeout_;
  std::string planner_ids_concat_;

  // TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_;

  // Global Costmap with Room Management (manages its own executor_thread_ internally)
  std::shared_ptr<navbim_multi_costmap_2d::MultiCostmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  std::unique_ptr<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>
  collision_checker_;

  // Node options for creating sub-nodes
  rclcpp::NodeOptions node_options_;

private:
  // Publishers for the path
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr plan_publisher_;

  // Service for floor smoothing
  rclcpp::Service<navbim_msgs::srv::SmoothPathWithFloorCostmaps>::SharedPtr floor_smoothing_service_;
  rclcpp::CallbackGroup::SharedPtr floor_smoothing_callback_group_;
  
  // Room-level path smoothing
  std::unique_ptr<nav2_core::Smoother> smoother_;
  std::string room_smoother_type_;
  bool enable_room_smoothing_;
  double room_smoother_max_time_;
};

}  // namespace navbim_gpp_bim

#endif  // NAVBIM_GPP_BIM__ROOM_PLANNER_SERVER_HPP_