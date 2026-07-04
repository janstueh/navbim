#ifndef NAVBIM_GPP_BIM__CLEARANCE_SERVER_HPP_
#define NAVBIM_GPP_BIM__CLEARANCE_SERVER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_util/lifecycle_node.hpp"

#include "navbim_multi_costmap_2d/multi_costmap_2d_ros.hpp"
#include "navbim_msgs/srv/calculate_path_clearance.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_util/node_thread.hpp"

namespace navbim_gpp_bim
{

/**
 * @class navbim_gpp_bim::ClearanceServer
 * @brief A service server that calculates path clearance metrics for validation.
 * Isolated from navigation to prevent crashes from affecting path planning.
 */
class ClearanceServer : public nav2_util::LifecycleNode
{
public:
  /**
   * @brief Constructor
   * @param options Node options
   */
  explicit ClearanceServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor
   */
  ~ClearanceServer();

protected:
  /**
   * @brief Configure member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

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

  /**
   * @brief Service callback for path clearance calculation
   * @param request_header Request header
   * @param request Service request with path and floor segments
   * @param response Service response with clearance metrics
   */
  void calculatePathClearance(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<navbim_msgs::srv::CalculatePathClearance::Request> request,
    std::shared_ptr<navbim_msgs::srv::CalculatePathClearance::Response> response);

  /**
   * @brief Calculate clearance for a path segment
   * @param path Path to analyze
   * @param start_idx Start index in path
   * @param end_idx End index in path
   * @param costmap Costmap for obstacle detection
   * @param min_clearance Output: minimum clearance in segment
   * @param avg_clearance Output: average clearance in segment
   * @return true if calculation successful
   */
  bool calculateSegmentClearance(
    const nav_msgs::msg::Path & path,
    size_t start_idx,
    size_t end_idx,
    const nav2_costmap_2d::Costmap2D * costmap,
    std::list<std::tuple<uint32_t, double>> & clearances);

  /**
   * @brief Calculate clearance at a single pose using spiral search
   * @param x World x-coordinate
   * @param y World y-coordinate
   * @param costmap Costmap for obstacle detection
   * @return Clearance in meters, or -1.0 if outside map
   */
  double calculateClearanceAtPose(
    double x,
    double y,
    const nav2_costmap_2d::Costmap2D * costmap);

private:
  // Costmap (manages its own executor thread internally via parent Costmap2DROS)
  std::shared_ptr<navbim_multi_costmap_2d::MultiCostmap2DROS> costmap_ros_;

  // Service for path clearance calculation
  rclcpp::Service<navbim_msgs::srv::CalculatePathClearance>::SharedPtr clearance_service_;
  rclcpp::CallbackGroup::SharedPtr clearance_callback_group_;

  // Node options for creating sub-nodes
  rclcpp::NodeOptions node_options_;
};

}  // namespace navbim_gpp_bim

#endif  // NAVBIM_GPP_BIM__CLEARANCE_SERVER_HPP_
