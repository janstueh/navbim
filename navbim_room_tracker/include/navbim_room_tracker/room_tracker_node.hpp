#ifndef NAVBIM_ROOM_TRACKER__ROOM_TRACKER_NODE_HPP_
#define NAVBIM_ROOM_TRACKER__ROOM_TRACKER_NODE_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/service_client.hpp"
#include "navbim_msgs/msg/current_room.hpp"
#include "navbim_msgs/srv/get_room_by_coordinates.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace navbim_room_tracker
{

/**
 * @class RoomTracker
 * @brief Lifecycle node that tracks the robot's current room by monitoring
 * tf transforms and querying the topological map server.
 */
class RoomTracker : public nav2_util::LifecycleNode
{
public:
  /**
   * @brief Constructor
   * @param options Additional options to control creation of the node.
   */
  explicit RoomTracker(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor
   */
  ~RoomTracker();

protected:
  /**
   * @brief Configure the node
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Activate the node
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Deactivate the node
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Cleanup the node
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Shutdown the node
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Get the current robot pose in the global frame
   * @return Robot position or nullptr if transform not available
   */
  std::unique_ptr<geometry_msgs::msg::Point> getRobotPose();

  /**
   * @brief Timer callback to update the current room information
   */
  void updateCurrentRoom();

private:
  // Parameters
  std::string robot_frame_;
  std::string global_frame_;
  double update_frequency_;
  double server_timeout_;

  // TF2 
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Service client for room detection
  rclcpp::Client<navbim_msgs::srv::GetRoomByCoordinates>::SharedPtr room_service_client_;

  // Publisher for current room information
  rclcpp_lifecycle::LifecyclePublisher<navbim_msgs::msg::CurrentRoom>::SharedPtr current_room_pub_;

  // Timer for periodic room checking
  rclcpp::TimerBase::SharedPtr timer_;

  // State variables
  std::string last_known_room_;
  bool service_available_;
  
  // Heartbeat for periodic status updates
  rclcpp::Time last_publish_time_;
  static constexpr double HEARTBEAT_INTERVAL = 30.0;  // seconds
};

}  // namespace navbim_room_tracker

#endif  // NAVBIM_ROOM_TRACKER__ROOM_TRACKER_NODE_HPP_