#include "navbim_room_tracker/room_tracker_node.hpp"

#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/point.hpp"

#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

namespace navbim_room_tracker
{

RoomTracker::RoomTracker(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("room_tracker", "", options),
  service_available_(false)
{
  // Declare parameters
  declare_parameter("robot_frame", "base_link");
  declare_parameter("global_frame", "ifc");
  declare_parameter("update_frequency", 2.0);
  declare_parameter("topomap_server_timeout", 5.0);
}

RoomTracker::~RoomTracker()
{
}

nav2_util::CallbackReturn RoomTracker::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring RoomTracker");

  // Get parameters
  get_parameter("robot_frame", robot_frame_);
  get_parameter("global_frame", global_frame_);
  get_parameter("update_frequency", update_frequency_);
  get_parameter("topomap_server_timeout", server_timeout_);

  // Initialize TF2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Create service client for room detection
  room_service_client_ = create_client<navbim_msgs::srv::GetRoomByCoordinates>(
    "topomap_server/get_room_by_coordinates");

  // Create publisher for current room information
  // Use TRANSIENT_LOCAL for reliability - costmap nodes can get the latest room info
  current_room_pub_ = create_publisher<navbim_msgs::msg::CurrentRoom>(
    "current_room", 
    rclcpp::QoS(10).transient_local()
  );

  // Initialize heartbeat timer
  last_publish_time_ = get_clock()->now();

  RCLCPP_INFO(
    get_logger(), 
    "RoomTracker configured: robot_frame=%s, global_frame=%s, frequency=%.1f Hz",
    robot_frame_.c_str(), global_frame_.c_str(), update_frequency_
  );

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn RoomTracker::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating RoomTracker");

  // Create bond connection to lifecycle manager
  createBond();

  // Activate publisher
  current_room_pub_->on_activate();

  // Create timer for periodic room checking
  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / update_frequency_),
    std::bind(&RoomTracker::updateCurrentRoom, this)
  );

  // Wait for topomap server to be available
  RCLCPP_INFO(get_logger(), "Waiting for topomap server...");
  if (room_service_client_->wait_for_service(std::chrono::seconds(static_cast<int>(server_timeout_)))) {
    service_available_ = true;
    RCLCPP_INFO(get_logger(), "Topomap server available");
  } else {
    RCLCPP_WARN(get_logger(), "Topomap server not available, will retry periodically");
    service_available_ = false;
  }

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn RoomTracker::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating RoomTracker");

  // Destroy bond connection
  destroyBond();

  // Cancel timer
  if (timer_) {
    timer_->cancel();
    timer_.reset();
  }

  // Deactivate publisher
  current_room_pub_->on_deactivate();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn RoomTracker::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up RoomTracker");

  // Reset all shared pointers
  timer_.reset();
  current_room_pub_.reset();
  room_service_client_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn RoomTracker::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down RoomTracker");
  return nav2_util::CallbackReturn::SUCCESS;
}

std::unique_ptr<geometry_msgs::msg::Point> RoomTracker::getRobotPose()
{
  try {
    // First check if we have the full tf chain: ifc -> map -> odom -> base_link
    // If any of these fail, the robot is not properly localized
    
    // Check if ifc -> map exists
    auto ifc_to_map = tf_buffer_->lookupTransform("ifc", "map", tf2::TimePointZero);
    
    // Check if map -> odom exists  
    auto map_to_odom = tf_buffer_->lookupTransform("map", "odom", tf2::TimePointZero);
    
    // Check if odom -> base_link exists
    auto odom_to_base = tf_buffer_->lookupTransform("odom", robot_frame_, tf2::TimePointZero);
    
    // If we get here, the full chain exists. Now get the full transform
    auto transform = tf_buffer_->lookupTransform(
      global_frame_,
      robot_frame_,
      tf2::TimePointZero
    );

    // Extract position
    auto position = std::make_unique<geometry_msgs::msg::Point>();
    position->x = transform.transform.translation.x;
    position->y = transform.transform.translation.y;
    position->z = transform.transform.translation.z;

    return position;

  } catch (const tf2::TransformException & ex) {
    RCLCPP_DEBUG(
      get_logger(), 
      "Could not get full tf chain or transform %s to %s: %s", 
      robot_frame_.c_str(), global_frame_.c_str(), ex.what()
    );
    return nullptr;
  }
}

void RoomTracker::updateCurrentRoom()
{
  // Get robot position
  auto robot_pos = getRobotPose();
  if (!robot_pos) {
    return;
  }

  // Check if topomap service is available
  if (!service_available_) {
    if (room_service_client_->wait_for_service(std::chrono::seconds(0))) {
      service_available_ = true;
      RCLCPP_INFO(get_logger(), "Topomap server became available");
    } else {
      return;
    }
  }

  // Call room detection service asynchronously to avoid deadlock in single-threaded executor
  auto request = std::make_shared<navbim_msgs::srv::GetRoomByCoordinates::Request>();
  request->coordinates = *robot_pos;

  auto position_snapshot = *robot_pos;
  room_service_client_->async_send_request(
    request,
    [this, position_snapshot](
      rclcpp::Client<navbim_msgs::srv::GetRoomByCoordinates>::SharedFuture future_response)
    {
      auto response = future_response.get();

      if (response->success) {
        auto room_msg = std::make_unique<navbim_msgs::msg::CurrentRoom>();
        room_msg->header.stamp = get_clock()->now();
        room_msg->header.frame_id = global_frame_;
        room_msg->floor_name = response->floor_node.name;
        room_msg->room_id = response->room_node.id;
        room_msg->room_name = response->room_node.name;
        room_msg->robot_position = position_snapshot;

        std::string current_room_id = response->floor_node.name + "/" + response->room_node.id;
        bool room_changed = (last_known_room_ != current_room_id);
        bool heartbeat_needed =
          (get_clock()->now() - last_publish_time_).seconds() > HEARTBEAT_INTERVAL;

        if (room_changed || heartbeat_needed || last_known_room_.empty()) {
          if (room_changed) {
            RCLCPP_INFO(
              get_logger(),
              "Robot moved to: %s (ID: %s) on floor %s",
              response->room_node.name.c_str(),
              response->room_node.id.c_str(),
              response->floor_node.name.c_str()
            );
          }
          last_known_room_ = current_room_id;
          last_publish_time_ = get_clock()->now();
          current_room_pub_->publish(std::move(room_msg));
        }
      } else {
        RCLCPP_INFO(get_logger(), "Robot not in any known room: %s", response->message.c_str());
        if (!last_known_room_.empty()) {
          RCLCPP_WARN(get_logger(), "Robot left known rooms");
          last_known_room_.clear();
        }
        auto room_msg = std::make_unique<navbim_msgs::msg::CurrentRoom>();
        room_msg->header.stamp = get_clock()->now();
        room_msg->header.frame_id = global_frame_;
        room_msg->floor_name = "";
        room_msg->room_id = "";
        room_msg->room_name = "";
        room_msg->robot_position = position_snapshot;
        current_room_pub_->publish(std::move(room_msg));
      }
    });
}

}  // namespace navbim_room_tracker

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(navbim_room_tracker::RoomTracker)