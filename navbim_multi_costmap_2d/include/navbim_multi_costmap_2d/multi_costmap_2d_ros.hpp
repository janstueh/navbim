#ifndef NAVBIM_MULTI_COSTMAP_2D__MULTI_COSTMAP_2D_ROS_HPP_
#define NAVBIM_MULTI_COSTMAP_2D__MULTI_COSTMAP_2D_ROS_HPP_

#include <atomic>
#include <future>
#include <memory>
#include <mutex>
#include <set>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/service_client.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/static_layer.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"
#include "navbim_msgs/msg/environments.hpp"
#include "navbim_msgs/msg/environment.hpp"
#include "navbim_msgs/msg/current_room.hpp"
#include "navbim_msgs/srv/get_min_z.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace navbim_multi_costmap_2d
{

/**
 * @struct RoomRegion
 * @brief Describes where a room is located within a floor costmap
 */
struct RoomRegion
{
  unsigned int x_offset;     // X offset in floor costmap (cells)
  unsigned int y_offset;     // Y offset in floor costmap (cells)
  unsigned int width;        // Width in cells
  unsigned int height;       // Height in cells
  std::string room_id;       // Full room identifier
};

/**
 * @struct FloorCostmapCache
 * @brief Cached aggregated costmap for an entire floor with room region tracking
 */
struct FloorCostmapCache
{
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap;  // Aggregated costmap (shared to prevent use-after-free)
  std::unordered_map<std::string, RoomRegion> room_regions;  // Map room_id -> region
  bool is_valid;  // False if needs regeneration
  rclcpp::Time last_update_time;  // Last aggregation time
  
  FloorCostmapCache() : is_valid(false) {}
};

/**
 * @struct RoomCostmapInfo
 * @brief Information about an individual room costmap
 */
struct RoomCostmapInfo
{
  std::string environment_name;
  std::string map_name;
  std::string topic_name;
  std::string full_map_id;  // environment_name/map_name
  
  // Each room gets its own complete LayeredCostmap with StaticLayer + InflationLayer
  std::shared_ptr<nav2_costmap_2d::LayeredCostmap> layered_costmap;
  std::shared_ptr<nav2_costmap_2d::StaticLayer> static_layer;
  std::shared_ptr<nav2_costmap_2d::InflationLayer> inflation_layer;
  
  bool is_active;
  bool has_been_published;  // Track if this room's costmap has been published at least once
  bool has_fetched_z;       // Track if we've successfully fetched z-coordinate from topomap
  std::atomic<bool> has_changed_since_last_floor_update;  // written from publishCostmap without lock
  double map_origin_z;      // Z-coordinate from the original map (for visualization)
  
  RoomCostmapInfo(
    const std::string & env_name,
    const std::string & map_name_param,
    const std::string & topic_name_param);
};

/**
 * @class MultiCostmap2DROS
 * @brief Multi-room aware costmap that manages individual room costmaps
 * 
 * This class extends nav2_costmap_2d::Costmap2DROS to handle multiple room-specific
 * costmaps. Instead of managing a single global costmap, it maintains separate
 * costmaps for each room/environment and provides access to them individually.
 */
class MultiCostmap2DROS : public nav2_costmap_2d::Costmap2DROS
{
public:
  /**
   * @brief Constructor for sub-node creation
   * @param name Name of the costmap node
   * @param parent_namespace Namespace of the parent node
   * @param use_sim_time Whether to use simulation time
   * @param options Node options
   */
  MultiCostmap2DROS(
    const std::string & name,
    const std::string & parent_namespace,
    bool use_sim_time,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor
   */
  virtual ~MultiCostmap2DROS();

  // Multi-room specific interface
  /**
   * @brief Get layered costmap for a specific room
   * @param room_id Full room identifier (environment_name/map_name)
   * @return Shared pointer to LayeredCostmap, nullptr if not found
   * 
   * CRITICAL: Returns shared_ptr to prevent use-after-free if costmap is
   * replaced while caller is using it. Call ->getCostmap() to get raw pointer.
   */
  std::shared_ptr<nav2_costmap_2d::LayeredCostmap> getCostmapForRoom(const std::string & room_id);

  /**
   * @brief Get all available room identifiers
   * @return Vector of all room identifiers
   */
  std::vector<std::string> getAvailableRooms() const;

  /**
   * @brief Get layered costmap for room using floor and room names
   * @param floor_name Floor name
   * @param room_name Room name
   * @return Shared pointer to LayeredCostmap, nullptr if not found
   * 
   * CRITICAL: Returns shared_ptr to prevent use-after-free if costmap is
   * replaced while caller is using it. Call ->getCostmap() to get raw pointer.
   */
  std::shared_ptr<nav2_costmap_2d::LayeredCostmap> getCostmapForRoom(
    const std::string & floor_name, 
    const std::string & room_name);

  /**
   * @brief Check if a room costmap exists
   * @param room_id Full room identifier (environment_name/map_name)
   * @return True if room costmap exists
   */
  bool hasRoom(const std::string & room_id) const;

  /**
   * @brief Get aggregated costmap for an entire floor
   * @param floor_name Floor identifier
   * @return Shared pointer to aggregated floor costmap, nullptr if not available
   * 
   * This method returns a cached aggregated costmap that combines all room
   * costmaps on the specified floor. The floor costmap is created on first
   * request (lazy initialization) and cached for subsequent requests.
   * Updates to individual room costmaps trigger smart region-based updates.
   * 
   * CRITICAL: Returns shared_ptr to prevent use-after-free if costmap is
   * replaced while caller is using it.
   */
  std::shared_ptr<nav2_costmap_2d::Costmap2D> getCostmapForFloor(const std::string & floor_name);

  /**
   * @brief Get all available floor identifiers
   * @return Vector of floor names that have room costmaps
   */
  std::vector<std::string> getAvailableFloors() const;

  /**
   * @brief Check if all room costmaps on a floor are ready
   * @param floor_name Floor identifier
   * @return True if all rooms on the floor have valid, current costmaps
   */
  bool areAllRoomsReadyOnFloor(const std::string & floor_name) const;

  // Nav2 compatibility (delegates to default behavior or returns nullptr)
  /**
   * @brief Get the default costmap (for compatibility)
   * @return Pointer to a default costmap or nullptr
   */
  nav2_costmap_2d::Costmap2D * getCostmap();

  /**
   * @brief Get mutex for thread-safe costmap access
   * @return Reference to the costmap mutex
   * 
   * Use this to hold a lock while accessing costmap pointers:
   *   std::shared_lock<std::shared_mutex> lock(costmap_ros->getCostmapMutex());
   *   auto costmap = costmap_ros->getCostmapForFloor("floor1");
   *   // ... use costmap safely while lock is held ...
   */
  std::shared_mutex & getCostmapMutex() { return costmap_mutex_; }

  /**
   * @brief Get mutex for thread-safe floor costmap cache access
   * @return Reference to the floor cache mutex
   * 
   * Use this to hold a lock while accessing floor costmaps:
   *   std::shared_lock<std::shared_mutex> lock(costmap_ros->getFloorCacheMutex());
   *   auto costmap = costmap_ros->getCostmapForFloor("floor1");
   *   // ... use floor costmap safely while lock is held ...
   */
  std::shared_mutex & getFloorCacheMutex() { return floor_cache_mutex_; }

protected:
  // Lifecycle management
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

private:
  // Room management (integrated from MultiStaticLayer)
  std::unordered_map<std::string, std::unique_ptr<RoomCostmapInfo>> room_costmaps_;
  
  // Floor-level aggregated costmap caching
  std::unordered_map<std::string, FloorCostmapCache> floor_costmap_cache_;
  mutable std::shared_mutex floor_cache_mutex_;  // Separate mutex for floor costmap operations
  std::set<std::string> floors_pre_created_;  // Track which floors have been eagerly created
  
  // Environment subscription
  rclcpp::Subscription<navbim_msgs::msg::Environments>::SharedPtr env_sub_;
  
  // Current room tracking
  std::string current_room_id_;
  rclcpp::Subscription<navbim_msgs::msg::CurrentRoom>::SharedPtr current_room_sub_;
  
  // Service client for getting min_z values from topomap server
  rclcpp::Client<navbim_msgs::srv::GetMinZ>::SharedPtr get_min_z_client_;
  
  // Publishers for each costmap (using lifecycle publishers)
  std::unordered_map<std::string, rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr> costmap_pubs_;
  std::unordered_map<std::string, rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr> floor_costmap_pubs_;
  
  // Thread safety
  mutable std::shared_mutex costmap_mutex_;
  std::atomic<bool> is_shutting_down_{false};  // Flag to prevent async callbacks during shutdown
  std::future<void> deferred_creation_future_;  // Keeps async thread alive past callback scope
  
  // Custom update timer for selective room updates
  rclcpp::TimerBase::SharedPtr update_timer_;
  rclcpp::TimerBase::SharedPtr floor_update_timer_;
  
  // Parameters - use primitives only to avoid ABI issues
  double update_frequency_;
  double floor_update_frequency_;
  double elevate_costmaps_;  // Elevation offset for costmap visualization (meters)
  
  // Room management methods (moved from MultiStaticLayer)
  /**
   * @brief Callback for environment updates from navbim_multimap_server
   * @param msg Environments message containing all available environments and maps
   */
  void onEnvironmentUpdate(const navbim_msgs::msg::Environments::SharedPtr msg);

  /**
   * @brief Callback for current room updates from room tracker
   * @param msg String message containing current room ID
   */
  void onCurrentRoomUpdate(const navbim_msgs::msg::CurrentRoom::SharedPtr msg);

  /**
   * @brief Update the current room's costmap (called by timer)
   */
  void updateCurrentRoomMap();

  /**
   * @brief Update floor-level aggregated costmaps (called by timer)
   * 
   * Checks which rooms have changed since last update and performs
   * incremental updates to affected floor costmaps.
   */
  void updateFloorCostmaps();

  /**
   * @brief Create a room costmap for a specific environment and map
   * @param env_name Environment name
   * @param map_name Map name
   */
  void createRoomCostmap(const std::string & env_name, const std::string & map_name);
  
  /**
   * @brief Create room costmap data without inserting into map (for batch operations)
   * @param env_name Environment name
   * @param map_name Map name
   * @return Pair of room_info and publisher, both nullptr on failure
   */
  std::pair<std::unique_ptr<RoomCostmapInfo>, 
            rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr>
  createRoomCostmapData(const std::string & env_name, const std::string & map_name);
  
  /**
   * @brief Create room costmaps deferred (called from timer, not from callback)
   * @param rooms_to_create Vector of (env_name, map_name) pairs to create
   * 
   * CRITICAL: Must be called from a timer/deferred context, NOT from an executor callback!
   * Creating subscriptions (in MultimapStaticLayer) from an executor callback corrupts
   * the executor's wait set.
   */
  void createRoomCostmapsDeferred(std::vector<std::pair<std::string, std::string>> rooms_to_create);
  
  /**
   * @brief Remove a room costmap
   * @param room_id Room identifier
   */
  void removeRoomCostmap(const std::string & room_id);

  /**
   * @brief Initialize a LayeredCostmap for a specific room
   * @param room_info Room information to initialize
   */
  void initializeRoomLayeredCostmap(RoomCostmapInfo & room_info);

  /**
   * @brief Generate topic name for a specific map
   * @param env_name Environment name
   * @param map_name Map name
   * @return Full topic name following navbim_multimap_server convention
   */
  std::string generateMapTopic(const std::string & env_name, const std::string & map_name);

  /**
   * @brief Get room z-coordinate from topomap server
   * @param room_name Room name
   * @return Z-coordinate height, or 0.0 if service call fails
   */
  double getRoomHeightFromTopomap(const std::string & room_name);

  /**
   * @brief Publish costmap as OccupancyGrid message
   * @param room_id Full room identifier
   * @param costmap Costmap to publish
   */
  void publishCostmap(const std::string & room_id, const nav2_costmap_2d::Costmap2D & costmap);

  /**
   * @brief Create room key for internal mapping
   * @param floor_name Floor name
   * @param room_name Room name
   * @return String key for room identification
   */
  std::string createRoomKey(const std::string & floor_name, const std::string & room_name);

  /**
   * @brief Get parameters for multi-room costmap
   */
  void getMultiCostmapParameters();

  // Floor costmap management methods
  /**
   * @brief Create aggregated costmap for a floor (combines all room costmaps)
   * @param floor_name Floor identifier
   * @return True if floor costmap was successfully created
   */
  bool createFloorCostmap(const std::string & floor_name);

  /**
   * @brief Update a specific room region in the floor costmap
   * @param floor_name Floor identifier
   * @param room_id Room identifier
   */
  void updateFloorCostmapRegion(const std::string & floor_name, const std::string & room_id);

  /**
   * @brief Invalidate floor costmap cache (mark for regeneration)
   * @param floor_name Floor identifier
   */
  void invalidateFloorCostmap(const std::string & floor_name);

  /**
   * @brief Get all rooms on a specific floor
   * @param floor_name Floor identifier
   * @return Vector of room IDs on the floor
   */
  std::vector<std::string> getRoomsOnFloor(const std::string & floor_name) const;

  /**
   * @brief Extract floor name from room ID (environment/map format)
   * @param room_id Full room identifier
   * @return Floor name (environment part)
   */
  std::string getFloorFromRoomId(const std::string & room_id) const;

  /**
   * @brief Check if a specific room costmap is ready (has valid map data)
   * @param room_id Room identifier
   * @return True if room costmap has loaded map and is current
   */
  bool isRoomCostmapReady(const std::string & room_id) const;
};

}  // namespace navbim_multi_costmap_2d

#endif  // NAVBIM_MULTI_COSTMAP_2D__MULTI_COSTMAP_2D_ROS_HPP_