#include "navbim_multi_costmap_2d/multi_costmap_2d_ros.hpp"
#include "navbim_multi_costmap_2d/multimap_static_layer.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <set>
#include <shared_mutex>
#include <string>
#include <thread>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "navbim_msgs/srv/get_min_z.hpp"

namespace navbim_multi_costmap_2d
{

RoomCostmapInfo::RoomCostmapInfo(
  const std::string & env_name,
  const std::string & map_name_param,
  const std::string & topic_name_param)
: environment_name(env_name),
  map_name(map_name_param),
  topic_name(topic_name_param),
  full_map_id(env_name + "/" + map_name_param),
  is_active(false),
  has_been_published(false),
  has_fetched_z(false),
  has_changed_since_last_floor_update(false),
  map_origin_z(0.0)  // Default to 0, will be updated when map is received
{
}

MultiCostmap2DROS::MultiCostmap2DROS(
  const std::string & name,
  const std::string & parent_namespace,
  bool use_sim_time,
  const rclcpp::NodeOptions & options)
: nav2_costmap_2d::Costmap2DROS(name, parent_namespace, "", use_sim_time),
  update_frequency_(5.0)
{
  (void)options;
  RCLCPP_DEBUG(get_logger(), "Creating MultiCostmap2DROS");
}

MultiCostmap2DROS::~MultiCostmap2DROS()
{
}

nav2_util::CallbackReturn MultiCostmap2DROS::on_configure(const rclcpp_lifecycle::State & state)
{ 
  // Call parent configuration first (sets up TF, etc.)
  auto result = nav2_costmap_2d::Costmap2DROS::on_configure(state);
  if (result != nav2_util::CallbackReturn::SUCCESS) {
    return result;
  }
  
  // Get our parameters
  getMultiCostmapParameters();

  // Use parent's callback_group_
  // Subscribe to environments updates from navbim_multimap_server
  // Use transient_local durability so late-joining subscribers receive the last message
  // This is critical because environments is published once at startup
  rclcpp::QoS env_qos(10);
  env_qos.transient_local();
  rclcpp::SubscriptionOptions env_sub_options;
  env_sub_options.callback_group = callback_group_;
  env_sub_ = create_subscription<navbim_msgs::msg::Environments>(
    "/environments",
    env_qos,
    std::bind(&MultiCostmap2DROS::onEnvironmentUpdate, this, std::placeholders::_1),
    env_sub_options);
  RCLCPP_INFO(get_logger(), "Created /environments subscription with transient_local QoS");

  // Subscribe to current room updates from room tracker
  rclcpp::QoS room_qos(10);
  room_qos.transient_local();
  rclcpp::SubscriptionOptions room_sub_options;
  room_sub_options.callback_group = callback_group_;
  current_room_sub_ = create_subscription<navbim_msgs::msg::CurrentRoom>(
    "/current_room",
    room_qos,
    std::bind(&MultiCostmap2DROS::onCurrentRoomUpdate, this, std::placeholders::_1),
    room_sub_options);

  // Create service client for getting room heights from topomap server
  get_min_z_client_ = rclcpp::create_client<navbim_msgs::srv::GetMinZ>(
    get_node_base_interface(),
    get_node_graph_interface(),
    get_node_services_interface(),
    "/topomap_server/get_min_z",
    rclcpp::ServicesQoS(),
    callback_group_);

  RCLCPP_DEBUG(get_logger(), "MultiCostmap2DROS configured");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn MultiCostmap2DROS::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  // Reset shutdown flag
  is_shutting_down_.store(false);
  
  // Wait for room costmaps to be loaded (they load via env_sub_ callback after executor starts)
  // The subscription has transient_local QoS so it should receive the latched message
  RCLCPP_INFO(get_logger(), "Waiting for room costmaps to load via /environments subscription...");
  auto start_time = std::chrono::steady_clock::now();
  const auto timeout = std::chrono::seconds(10);
  
  while (std::chrono::steady_clock::now() - start_time < timeout) {
    {
      std::shared_lock<std::shared_mutex> lock(costmap_mutex_);  // Read-only check
      if (!room_costmaps_.empty()) {
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  
  std::unique_lock<std::shared_mutex> lock(costmap_mutex_);  // Write: activating rooms
  if (room_costmaps_.empty()) {
    RCLCPP_WARN(get_logger(), "No room costmaps loaded after %ld seconds - continuing with empty costmaps", 
                timeout.count());
  } else {
    RCLCPP_INFO(get_logger(), "Activating with %zu room costmaps", room_costmaps_.size());
  }

  // Wait for topomap service to be available
  if (get_min_z_client_) {
    if (!get_min_z_client_->wait_for_service(std::chrono::seconds(10))) {
      RCLCPP_WARN(get_logger(), "Topomap service not available - z-coordinates will be 0");
    }
  }

  // Activate publishers
  footprint_pub_->on_activate();
  costmap_publisher_->on_activate();

  for (auto & layer_pub : layer_publishers_) {
    layer_pub->on_activate();
  }

  // Activate all room costmap layers and publishers
  for (auto & [room_id, room_info] : room_costmaps_) {
    if (room_info->static_layer) {
      room_info->static_layer->activate();
    }
    if (room_info->inflation_layer) {
      room_info->inflation_layer->activate();
    }
    room_info->is_active = true;
  }
  
  for (auto & [room_id, pub] : costmap_pubs_) {
    pub->on_activate();
  }
  
  // Activate floor costmap publishers (if any exist)
  for (auto & [floor_name, pub] : floor_costmap_pubs_) {
    if (pub) {
      pub->on_activate();
      RCLCPP_DEBUG(get_logger(), "Activated floor costmap publisher for '%s'", floor_name.c_str());
    }
  }
  
  // Note: Floor costmaps will be created on-demand or by the update timer
  // to avoid blocking activation and the executor thread

  // Create update timers
  stopped_ = true;
  stop_updates_ = false;
  
  auto update_period = std::chrono::duration<double>(1.0 / update_frequency_);
  update_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(update_period),
    std::bind(&MultiCostmap2DROS::updateCurrentRoomMap, this),
    callback_group_);

  auto floor_update_period = std::chrono::duration<double>(1.0 / floor_update_frequency_);
  floor_update_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(floor_update_period),
    std::bind(&MultiCostmap2DROS::updateFloorCostmaps, this),
    callback_group_);

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn MultiCostmap2DROS::on_deactivate(const rclcpp_lifecycle::State & state)
{
  // Stop update timers FIRST to prevent new callbacks
  if (update_timer_) {
    update_timer_->cancel();
  }
  if (floor_update_timer_) {
    floor_update_timer_->cancel();
  }

  stop_updates_ = true;
  stopped_ = true;
  is_shutting_down_.store(true);

  // on_activate() replaces Costmap2DROS::on_activate() entirely, so
  // map_update_thread_ and dyn_params_handler are never set by the parent.
  // Calling Costmap2DROS::on_deactivate() would crash on the null
  // map_update_thread_->joinable() dereference.  Replicate what the parent
  // does, guarding against the un-initialized members.
  if (!map_update_thread_) {
    if (dyn_params_handler) {
      remove_on_set_parameters_callback(dyn_params_handler.get());
      dyn_params_handler.reset();
    }
    stop();
    footprint_pub_->on_deactivate();
    costmap_publisher_->on_deactivate();
    for (auto & layer_pub : layer_publishers_) {
      layer_pub->on_deactivate();
    }
  } else {
    // Fallback: parent's on_activate() was called — use the normal path.
    nav2_util::CallbackReturn result = nav2_util::CallbackReturn::SUCCESS;
    try {
      result = Costmap2DROS::on_deactivate(state);
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "Exception during parent deactivation: %s", e.what());
    }
    if (result != nav2_util::CallbackReturn::SUCCESS) {
      return result;
    }
  }

  std::unique_lock<std::shared_mutex> lock(costmap_mutex_);

  for (auto & [room_id, room_info] : room_costmaps_) {
    if (room_info->static_layer) {
      room_info->static_layer->deactivate();
    }
    if (room_info->inflation_layer) {
      room_info->inflation_layer->deactivate();
    }
    room_info->is_active = false;
  }

  for (auto & [room_id, pub] : costmap_pubs_) {
    pub->on_deactivate();
  }

  for (auto & [floor_name, pub] : floor_costmap_pubs_) {
    pub->on_deactivate();
  }

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn MultiCostmap2DROS::on_cleanup(const rclcpp_lifecycle::State & state)
{
  // Call parent's cleanup FIRST - this stops the executor thread
  // The executor_thread_.reset() calls cancel() then join()
  auto result = Costmap2DROS::on_cleanup(state);
  if (result != nav2_util::CallbackReturn::SUCCESS) {
    return result;
  }
  
  // Set shutdown flag to prevent any late async callbacks
  is_shutting_down_.store(true);
  
  // NOW that executor is stopped, explicitly reset OUR subscriptions
  // This ensures they're destroyed in the correct order while everything is stopped
  if (env_sub_) {
    env_sub_.reset();
  }
  if (current_room_sub_) {
    current_room_sub_.reset();
  }
  
  // Reset timers
  if (update_timer_) {
    update_timer_.reset();
  }
  if (floor_update_timer_) {
    floor_update_timer_.reset();
  }
  
  // Reset service client
  if (get_min_z_client_) {
    get_min_z_client_.reset();
  }
  
  std::unique_lock<std::shared_mutex> lock(costmap_mutex_);  // Write: cleaning up rooms
  
  // Clean up room costmaps
  for (auto & [room_id, room_info] : room_costmaps_) {
    if (room_info) {
      if (room_info->static_layer) {
        try {
          room_info->static_layer->deactivate();
        } catch (const std::exception & e) {
          RCLCPP_WARN(get_logger(), "Exception deactivating static layer for '%s': %s",
                      room_id.c_str(), e.what());
        }
      }
      if (room_info->inflation_layer) {
        try {
          room_info->inflation_layer->deactivate();
        } catch (const std::exception & e) {
          RCLCPP_WARN(get_logger(), "Exception deactivating inflation layer for '%s': %s",
                      room_id.c_str(), e.what());
        }
      }
    }
  }
  room_costmaps_.clear();
  costmap_pubs_.clear();
  
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_costmap_2d::Costmap2D * MultiCostmap2DROS::getCostmap()
{
  // For nav2 compatibility, return nullptr or implement default behavior
  // External code should use getCostmapForRoom() instead
  RCLCPP_WARN_ONCE(
    get_logger(),
    "MultiCostmap2DROS::getCostmap() called - use getCostmapForRoom() instead");
  return nullptr;
}

std::shared_ptr<nav2_costmap_2d::LayeredCostmap> MultiCostmap2DROS::getCostmapForRoom(const std::string & room_id)
{
  std::shared_lock<std::shared_mutex> lock(costmap_mutex_);  // Read-only access
  auto it = room_costmaps_.find(room_id);
  if (it != room_costmaps_.end() && it->second->layered_costmap) {
    return it->second->layered_costmap;  // Return shared_ptr to keep LayeredCostmap alive
  }
  return nullptr;
}

std::shared_ptr<nav2_costmap_2d::LayeredCostmap> MultiCostmap2DROS::getCostmapForRoom(
  const std::string & floor_name, 
  const std::string & room_name)
{
  std::string room_id = createRoomKey(floor_name, room_name);
  return getCostmapForRoom(room_id);
}

std::vector<std::string> MultiCostmap2DROS::getAvailableRooms() const
{
  std::shared_lock<std::shared_mutex> lock(costmap_mutex_);  // Read-only access
  std::vector<std::string> room_ids;
  room_ids.reserve(room_costmaps_.size());
  
  for (const auto & [room_id, room_info] : room_costmaps_) {
    room_ids.push_back(room_id);
  }
  
  return room_ids;
}

bool MultiCostmap2DROS::hasRoom(const std::string & room_id) const
{
  std::shared_lock<std::shared_mutex> lock(costmap_mutex_);  // Read-only access
  return room_costmaps_.find(room_id) != room_costmaps_.end();
}

void MultiCostmap2DROS::getMultiCostmapParameters()
{ 
  declare_parameter("room_update_frequency", 5.0);
  update_frequency_ = get_parameter("room_update_frequency").as_double();
  
  declare_parameter("floor_update_frequency", 1.0);
  floor_update_frequency_ = get_parameter("floor_update_frequency").as_double();
  
  declare_parameter("elevate_costmaps", 0.0);
  elevate_costmaps_ = get_parameter("elevate_costmaps").as_double();
}

void MultiCostmap2DROS::onEnvironmentUpdate(const navbim_msgs::msg::Environments::SharedPtr msg)
{
  auto callback_start = now();
  
  // CRITICAL: Don't hold mutex while creating costmaps - this can take seconds!
  // First, collect what needs to be done
  std::set<std::string> existing_rooms;
  std::set<std::string> new_rooms;
  std::vector<std::pair<std::string, std::string>> rooms_to_create; // floor, room pairs
  std::vector<std::string> rooms_to_remove;
  
  {
    std::shared_lock<std::shared_mutex> lock(costmap_mutex_);  // Read: checking existing rooms
    
    // Track existing room costmaps
    for (const auto & [room_id, room_info] : room_costmaps_) {
      existing_rooms.insert(room_id);
    }

    // Process each environment and its maps
    for (const auto & environment : msg->environments) {
      for (const auto & map_name : environment.map_name) {
        std::string full_room_id = environment.name + "/" + map_name;
        new_rooms.insert(full_room_id);

        // Mark for creation if it doesn't exist
        if (room_costmaps_.find(full_room_id) == room_costmaps_.end()) {
          rooms_to_create.emplace_back(environment.name, map_name);
        }
      }
    }

    // Identify rooms to remove
    for (const auto & existing_room : existing_rooms) {
      if (new_rooms.find(existing_room) == new_rooms.end()) {
        rooms_to_remove.push_back(existing_room);
      }
    }
    
  } // Release mutex here!
  
  // Only log if there are changes to report
  if (!rooms_to_create.empty() || !rooms_to_remove.empty()) {
    RCLCPP_INFO(get_logger(), "Environment update: %zu rooms to create, %zu to remove",
                rooms_to_create.size(), rooms_to_remove.size());
  } else {
    RCLCPP_DEBUG(get_logger(), "Environment update: no changes (already have %zu rooms)",
                 existing_rooms.size());
    return;  // No changes, exit early
  }
  
  // CRITICAL: Defer room creation to avoid executor corruption!
  // Creating MultimapStaticLayer (with subscriptions) from this executor callback
  // would modify the subscription list during iteration → segfault.
  // Solution: Use std::async to create rooms on a separate thread.
  if (!rooms_to_create.empty()) {
    RCLCPP_DEBUG(get_logger(), "Deferring creation of %zu room costmaps", rooms_to_create.size());
    
    deferred_creation_future_ = std::async(std::launch::async,
      [this, rooms = rooms_to_create]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        createRoomCostmapsDeferred(rooms);
      });
  }
  
  // Mark old rooms as inactive (never remove during runtime)
  // Removal would destroy subscriptions while executor is active → corruption
  if (!rooms_to_remove.empty()) {
    std::unique_lock<std::shared_mutex> lock(costmap_mutex_);
    for (const auto & room_id : rooms_to_remove) {
      auto it = room_costmaps_.find(room_id);
      if (it != room_costmaps_.end()) {
        RCLCPP_DEBUG(get_logger(), "Marking room costmap as inactive: %s", room_id.c_str());
        it->second->is_active = false;
      }
    }
  }
}

std::pair<std::unique_ptr<navbim_multi_costmap_2d::RoomCostmapInfo>, 
          rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr>
MultiCostmap2DROS::createRoomCostmapData(const std::string & env_name, const std::string & map_name)
{
  std::string full_room_id = env_name + "/" + map_name;
  std::string topic_name = generateMapTopic(env_name, map_name);

  auto room_info = std::make_unique<RoomCostmapInfo>(env_name, map_name, topic_name);

  initializeRoomLayeredCostmap(*room_info);

  // Create and configure layers using our custom MultimapStaticLayer (uses callback_group_)
  auto static_layer_shared = std::make_shared<navbim_multi_costmap_2d::MultimapStaticLayer>();
  room_info->static_layer = static_layer_shared;
  
  auto inflation_layer_shared = std::make_shared<nav2_costmap_2d::InflationLayer>();
  room_info->inflation_layer = inflation_layer_shared;
  
  std::string static_layer_name = std::string(get_name()) + "_" + env_name + "_" + map_name + "_static";
  std::string inflation_layer_name = std::string(get_name()) + "_" + env_name + "_" + map_name + "_inflation";
  
  // Declare inflation layer parameters BEFORE calling initialize()
  try {
    // Get parameters from global inflation_layer config
    double inflation_radius = 0.55;
    double cost_scaling_factor = 10.0;
    bool inflate_unknown = false;
    bool inflate_around_unknown = false;
    
    if (has_parameter("inflation_layer.inflation_radius")) {
      get_parameter("inflation_layer.inflation_radius", inflation_radius);
    }
    if (has_parameter("inflation_layer.cost_scaling_factor")) {
      get_parameter("inflation_layer.cost_scaling_factor", cost_scaling_factor);
    }
    if (has_parameter("inflation_layer.inflate_unknown")) {
      get_parameter("inflation_layer.inflate_unknown", inflate_unknown);
    }
    if (has_parameter("inflation_layer.inflate_around_unknown")) {
      get_parameter("inflation_layer.inflate_around_unknown", inflate_around_unknown);
    }
    
    std::string enabled_param = inflation_layer_name + ".enabled";
    std::string radius_param = inflation_layer_name + ".inflation_radius";
    std::string scaling_param = inflation_layer_name + ".cost_scaling_factor";
    std::string unknown_param = inflation_layer_name + ".inflate_unknown";
    std::string around_unknown_param = inflation_layer_name + ".inflate_around_unknown";
    
    if (!has_parameter(enabled_param)) {
      declare_parameter(enabled_param, true);
    }
    if (!has_parameter(radius_param)) {
      declare_parameter(radius_param, inflation_radius);
    }
    if (!has_parameter(scaling_param)) {
      declare_parameter(scaling_param, cost_scaling_factor);
    }
    if (!has_parameter(unknown_param)) {
      declare_parameter(unknown_param, inflate_unknown);
    }
    if (!has_parameter(around_unknown_param)) {
      declare_parameter(around_unknown_param, inflate_around_unknown);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to declare inflation parameters for %s: %s",
                 full_room_id.c_str(), e.what());
  }
  
  // Pre-declare the map_topic parameter that StaticLayer will need
  std::string static_param_name = static_layer_name + ".map_topic";
  try {
    if (!has_parameter(static_param_name)) {
      declare_parameter(static_param_name, topic_name);
    } else {
      set_parameter(rclcpp::Parameter(static_param_name, topic_name));
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), "Could not set parameter %s: %s",
                static_param_name.c_str(), e.what());
  }
  
  // Add both layers to the layered costmap as plugins
  {
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(
      *(room_info->layered_costmap->getCostmap()->getMutex()));
    
    room_info->layered_costmap->addPlugin(static_layer_shared);
    room_info->layered_costmap->addPlugin(inflation_layer_shared);
    
    // Initialize layers with parent's callback_group_
    room_info->static_layer->initialize(
      room_info->layered_costmap.get(),
      static_layer_name,
      getTfBuffer().get(),
      shared_from_this(),
      callback_group_);
    
    room_info->inflation_layer->initialize(
      room_info->layered_costmap.get(),
      inflation_layer_name,
      getTfBuffer().get(),
      shared_from_this(),
      callback_group_);
  }

  // Initialize layers
  try {
    room_info->static_layer->onInitialize();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize static layer for %s: %s",
                 full_room_id.c_str(), e.what());
    return {nullptr, nullptr};
  }
  
  try {
    room_info->inflation_layer->onInitialize();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize inflation layer for %s: %s",
                 full_room_id.c_str(), e.what());
    return {nullptr, nullptr};
  }
  
  // CRITICAL: Set footprint AFTER layers are initialized so they receive onFootprintChanged() callback
  // This must happen after onInitialize() so the InflationLayer can update its inscribed_radius_
  std::vector<geometry_msgs::msg::Point> footprint = getRobotFootprint();
  if (footprint.empty()) {
    RCLCPP_ERROR(get_logger(), "Robot footprint is EMPTY! Check robot_radius parameter.");
    return {nullptr, nullptr};
  }
  
  room_info->layered_costmap->setFootprint(footprint);
  
  // Force inflation layer to recalculate caches with the new inscribed radius
  room_info->inflation_layer->matchSize();
  
  RCLCPP_DEBUG(get_logger(), "Room '%s' initialized with inscribed_radius: %.3f m", 
              full_room_id.c_str(), room_info->layered_costmap->getInscribedRadius());

  // Create publisher
  std::string costmap_topic = "costmap/" + env_name + "/" + map_name;
  auto costmap_pub = create_publisher<nav_msgs::msg::OccupancyGrid>(
    costmap_topic, rclcpp::QoS(1).transient_local());
  
  return {std::move(room_info), costmap_pub};
}

void MultiCostmap2DROS::createRoomCostmap(const std::string & env_name, const std::string & map_name)
{
  // Use the data creation function then insert with mutex
  std::string full_room_id = env_name + "/" + map_name;
  auto [room_info, costmap_pub] = createRoomCostmapData(env_name, map_name);
  
  if (!room_info || !costmap_pub) {
    return; // Failed to create
  }
  
  // Insert with mutex lock
  {
    std::unique_lock<std::shared_mutex> lock(costmap_mutex_);  // Write: inserting room
    room_costmaps_[full_room_id] = std::move(room_info);
    costmap_pubs_[full_room_id] = costmap_pub;
  }
  
  // If the node is already active, activate this room's layers and publisher immediately
  if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    costmap_pub->on_activate();
    
    std::unique_lock<std::shared_mutex> lock(costmap_mutex_);  // Write: activating layers
    auto it = room_costmaps_.find(full_room_id);
    if (it != room_costmaps_.end()) {
      if (it->second->static_layer) {
        it->second->static_layer->activate();
      }
      if (it->second->inflation_layer) {
        it->second->inflation_layer->activate();
      }
    }
  }
}

void MultiCostmap2DROS::createRoomCostmapsDeferred(
  std::vector<std::pair<std::string, std::string>> rooms_to_create)
{
  // Runs on separate thread via std::async - safe to create subscriptions
  
  std::vector<std::tuple<std::string, std::unique_ptr<RoomCostmapInfo>, 
                         rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr>> 
    created_rooms;
  
  for (const auto & [env_name, map_name] : rooms_to_create) {
    std::string full_room_id = env_name + "/" + map_name;
    RCLCPP_DEBUG(get_logger(), "Creating costmap for room: %s", full_room_id.c_str());
    auto [room_info, costmap_pub] = createRoomCostmapData(env_name, map_name);
    if (room_info && costmap_pub) {
      created_rooms.emplace_back(full_room_id, std::move(room_info), costmap_pub);
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to create costmap for room: %s", full_room_id.c_str());
    }
  }
  
  // Insert all created rooms with a single mutex lock
  if (!created_rooms.empty()) {
    std::unique_lock<std::shared_mutex> lock(costmap_mutex_);
    bool is_active = (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    
    for (auto & [room_id, room_info, costmap_pub] : created_rooms) {
      room_costmaps_[room_id] = std::move(room_info);
      costmap_pubs_[room_id] = costmap_pub;
      
      // Activate if node is already active
      if (is_active) {
        costmap_pub->on_activate();
        auto it = room_costmaps_.find(room_id);
        if (it != room_costmaps_.end()) {
          if (it->second->static_layer) {
            it->second->static_layer->activate();
          }
          if (it->second->inflation_layer) {
            it->second->inflation_layer->activate();
          }
        }
      }
    }
    
    RCLCPP_INFO(get_logger(), "Created %zu room costmaps", created_rooms.size());
  }
}

void MultiCostmap2DROS::removeRoomCostmap(const std::string & room_id)
{
  // CRITICAL: Only call from on_cleanup() when executor is stopped!
  // Destroying MultimapStaticLayer (with subscriptions) during executor
  // operation corrupts the wait set → segfault in rcl_subscription_is_valid.
  
  auto it = room_costmaps_.find(room_id);
  if (it != room_costmaps_.end()) {
    // Deactivate and clear the layers safely
    if (it->second->static_layer) {
      try {
        it->second->static_layer->deactivate();
      } catch (const std::exception & e) {
        RCLCPP_WARN(get_logger(), "Exception deactivating static layer for '%s': %s",
                    room_id.c_str(), e.what());
      }
      it->second->static_layer.reset();
    }
    
    if (it->second->inflation_layer) {
      try {
        it->second->inflation_layer->deactivate();
      } catch (const std::exception & e) {
        RCLCPP_WARN(get_logger(), "Exception deactivating inflation layer for '%s': %s",
                    room_id.c_str(), e.what());
      }
      it->second->inflation_layer.reset();
    }
    
    // Clear the layered costmap
    if (it->second->layered_costmap) {
      it->second->layered_costmap.reset();
    }
    
    // Remove publisher
    costmap_pubs_.erase(room_id);
    
    // Finally, remove from map
    room_costmaps_.erase(it);
  }
}

void MultiCostmap2DROS::initializeRoomLayeredCostmap(RoomCostmapInfo & room_info)
{
  std::string global_frame = getGlobalFrameID();
  bool rolling_window = false;
  bool track_unknown = true;
  
  room_info.layered_costmap = std::make_shared<nav2_costmap_2d::LayeredCostmap>(
    global_frame, rolling_window, track_unknown);

  // Initialize with larger dummy map to accommodate typical building coordinates
  // This will be resized by StaticLayer when it receives the actual map
  // Use 200x200 cells at 0.05m resolution = 10m x 10m, centered at origin
  room_info.layered_costmap->resizeMap(200, 200, 0.05, -5.0, -5.0);
  
  RCLCPP_DEBUG(get_logger(), "Initialized layered costmap for room '%s' with temporary bounds: "
               "200x200 cells @ 0.05m res, origin (-5.0, -5.0)",
               room_info.full_map_id.c_str());
}

std::string MultiCostmap2DROS::generateMapTopic(
  const std::string & env_name, 
  const std::string & map_name)
{
  // Follow navbim_multimap_server convention: maps/{namespace}/{map_name}/map
  return std::string("maps/") + env_name + "/" + map_name + "/map";
}

void MultiCostmap2DROS::publishCostmap(
  const std::string & room_id, 
  const nav2_costmap_2d::Costmap2D & costmap)
{
  auto pub_it = costmap_pubs_.find(room_id);
  if (pub_it == costmap_pubs_.end()) {
    return;
  }

  // Check if publisher is valid and activated
  if (!pub_it->second || !pub_it->second->is_activated()) {
    return;
  }

  auto room_it = room_costmaps_.find(room_id);
  if (room_it == room_costmaps_.end()) {
    return;
  }

  // Convert Costmap2D to OccupancyGrid message
  auto grid_msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();
  
  grid_msg->header.frame_id = getGlobalFrameID();
  grid_msg->header.stamp = get_clock()->now();
  
  grid_msg->info.resolution = costmap.getResolution();
  grid_msg->info.width = costmap.getSizeInCellsX();
  grid_msg->info.height = costmap.getSizeInCellsY();
  grid_msg->info.origin.position.x = costmap.getOriginX();
  grid_msg->info.origin.position.y = costmap.getOriginY();
  grid_msg->info.origin.position.z = room_it->second->map_origin_z + elevate_costmaps_;  // Use stored z-coordinate + elevation offset
  grid_msg->info.origin.orientation.w = 1.0;

  // Convert costmap data to occupancy grid data
  grid_msg->data.resize(grid_msg->info.width * grid_msg->info.height);
  
  unsigned char * costmap_data = costmap.getCharMap();
  for (unsigned int i = 0; i < grid_msg->info.width * grid_msg->info.height; ++i) {
    unsigned char cost = costmap_data[i];
    
    // Convert costmap values to occupancy grid values
    if (cost == nav2_costmap_2d::NO_INFORMATION) {
      grid_msg->data[i] = -1;  // Unknown
    } else if (cost == nav2_costmap_2d::FREE_SPACE) {
      grid_msg->data[i] = 0;   // Free
    } else if (cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
      grid_msg->data[i] = 100; // Occupied
    } else {
      // Scale intermediate values
      grid_msg->data[i] = static_cast<int8_t>((cost * 100) / 254);
    }
  }

  try {
    pub_it->second->publish(std::move(grid_msg));
    
    // Mark room as changed for floor costmap aggregation
    room_it->second->has_changed_since_last_floor_update = true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to publish costmap for '%s': %s", room_id.c_str(), e.what());
  }
}

std::string MultiCostmap2DROS::createRoomKey(const std::string & floor_name, const std::string & room_name)
{
  return floor_name + "/" + room_name;
}

void MultiCostmap2DROS::onCurrentRoomUpdate(const navbim_msgs::msg::CurrentRoom::SharedPtr msg)
{
  std::unique_lock<std::shared_mutex> lock(costmap_mutex_);  // Write: updating current room
  
  // Create room_id from floor_name/room_name to match our room costmap keys
  std::string new_room_id = msg->floor_name + "/" + msg->room_name;
  
  if (new_room_id != current_room_id_) {
    RCLCPP_DEBUG(
      get_logger(),
      "Current room changed: '%s' -> '%s'",
      current_room_id_.c_str(), new_room_id.c_str());
    
    current_room_id_ = new_room_id;
  }
}

void MultiCostmap2DROS::updateCurrentRoomMap()
{
  // Early return if shutting down to prevent race conditions
  if (is_shutting_down_.load()) {
    return;
  }
  
  // OPTIMIZATION: Minimize mutex hold time to prevent blocking getCostmapForRoom() calls
  // Only hold mutex when accessing shared data, release during expensive operations
  
  // First pass: Handle initial publication of room costmaps
  // Collect rooms that need work, then process without holding the mutex
  struct RoomWorkData {
    std::string room_id;
    std::shared_ptr<nav2_costmap_2d::LayeredCostmap> layered_costmap;
    std::shared_ptr<nav2_costmap_2d::StaticLayer> static_layer;  // Keep layer alive (has subscription)
    std::shared_ptr<nav2_costmap_2d::InflationLayer> inflation_layer;
  };
  
  std::vector<std::string> rooms_needing_z_fetch;
  std::vector<RoomWorkData> rooms_to_publish;
  
  {
    std::shared_lock<std::shared_mutex> lock(costmap_mutex_);  // Read: collecting rooms to process
    
    if (is_shutting_down_.load()) {
      return;
    }
    
    for (auto & [room_id, room_info] : room_costmaps_) {
      if (!room_info->layered_costmap || !room_info->static_layer) {
        continue;
      }
      
      auto * costmap_ptr = room_info->layered_costmap->getCostmap();
      if (!costmap_ptr) {
        continue;
      }
      
      auto size_x = costmap_ptr->getSizeInCellsX();
      auto size_y = costmap_ptr->getSizeInCellsY();
      
      if (size_x == 0 || size_y == 0) {
        continue;
      }
      
      if (!room_info->static_layer->isCurrent()) {
        continue;
      }
      
      // Check if we need to fetch z-coordinate
      if (!room_info->has_fetched_z && get_min_z_client_) {
        rooms_needing_z_fetch.push_back(room_id);
      }
      
      // Publish if never published and z-coordinate has been fetched
      if (!room_info->has_been_published && room_info->has_fetched_z) {
        rooms_to_publish.push_back({
          room_id,
          room_info->layered_costmap,
          room_info->static_layer,
          room_info->inflation_layer
        });
      }
    }
  } // Release mutex here
  
  // Process z-coordinate fetches (async, doesn't block)
  for (const auto & room_id : rooms_needing_z_fetch) {
    if (get_min_z_client_->wait_for_service(std::chrono::milliseconds(0))) {
      // Prepare request with brief lock, then call async WITHOUT holding lock
      std::string map_name;
      {
        std::shared_lock<std::shared_mutex> lock(costmap_mutex_);  // Read: getting map name
        auto room_it = room_costmaps_.find(room_id);
        if (room_it == room_costmaps_.end()) continue;
        map_name = room_it->second->map_name;
      }
      
      auto request = std::make_shared<navbim_msgs::srv::GetMinZ::Request>();
      request->name = map_name;

      // CRITICAL: Call async_send_request WITHOUT holding costmap_mutex_
      // The callback will be invoked by our executor since client uses callback_group_
      get_min_z_client_->async_send_request(
        request,
        [this, room_id](rclcpp::Client<navbim_msgs::srv::GetMinZ>::SharedFuture future) {
          try {
            auto response = future.get();
            if (response && response->success) {
              if (is_shutting_down_.load()) {
                return;
              }
              
              std::unique_lock<std::shared_mutex> callback_lock(costmap_mutex_);  // Write: marking has_fetched_z
              
              if (is_shutting_down_.load()) {
                return;
              }
              
              auto it = room_costmaps_.find(room_id);
              if (it != room_costmaps_.end() && !it->second->has_fetched_z) {
                it->second->map_origin_z = response->min_z;
                it->second->has_fetched_z = true;
              }
            } else {
              RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 30000,
                "Failed to get z-coordinate for room '%s'. Will retry.",
                room_id.c_str());
            }
          } catch (const std::exception & e) {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 30000,
              "Service call exception for room '%s': %s",
              room_id.c_str(), e.what());
          }
        }
      );
    } else {
      RCLCPP_WARN_ONCE(
        get_logger(),
        "Topomap service not yet available, costmaps will have z=0 until service is ready");
      break; // Don't spam warnings for every room
    }
  }
  
  // Publish rooms that are ready (expensive operation, done without mutex)
  // Holding shared_ptrs to layers ensures subscriptions stay alive during processing
  for (auto & room_work : rooms_to_publish) {
    if (is_shutting_down_.load()) {
      return;
    }
    
    room_work.layered_costmap->updateMap(0.0, 0.0, 0.0);
    auto * costmap_ptr = room_work.layered_costmap->getCostmap();
    if (costmap_ptr) {
      publishCostmap(room_work.room_id, *costmap_ptr);
    }
    
    // Mark as published
    std::unique_lock<std::shared_mutex> lock(costmap_mutex_);  // Write: marking published
    auto it = room_costmaps_.find(room_work.room_id);
    if (it != room_costmaps_.end()) {
      it->second->has_been_published = true;
    }
  }

  // Second pass: Update and publish only the current room's costmap
  std::string current_room;
  std::shared_ptr<nav2_costmap_2d::LayeredCostmap> current_layered_costmap = nullptr;
  std::shared_ptr<nav2_costmap_2d::StaticLayer> current_static_layer = nullptr;
  std::shared_ptr<nav2_costmap_2d::InflationLayer> current_inflation_layer = nullptr;
  
  {
    std::shared_lock<std::shared_mutex> lock(costmap_mutex_);  // Read: getting current room
    
    if (is_shutting_down_.load()) {
      return;
    }
    
    if (!current_room_id_.empty()) {
      auto room_it = room_costmaps_.find(current_room_id_);
      if (room_it != room_costmaps_.end() && room_it->second->layered_costmap) {
        current_room = current_room_id_;
        current_layered_costmap = room_it->second->layered_costmap;
        current_static_layer = room_it->second->static_layer;  // Keep layer alive
        current_inflation_layer = room_it->second->inflation_layer;
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                             "No costmap found for current room '%s'", current_room_id_.c_str());
      }
    }
  } // Release mutex here
  
  // Update current room (expensive operation, done without mutex)
  if (current_layered_costmap) {
    geometry_msgs::msg::PoseStamped robot_pose;
    if (getRobotPose(robot_pose)) {
      const double & x = robot_pose.pose.position.x;
      const double & y = robot_pose.pose.position.y;
      const double yaw = tf2::getYaw(robot_pose.pose.orientation);
      
      current_layered_costmap->updateMap(x, y, yaw);
      
      auto * costmap_ptr = current_layered_costmap->getCostmap();
      if (costmap_ptr) {
        publishCostmap(current_room, *costmap_ptr);
      }
    }
  }
}

double MultiCostmap2DROS::getRoomHeightFromTopomap(const std::string & room_name)
{
  if (!get_min_z_client_) {
    RCLCPP_ERROR(get_logger(), "Room min_z service client not initialized");
    return 0.0;
  }

  // Create request
  auto request = std::make_shared<navbim_msgs::srv::GetMinZ::Request>();
  request->name = room_name;

  try {
    // Call service synchronously
    if (!get_min_z_client_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_WARN(get_logger(), "GetMinZ service not available for '%s'", room_name.c_str());
      return 0.0;
    }

    auto future = get_min_z_client_->async_send_request(request);
    
    // Wait for response
    if (rclcpp::spin_until_future_complete(
      get_node_base_interface(), future, std::chrono::seconds(2)) != 
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_WARN(get_logger(), "GetMinZ service call timed out for '%s'", room_name.c_str());
      return 0.0;
    }

    auto response = future.get();
    if (response && response->success) {
      return response->min_z;
    } else if (response) {
      RCLCPP_WARN(get_logger(), "Failed to get height for '%s': %s",
                  room_name.c_str(), response->message.c_str());
      return 0.0;
    } else {
      RCLCPP_WARN(get_logger(), "Null response for '%s'", room_name.c_str());
      return 0.0;
    }
  } catch (const std::runtime_error & e) {
    RCLCPP_WARN(get_logger(), "Exception calling topomap service for '%s': %s",
                room_name.c_str(), e.what());
    return 0.0;
  }
}

void MultiCostmap2DROS::updateFloorCostmaps()
{
  // Updates floor costmaps when room changes have been detected
  // Creates floor costmaps via createFloorCostmap if they don't exist yet

  // Early return if shutting down
  if (is_shutting_down_.load()) {
    return;
  }
  
  // Step 1: Identify which rooms have changed and which floors are affected
  std::unordered_map<std::string, std::vector<std::string>> affected_floors_to_rooms;
  
  {
    std::shared_lock<std::shared_mutex> lock(costmap_mutex_);  // Read: checking room changes
    
    if (is_shutting_down_.load()) {
      return;
    }
    
    for (auto & [room_id, room_info] : room_costmaps_) {
      if (room_info->has_changed_since_last_floor_update) {
        std::string floor_name = getFloorFromRoomId(room_id);
        affected_floors_to_rooms[floor_name].push_back(room_id);
        room_info->has_changed_since_last_floor_update = false;
      }
    }
  }
  
  // Step 2: Update affected floor costmaps (only if there are changes)
  // CRITICAL: Don't hold locks for long periods to avoid blocking room planner
  if (!affected_floors_to_rooms.empty()) {
    for (const auto & [floor_name, changed_rooms] : affected_floors_to_rooms) {
      bool floor_exists = false;
      
      // Quick check if floor exists (minimal lock time)
      {
        std::shared_lock<std::shared_mutex> floor_lock(floor_cache_mutex_);  // Read: checking floor existence
        auto floor_it = floor_costmap_cache_.find(floor_name);
        floor_exists = (floor_it != floor_costmap_cache_.end() && floor_it->second.is_valid);
      }
      
      if (floor_exists) {
        // Floor costmap exists - perform incremental updates
        RCLCPP_DEBUG(get_logger(), "Incrementally updating floor '%s' with %zu changed rooms",
                     floor_name.c_str(), changed_rooms.size());
        
        for (const auto & room_id : changed_rooms) {
          updateFloorCostmapRegion(floor_name, room_id);
        }
      } else {
        // Floor costmap doesn't exist yet - create it
        RCLCPP_DEBUG(get_logger(), "Creating floor costmap for '%s'", floor_name.c_str());
        
        if (!createFloorCostmap(floor_name)) {
          RCLCPP_DEBUG(get_logger(), "Floor costmap for '%s' not created - some rooms not ready yet. Will retry.", floor_name.c_str());
        } else {
          RCLCPP_INFO(get_logger(), "Created floor costmap for '%s' in update timer", floor_name.c_str());
          // Floor costmap is published in createFloorCostmap(), so it's already sent to late joiners
        }
      }
    }
  }
  
  // Step 3: Publish ONLY affected floor costmaps (those that were just updated)
  // With transient_local QoS, late joiners (like RViz) get cached messages automatically
  // So we only publish when there are actual changes to avoid serialization overhead
  if (!affected_floors_to_rooms.empty()) {
    std::shared_lock<std::shared_mutex> floor_lock(floor_cache_mutex_);  // Read: publishing floor costmaps
    
    for (const auto & [floor_name, changed_rooms] : affected_floors_to_rooms) {
      auto pub_it = floor_costmap_pubs_.find(floor_name);
      if (pub_it == floor_costmap_pubs_.end() || !pub_it->second || !pub_it->second->is_activated()) {
        continue;  // Publisher doesn't exist or isn't active
      }
      
      auto floor_it = floor_costmap_cache_.find(floor_name);
      if (floor_it == floor_costmap_cache_.end() || !floor_it->second.is_valid || !floor_it->second.costmap) {
        continue;  // Floor costmap not ready
      }
      
      const auto & costmap = *floor_it->second.costmap;
      
      auto grid_msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();
      grid_msg->header.frame_id = getGlobalFrameID();
      grid_msg->header.stamp = get_clock()->now();
      grid_msg->info.resolution = costmap.getResolution();
      grid_msg->info.width = costmap.getSizeInCellsX();
      grid_msg->info.height = costmap.getSizeInCellsY();
      grid_msg->info.origin.position.x = costmap.getOriginX();
      grid_msg->info.origin.position.y = costmap.getOriginY();
      
      // Get floor's min_z from any room on this floor (they share the same floor height)
      double floor_z = 0.0;
      {
        // Acquire costmap_mutex_ briefly to get floor z-coordinate
        std::shared_lock<std::shared_mutex> room_lock(costmap_mutex_);  // Read: getting floor z
        
        // Find first room on this floor
        std::string first_room_id;
        for (const auto & [room_id, room_info] : room_costmaps_) {
          if (getFloorFromRoomId(room_id) == floor_name) {
            first_room_id = room_id;
            break;
          }
        }
        
        if (!first_room_id.empty()) {
          auto room_it = room_costmaps_.find(first_room_id);
          if (room_it != room_costmaps_.end() && room_it->second) {
            floor_z = room_it->second->map_origin_z;
          }
        }
      }
      grid_msg->info.origin.position.z = floor_z + elevate_costmaps_;
      
      grid_msg->info.origin.orientation.w = 1.0;
      
      grid_msg->data.resize(grid_msg->info.width * grid_msg->info.height);
      unsigned char * costmap_data = costmap.getCharMap();
      
      for (unsigned int i = 0; i < grid_msg->info.width * grid_msg->info.height; ++i) {
        unsigned char cost = costmap_data[i];
        if (cost == nav2_costmap_2d::NO_INFORMATION) {
          grid_msg->data[i] = -1;
        } else if (cost == nav2_costmap_2d::FREE_SPACE) {
          grid_msg->data[i] = 0;
        } else if (cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
          grid_msg->data[i] = 100;
        } else {
          grid_msg->data[i] = static_cast<int8_t>((cost * 100) / 254);
        }
      }
      
      try {
        pub_it->second->publish(std::move(grid_msg));
        RCLCPP_DEBUG(get_logger(), "Published floor costmap '%s' (%u x %u cells)",
                     floor_name.c_str(), costmap.getSizeInCellsX(), costmap.getSizeInCellsY());
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "Failed to publish floor costmap '%s': %s",
                     floor_name.c_str(), e.what());
      }
    }
  }
}

// ============================================================================
// Floor-Level Aggregated Costmap Methods
// ============================================================================

std::shared_ptr<nav2_costmap_2d::Costmap2D> MultiCostmap2DROS::getCostmapForFloor(const std::string & floor_name)
{
  // IMPORTANT: Caller MUST hold floor_cache_mutex_ (shared) while using the returned pointer
  // to prevent race conditions with floor costmap updates.
  // NOTE: This function does NOT create floor costmaps on-demand.
  // Floor costmaps are created asynchronously by updateFloorCostmaps timer.
  
  // Caller should already hold the lock, so we just do a simple lookup
  auto it = floor_costmap_cache_.find(floor_name);
  if (it != floor_costmap_cache_.end() && it->second.is_valid && it->second.costmap) {
    return it->second.costmap;  // Return shared_ptr - keeps costmap alive
  }
  
  // Floor costmap doesn't exist or isn't valid yet
  // It will be created asynchronously by updateFloorCostmaps timer
  RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000,
    "Floor costmap for '%s' not ready yet. Will be created by update timer once all rooms are loaded.",
    floor_name.c_str());
  return nullptr;
}

std::vector<std::string> MultiCostmap2DROS::getAvailableFloors() const
{
  std::shared_lock<std::shared_mutex> lock(costmap_mutex_);  // Read: getting floors
  std::set<std::string> floors;
  
  for (const auto & [room_id, room_info] : room_costmaps_) {
    floors.insert(getFloorFromRoomId(room_id));
  }
  
  return std::vector<std::string>(floors.begin(), floors.end());
}

bool MultiCostmap2DROS::createFloorCostmap(const std::string & floor_name)
{
  // Fast path: Check if floor costmap already exists (avoid redundant work)
  {
    std::shared_lock<std::shared_mutex> lock(floor_cache_mutex_);
    if (floor_costmap_cache_.find(floor_name) != floor_costmap_cache_.end()) {
      return true;  // Already exists, nothing to do
    }
  }
  
  // Collect room bounds data with brief mutex locks, then create floor costmap without holding mutex
  
  struct RoomBounds {
    double origin_x;
    double origin_y;
    double width;
    double height;
    double resolution;
  };
  
  std::vector<RoomBounds> room_bounds_list;
  std::vector<std::string> room_ids_copy;
  int total_rooms_on_floor = 0;
  
  // Step 1: Get room IDs and collect bounds with brief lock
  {
    std::shared_lock<std::shared_mutex> lock(costmap_mutex_);  // Read: checking rooms on floor
    
    // Find all rooms on this floor (inline to avoid recursive lock)
    std::vector<std::string> room_ids;
    for (const auto & [room_id, room_info] : room_costmaps_) {
      if (getFloorFromRoomId(room_id) == floor_name) {
        room_ids.push_back(room_id);
      }
    }
    
    if (room_ids.empty()) {
      RCLCPP_WARN(get_logger(), "No rooms found on floor '%s'", floor_name.c_str());
      return false;
    }
    
    total_rooms_on_floor = room_ids.size();
    
    // CRITICAL: Check that ALL rooms on floor are ready before creating
    // If we create with partial rooms, the floor costmap will be incomplete forever
    for (const auto & room_id : room_ids) {
      auto room_it = room_costmaps_.find(room_id);
      if (room_it == room_costmaps_.end() || !room_it->second->layered_costmap) {
        RCLCPP_DEBUG(get_logger(), 
          "Floor '%s': Room '%s' not ready (no layered costmap) - cannot create complete floor costmap yet",
          floor_name.c_str(), room_id.c_str());
        return false;  // Must wait for ALL rooms
      }
      
      auto * room_costmap = room_it->second->layered_costmap->getCostmap();
      if (!room_costmap) {
        RCLCPP_DEBUG(get_logger(), 
          "Floor '%s': Room '%s' not ready (no costmap) - cannot create complete floor costmap yet",
          floor_name.c_str(), room_id.c_str());
        return false;  // Must wait for ALL rooms
      }
      
      // Check room has valid size
      if (room_costmap->getSizeInCellsX() == 0 || room_costmap->getSizeInCellsY() == 0) {
        RCLCPP_DEBUG(get_logger(), 
          "Floor '%s': Room '%s' not ready (zero size) - cannot create complete floor costmap yet",
          floor_name.c_str(), room_id.c_str());
        return false;  // Must wait for ALL rooms
      }
      
      // Check static layer is current (has received and processed map data)
      if (!room_it->second->static_layer || !room_it->second->static_layer->isCurrent()) {
        RCLCPP_DEBUG(get_logger(), 
          "Floor '%s': Room '%s' not ready (map not loaded yet) - cannot create complete floor costmap yet",
          floor_name.c_str(), room_id.c_str());
        return false;  // Must wait for ALL rooms
      }
      
      // Critical: Verify room has actual map data (not all NO_INFORMATION)
      // isCurrent() only means map was received, not that it contains valid data
      bool has_valid_data = false;
      unsigned int size_x = room_costmap->getSizeInCellsX();
      unsigned int size_y = room_costmap->getSizeInCellsY();
      // Sample strategically to quickly detect if map has data
      unsigned int sample_step = std::max(1u, std::min(size_x, size_y) / 10);
      for (unsigned int x = 0; x < size_x && !has_valid_data; x += sample_step) {
        for (unsigned int y = 0; y < size_y && !has_valid_data; y += sample_step) {
          if (room_costmap->getCost(x, y) != nav2_costmap_2d::NO_INFORMATION) {
            has_valid_data = true;
          }
        }
      }
      
      if (!has_valid_data) {
        RCLCPP_DEBUG(get_logger(), 
          "Floor '%s': Room '%s' not ready (all cells NO_INFORMATION) - cannot create complete floor costmap yet",
          floor_name.c_str(), room_id.c_str());
        return false;  // Must wait for ALL rooms
      }
    }
    
    // ALL rooms are ready - now collect their bounds for floor costmap creation
    RCLCPP_DEBUG(get_logger(), 
      "Floor '%s': All %d rooms ready - proceeding with complete floor costmap creation",
      floor_name.c_str(), total_rooms_on_floor);
    
    for (const auto & room_id : room_ids) {
      auto room_it = room_costmaps_.find(room_id);
      // Already validated above, so these should never fail
      if (room_it == room_costmaps_.end() || !room_it->second->layered_costmap) {
        continue;
      }
      
      auto * room_costmap = room_it->second->layered_costmap->getCostmap();
      if (!room_costmap) {
        continue;
      }
      
      // Collect bounds (all rooms already validated)
      RoomBounds bounds;
      bounds.origin_x = room_costmap->getOriginX();
      bounds.origin_y = room_costmap->getOriginY();
      bounds.width = room_costmap->getSizeInMetersX();
      bounds.height = room_costmap->getSizeInMetersY();
      bounds.resolution = room_costmap->getResolution();
      
      room_bounds_list.push_back(bounds);
      room_ids_copy.push_back(room_id);
    }
  } // Release costmap_mutex_
  
  // Sanity check - should match total_rooms_on_floor
  if (room_bounds_list.size() != static_cast<size_t>(total_rooms_on_floor)) {
    RCLCPP_ERROR(get_logger(), 
      "Floor '%s': Collected %zu rooms but expected %d - this should not happen!",
      floor_name.c_str(), room_bounds_list.size(), total_rooms_on_floor);
    return false;
  }
  
  RCLCPP_DEBUG(get_logger(), "Creating floor costmap for '%s' with %zu rooms", 
              floor_name.c_str(), room_bounds_list.size());
  
  // Step 2: Compute bounding box (without holding mutex)
  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();
  double resolution = room_bounds_list[0].resolution;
  
  for (const auto & bounds : room_bounds_list) {
    min_x = std::min(min_x, bounds.origin_x);
    min_y = std::min(min_y, bounds.origin_y);
    max_x = std::max(max_x, bounds.origin_x + bounds.width);
    max_y = std::max(max_y, bounds.origin_y + bounds.height);
  }
  
  // Snap to grid boundaries
  min_x = std::floor(min_x / resolution) * resolution;
  min_y = std::floor(min_y / resolution) * resolution;
  
  // Step 3: Create floor costmap structure (without holding mutex)
  unsigned int size_x = static_cast<unsigned int>(std::ceil((max_x - min_x) / resolution));
  unsigned int size_y = static_cast<unsigned int>(std::ceil((max_y - min_y) / resolution));
  
  RCLCPP_DEBUG(get_logger(), 
              "Floor '%s' costmap size: %u x %u cells (%.2f x %.2f m) at resolution %.3f m/cell",
              floor_name.c_str(), size_x, size_y, max_x - min_x, max_y - min_y, resolution);
  
  // Initialize floor costmap with FREE_SPACE
  // This ensures all cells start as free, then room obstacles override them
  auto floor_costmap = std::make_shared<nav2_costmap_2d::Costmap2D>(
    size_x, size_y, resolution, min_x, min_y, nav2_costmap_2d::FREE_SPACE);
  
  // Step 4: Copy all room costmap data with brief locks, then merge without holding mutex
  struct RoomCostmapData {
    std::string room_id;
    std::vector<unsigned char> data;
    unsigned int size_x;
    unsigned int size_y;
    double origin_x;
    double origin_y;
    double resolution;
  };
  
  std::vector<RoomCostmapData> room_data_list;
  
  // Copy ALL room data with SINGLE mutex acquisition to avoid livelock
  // CRITICAL: Abort if any room fails - prevents incomplete floor costmaps
  {
    std::shared_lock<std::shared_mutex> lock(costmap_mutex_);  // Read: copying room data
    
    for (const auto & room_id : room_ids_copy) {
      auto room_it = room_costmaps_.find(room_id);
      if (room_it == room_costmaps_.end() || !room_it->second->layered_costmap) {
        RCLCPP_ERROR(get_logger(), 
          "Floor '%s': Room '%s' disappeared between validation and copy - ABORTING",
          floor_name.c_str(), room_id.c_str());
        return false;
      }
      
      auto * room_costmap = room_it->second->layered_costmap->getCostmap();
      if (!room_costmap) {
        RCLCPP_ERROR(get_logger(), 
          "Floor '%s': Room '%s' lost costmap between validation and copy - ABORTING",
          floor_name.c_str(), room_id.c_str());
        return false;
      }
      
      // Verify size hasn't changed
      if (room_costmap->getSizeInCellsX() == 0 || room_costmap->getSizeInCellsY() == 0) {
        RCLCPP_ERROR(get_logger(), 
          "Floor '%s': Room '%s' size became zero - ABORTING",
          floor_name.c_str(), room_id.c_str());
        return false;
      }
      
      // Quick copy of room costmap metadata and data
      RoomCostmapData room_data;
      room_data.room_id = room_id;
      room_data.size_x = room_costmap->getSizeInCellsX();
      room_data.size_y = room_costmap->getSizeInCellsY();
      room_data.origin_x = room_costmap->getOriginX();
      room_data.origin_y = room_costmap->getOriginY();
      room_data.resolution = room_costmap->getResolution();
      
      size_t num_cells = static_cast<size_t>(room_data.size_x) * static_cast<size_t>(room_data.size_y);
      room_data.data.resize(num_cells);
      std::memcpy(room_data.data.data(), room_costmap->getCharMap(), num_cells * sizeof(unsigned char));
      
      room_data_list.push_back(std::move(room_data));
    }
  } // Release costmap_mutex_ after copying ALL rooms
  
  // CRITICAL VERIFICATION: Ensure we got data for ALL validated rooms
  if (room_data_list.size() != room_ids_copy.size()) {
    RCLCPP_ERROR(get_logger(), 
      "Floor '%s': INCOMPLETE DATA - only copied %zu/%zu rooms despite explicit checks!",
      floor_name.c_str(), room_data_list.size(), room_ids_copy.size());
    return false;
  }
  
  RCLCPP_DEBUG(get_logger(), "Floor '%s': Successfully copied all %zu room costmaps", 
              floor_name.c_str(), room_data_list.size());
  
  // Merge room data into floor costmap WITHOUT holding mutex (expensive operation)
  for (const auto & room_data : room_data_list) {
    for (unsigned int y = 0; y < room_data.size_y; ++y) {
      for (unsigned int x = 0; x < room_data.size_x; ++x) {
        // Convert room cell to world coordinates
        double wx = room_data.origin_x + (x + 0.5) * room_data.resolution;
        double wy = room_data.origin_y + (y + 0.5) * room_data.resolution;
        
        unsigned int fx, fy;
        if (floor_costmap->worldToMap(wx, wy, fx, fy)) {
          unsigned char room_cost = room_data.data[y * room_data.size_x + x];
          unsigned char floor_cost = floor_costmap->getCost(fx, fy);
          
          if (room_cost == nav2_costmap_2d::NO_INFORMATION) {
            // Keep floor value
          } else if (floor_cost == nav2_costmap_2d::NO_INFORMATION) {
            floor_costmap->setCost(fx, fy, room_cost);
          } else {
            floor_costmap->setCost(fx, fy, std::max(room_cost, floor_cost));
          }
        }
      }
    }
  }
  
  // Step 5: Insert floor costmap into cache (need floor_cache_mutex_)
  {
    std::unique_lock<std::shared_mutex> floor_lock(floor_cache_mutex_);  // Write: inserting floor cache
    auto & floor_cache = floor_costmap_cache_[floor_name];
    floor_cache.costmap = floor_costmap;  // Shared ownership - safe even if used concurrently
    floor_cache.is_valid = true;
    floor_cache.last_update_time = now();
    
    // Calculate and store region info for incremental updates
    // Each room's offset is calculated from its origin relative to floor origin
    for (const auto & room_data : room_data_list) {
      navbim_multi_costmap_2d::RoomRegion region;
      
      // Calculate room's position in floor costmap cell coordinates
      double relative_x = room_data.origin_x - min_x;
      double relative_y = room_data.origin_y - min_y;
      
      // CRITICAL: Ensure non-negative before casting to unsigned (prevent wraparound)
      if (relative_x < 0.0) {
        RCLCPP_WARN(get_logger(), 
          "Floor '%s': Room '%s' has negative relative_x=%.3f (origin_x=%.3f < min_x=%.3f) - clamping to 0",
          floor_name.c_str(), room_data.room_id.c_str(), relative_x, room_data.origin_x, min_x);
        relative_x = 0.0;
      }
      if (relative_y < 0.0) {
        RCLCPP_WARN(get_logger(), 
          "Floor '%s': Room '%s' has negative relative_y=%.3f (origin_y=%.3f < min_y=%.3f) - clamping to 0",
          floor_name.c_str(), room_data.room_id.c_str(), relative_y, room_data.origin_y, min_y);
        relative_y = 0.0;
      }
      
      region.x_offset = static_cast<unsigned int>(std::round(relative_x / resolution));
      region.y_offset = static_cast<unsigned int>(std::round(relative_y / resolution));
      region.width = room_data.size_x;
      region.height = room_data.size_y;
      
      // Validate region is within floor bounds
      if (region.x_offset + region.width > floor_cache.costmap->getSizeInCellsX() ||
          region.y_offset + region.height > floor_cache.costmap->getSizeInCellsY()) {
        RCLCPP_ERROR(get_logger(), 
          "Floor '%s': Room '%s' region out of bounds! offset=(%u,%u) size=%ux%u floor_size=%ux%u",
          floor_name.c_str(), room_data.room_id.c_str(),
          region.x_offset, region.y_offset, region.width, region.height,
          floor_cache.costmap->getSizeInCellsX(), floor_cache.costmap->getSizeInCellsY());
        continue;  // Skip this room to prevent corruption
      }
      
      floor_cache.room_regions[room_data.room_id] = region;
      
      RCLCPP_DEBUG(get_logger(), 
        "Floor '%s': Room '%s' region: offset=(%u,%u), size=%ux%u",
        floor_name.c_str(), room_data.room_id.c_str(),
        region.x_offset, region.y_offset, region.width, region.height);
    }
  }

  // Create publisher for this floor costmap (if it doesn't exist yet)
  if (floor_costmap_pubs_.find(floor_name) == floor_costmap_pubs_.end()) {
    std::string floor_topic = "costmap/" + floor_name + "/aggregated";
    auto floor_pub = create_publisher<nav_msgs::msg::OccupancyGrid>(
      floor_topic, rclcpp::QoS(1).transient_local());
    
    floor_costmap_pubs_[floor_name] = floor_pub;
    
    // If the node is already active, activate this publisher immediately
    if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      floor_pub->on_activate();
    }
    
    RCLCPP_INFO(get_logger(), "Created publisher for floor costmap '%s' on topic '%s'",
                floor_name.c_str(), floor_topic.c_str());
  }

  RCLCPP_INFO(get_logger(), "Successfully created floor costmap for '%s' with %zu rooms",
              floor_name.c_str(), room_data_list.size());
  return true;
}

void MultiCostmap2DROS::updateFloorCostmapRegion(
  const std::string & floor_name, 
  const std::string & room_id)
{
  // Step 1: Verify room is ready and copy its data
  std::vector<unsigned char> room_data;
  unsigned int room_width = 0, room_height = 0;
  
  {
    std::shared_lock<std::shared_mutex> room_lock(costmap_mutex_);  // Read: copying room data
    
    auto room_it = room_costmaps_.find(room_id);
    if (room_it == room_costmaps_.end() || !room_it->second->layered_costmap) {
      RCLCPP_DEBUG(get_logger(), "UPDATE: Room '%s' not found or no layered costmap", room_id.c_str());
      return;
    }
    
    auto * room_costmap = room_it->second->layered_costmap->getCostmap();
    if (!room_costmap) {
      RCLCPP_DEBUG(get_logger(), "UPDATE: Room '%s' has no costmap", room_id.c_str());
      return;
    }
    
    // Check if room has valid map data before updating floor
    if (room_costmap->getSizeInCellsX() == 0 || room_costmap->getSizeInCellsY() == 0) {
      RCLCPP_DEBUG(get_logger(), "UPDATE: Room '%s' has zero size - skipping update", room_id.c_str());
      return;
    }
    
    if (!room_it->second->static_layer || !room_it->second->static_layer->isCurrent()) {
      RCLCPP_DEBUG(get_logger(), "UPDATE: Room '%s' map not loaded yet - skipping update", room_id.c_str());
      return;
    }
    
    // Verify room has actual map data (not all NO_INFORMATION)
    bool has_valid_data = false;
    unsigned int sample_step = std::max(1u, std::min(room_costmap->getSizeInCellsX(), room_costmap->getSizeInCellsY()) / 10);
    for (unsigned int x = 0; x < room_costmap->getSizeInCellsX() && !has_valid_data; x += sample_step) {
      for (unsigned int y = 0; y < room_costmap->getSizeInCellsY() && !has_valid_data; y += sample_step) {
        if (room_costmap->getCost(x, y) != nav2_costmap_2d::NO_INFORMATION) {
          has_valid_data = true;
        }
      }
    }
    
    if (!has_valid_data) {
      RCLCPP_DEBUG(get_logger(), "UPDATE: Room '%s' has no valid map data (all NO_INFORMATION) - skipping update", room_id.c_str());
      return;
    }
    
    room_width = room_costmap->getSizeInCellsX();
    room_height = room_costmap->getSizeInCellsY();
    
    // Quick copy of room costmap data
    size_t num_cells = static_cast<size_t>(room_width) * static_cast<size_t>(room_height);
    room_data.resize(num_cells);
    std::memcpy(room_data.data(), room_costmap->getCharMap(), num_cells * sizeof(unsigned char));
    
    RCLCPP_DEBUG(get_logger(), "UPDATE: Room '%s' is ready - proceeding with floor update", room_id.c_str());
  }
  
  // Step 2: Update floor costmap with brief lock
  {
    std::unique_lock<std::shared_mutex> floor_lock(floor_cache_mutex_);  // Write: updating floor costmap
    
    auto floor_it = floor_costmap_cache_.find(floor_name);
    if (floor_it == floor_costmap_cache_.end() || !floor_it->second.is_valid) {
      return;
    }
    
    auto & floor_cache = floor_it->second;
    
    // Find room region
    auto region_it = floor_cache.room_regions.find(room_id);
    if (region_it == floor_cache.room_regions.end()) {
      RCLCPP_WARN(get_logger(), "Room '%s' not found in floor '%s' regions",
                  room_id.c_str(), floor_name.c_str());
      // Already holding floor_cache_mutex_, so just set invalid directly
      floor_cache.is_valid = false;
      return;
    }
    
    // Smart region update using copied data
    const auto & region = region_it->second;
    
    // CRITICAL: Validate region bounds to prevent memory corruption
    if (region.x_offset >= floor_cache.costmap->getSizeInCellsX() ||
        region.y_offset >= floor_cache.costmap->getSizeInCellsY()) {
      RCLCPP_ERROR(get_logger(), 
        "Room '%s' region offset out of bounds: offset=(%u,%u) floor_size=%ux%u",
        room_id.c_str(), region.x_offset, region.y_offset,
        floor_cache.costmap->getSizeInCellsX(), floor_cache.costmap->getSizeInCellsY());
      floor_cache.is_valid = false;
      return;
    }
    
    if (region.x_offset + region.width > floor_cache.costmap->getSizeInCellsX() ||
        region.y_offset + region.height > floor_cache.costmap->getSizeInCellsY()) {
      RCLCPP_ERROR(get_logger(), 
        "Room '%s' region extends beyond floor bounds: offset=(%u,%u) size=%ux%u floor_size=%ux%u",
        room_id.c_str(), region.x_offset, region.y_offset, region.width, region.height,
        floor_cache.costmap->getSizeInCellsX(), floor_cache.costmap->getSizeInCellsY());
      floor_cache.is_valid = false;
      return;
    }
    
    // CRITICAL: Lock Costmap2D's internal mutex to prevent data races
    // Smoother may be reading this costmap concurrently in another thread
    auto * costmap_mutex = floor_cache.costmap->getMutex();
    if (!costmap_mutex) {
      RCLCPP_ERROR(get_logger(), "Floor costmap mutex is null for floor '%s'", floor_name.c_str());
      floor_cache.is_valid = false;
      return;
    }
    
    {
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> costmap_lock(*costmap_mutex);
      
      for (unsigned int y = 0; y < region.height; ++y) {
        for (unsigned int x = 0; x < region.width; ++x) {
          unsigned int floor_x = region.x_offset + x;
          unsigned int floor_y = region.y_offset + y;
          
          // Bounds check
          if (floor_x >= floor_cache.costmap->getSizeInCellsX() ||
              floor_y >= floor_cache.costmap->getSizeInCellsY()) {
            continue;
          }
          
          if (x < room_width && y < room_height) {
            unsigned char room_cost = room_data[y * room_width + x];
            unsigned char floor_cost = floor_cache.costmap->getCost(floor_x, floor_y);
            
            // Smart merging: prefer known information over unknown
            if (room_cost == nav2_costmap_2d::NO_INFORMATION) {
              // Room has unknown - keep current floor value
            } else if (floor_cost == nav2_costmap_2d::NO_INFORMATION) {
              // Floor has unknown but room has known info - use room's value
              floor_cache.costmap->setCost(floor_x, floor_y, room_cost);
            } else {
              // Both are known - use maximum cost (most conservative)
              floor_cache.costmap->setCost(floor_x, floor_y, std::max(room_cost, floor_cost));
            }
          }
        }
      }
    }
    
    floor_cache.last_update_time = now();
    
    RCLCPP_DEBUG(get_logger(), "[TRACE] updateFloorCostmapRegion END: floor='%s' room='%s'", 
                 floor_name.c_str(), room_id.c_str());
  }
}

void MultiCostmap2DROS::invalidateFloorCostmap(const std::string & floor_name)
{
  std::unique_lock<std::shared_mutex> lock(floor_cache_mutex_);  // Write: invalidating floor
  
  auto it = floor_costmap_cache_.find(floor_name);
  if (it != floor_costmap_cache_.end()) {
    it->second.is_valid = false;
    RCLCPP_DEBUG(get_logger(), "Invalidated floor costmap '%s'", floor_name.c_str());
  }
}

std::vector<std::string> MultiCostmap2DROS::getRoomsOnFloor(const std::string & floor_name) const
{
  std::shared_lock<std::shared_mutex> lock(costmap_mutex_);  // Read: getting rooms on floor
  
  std::vector<std::string> rooms;
  
  for (const auto & [room_id, room_info] : room_costmaps_) {
    if (getFloorFromRoomId(room_id) == floor_name) {
      rooms.push_back(room_id);
    }
  }
  
  return rooms;
}

std::string MultiCostmap2DROS::getFloorFromRoomId(const std::string & room_id) const
{
  // Room ID format: "floor_name/room_name"
  size_t slash_pos = room_id.find('/');
  if (slash_pos != std::string::npos) {
    return room_id.substr(0, slash_pos);
  }
  return room_id;  // Fallback if no slash found
}

bool MultiCostmap2DROS::areAllRoomsReadyOnFloor(const std::string & floor_name) const
{
  std::shared_lock<std::shared_mutex> lock(costmap_mutex_);  // Read: checking rooms ready
  
  // Get all rooms on this floor
  std::vector<std::string> floor_rooms;
  for (const auto & [room_id, room_info] : room_costmaps_) {
    if (getFloorFromRoomId(room_id) == floor_name) {
      floor_rooms.push_back(room_id);
    }
  }
  
  if (floor_rooms.empty()) {
    return false;  // No rooms on this floor
  }
  
  // Check if ALL rooms are ready
  for (const auto & room_id : floor_rooms) {
    if (!isRoomCostmapReady(room_id)) {
      return false;
    }
  }
  
  return true;  // All rooms on floor are ready
}

bool MultiCostmap2DROS::isRoomCostmapReady(const std::string & room_id) const
{
  // Assumes costmap_mutex_ is already locked by caller
  auto it = room_costmaps_.find(room_id);
  if (it == room_costmaps_.end()) {
    return false;
  }
  
  const auto & room_info = it->second;
  
  // Check if room has valid costmap data
  if (!room_info->layered_costmap || !room_info->static_layer) {
    return false;
  }
  
  auto * costmap_ptr = room_info->layered_costmap->getCostmap();
  if (!costmap_ptr) {
    return false;
  }
  
  // Check if map has been received and is current
  if (costmap_ptr->getSizeInCellsX() == 0 || 
      costmap_ptr->getSizeInCellsY() == 0 ||
      !room_info->static_layer->isCurrent()) {
    return false;
  }
  
  return true;
}



}  // namespace navbim_multi_costmap_2d
