#include "navbim_multimap_server/multimap_server.hpp"

#include <fstream>
#include <memory>
#include <string>
#include <yaml-cpp/yaml.h>

#include "nav2_util/geometry_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace navbim_multimap_server
{

MultimapMapEntry::MultimapMapEntry(
  const std::string & map_url,
  const std::string & ns,
  const std::string & map_name,
  const std::string & global_frame,
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node)
: namespace_(ns), map_name_(map_name), node_(node)
{
  map_full_name_ = ns + "/" + map_name;

  // Load the map using the existing map_io functionality
  try {
    LoadParameters load_params = loadMapYaml(map_url);
    loadMapFromFile(load_params, map_);
  } catch (const std::exception & e) {
    throw std::runtime_error("Failed to load map from: " + map_url + ". Error: " + e.what());
  }

  // Set the header information
  map_.header.frame_id = global_frame;
  map_.header.stamp = node_->get_clock()->now();
  meta_data_ = map_.info;

  // Create topics and services for this map
  std::string service_name = "maps/" + ns + "/" + map_name + "/static_map";
  map_service_ = node_->create_service<nav_msgs::srv::GetMap>(
    service_name,
    std::bind(&MultimapMapEntry::mapCallback, this, std::placeholders::_1, std::placeholders::_2));

  // Latched publisher for metadata
  std::string metadata_topic = "maps/" + ns + "/" + map_name + "/map_metadata";
  auto metadata_qos = rclcpp::QoS(1).transient_local();
  metadata_pub_ = node_->create_publisher<nav_msgs::msg::MapMetaData>(metadata_topic, metadata_qos);

  // Latched publisher for data
  std::string map_topic = "maps/" + ns + "/" + map_name + "/map";
  auto map_qos = rclcpp::QoS(1).transient_local();
  map_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic, map_qos);
  
  // Publish immediately
  metadata_pub_->publish(meta_data_);
  map_pub_->publish(map_);

  RCLCPP_DEBUG(
    node_->get_logger(),
    "Loaded map '%s' in environment '%s' with %dx%d cells at %.3f m/cell",
    map_name.c_str(), ns.c_str(), 
    map_.info.width, map_.info.height, map_.info.resolution);
}

std::string MultimapMapEntry::getMapFullName() const
{
  return map_full_name_;
}

std::string MultimapMapEntry::getNamespace() const
{
  return namespace_;
}

std::string MultimapMapEntry::getMapName() const
{
  return map_name_;
}

void MultimapMapEntry::mapCallback(
  const std::shared_ptr<nav_msgs::srv::GetMap::Request> /*request*/,
  std::shared_ptr<nav_msgs::srv::GetMap::Response> response)
{
  response->map = map_;
  RCLCPP_INFO(node_->get_logger(), "Sending map '%s'", map_full_name_.c_str());
}

MultimapServer::MultimapServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("multimap_server", "", options)
{
  declare_parameter("environments_file", "");
}

MultimapServer::~MultimapServer()
{
  maps_.clear();
}

nav2_util::CallbackReturn
MultimapServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring multimap server");

  // Get parameters
  get_parameter("environments_file", environments_file_);

  // Create publishers
  // Use transient_local durability so late-joining subscribers receive the last message
  // This is a latched topic that publishes environment data once at startup
  rclcpp::QoS env_qos(1);
  env_qos.transient_local();
  environments_pub_ = create_publisher<navbim_msgs::msg::Environments>("environments", env_qos);

  // Create services
  load_map_service_ = create_service<navbim_msgs::srv::LoadMultiMap>(
    "load_map",
    std::bind(&MultimapServer::loadMapCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  load_environments_service_ = create_service<navbim_msgs::srv::LoadEnvironments>(
    "load_environments",
    std::bind(&MultimapServer::loadEnvironmentsCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  dump_map_service_ = create_service<navbim_msgs::srv::DumpMultiMap>(
    "dump_map",
    std::bind(&MultimapServer::dumpMapCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  dump_environments_service_ = create_service<std_srvs::srv::Trigger>(
    "dump_environments",
    std::bind(&MultimapServer::dumpEnvironmentsCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  // Load initial environments if specified
  if (!environments_file_.empty()) {
    if (!loadEnvironmentsFromYAML(environments_file_)) {
      RCLCPP_ERROR(get_logger(), "Failed to load environments from file: %s", environments_file_.c_str());
      return nav2_util::CallbackReturn::FAILURE;
    }
  }

  RCLCPP_INFO(get_logger(), "Multimap server configured successfully");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MultimapServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating multimap server");
  
  environments_pub_->on_activate();
  
  // Publish environments once (transient_local QoS will deliver to late joiners)
  publishEnvironments();
  
  // create bond connection
  createBond();
  
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MultimapServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating multimap server");
  
  environments_pub_->on_deactivate();
  
  // destroy bond connection
  destroyBond();
  
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MultimapServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up multimap server");
  
  maps_.clear();
  environments_.environments.clear();
  
  environments_pub_.reset();
  load_map_service_.reset();
  load_environments_service_.reset();
  dump_map_service_.reset();
  dump_environments_service_.reset();
  
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MultimapServer::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down multimap server");
  return nav2_util::CallbackReturn::SUCCESS;
}

bool MultimapServer::loadEnvironmentsFromYAML(const std::string & filename)
{
  std::ifstream file(filename);
  if (!file.is_open()) {
    RCLCPP_ERROR(get_logger(), "Could not open environments file: %s", filename.c_str());
    return false;
  }

  // Extract the directory path from the environments file path
  std::string environments_dir = filename.substr(0, filename.find_last_of("/\\"));
  if (environments_dir == filename) {
    // No directory separator found, use current directory
    environments_dir = ".";
  }

  try {
    YAML::Node doc = YAML::Load(file);
    
    for (const auto & env_node : doc) {
      navbim_msgs::msg::Environment new_environment;
      
      std::string env_name = env_node.first.as<std::string>();
      auto env_config = env_node.second;
      
      new_environment.name = env_name;
      new_environment.global_frame = env_config["global_frame"].as<std::string>();
      
      // Use the environments file directory as the maps_path
      std::string maps_path = environments_dir;
      
      auto maps = env_config["maps"];
      bool all_maps_loaded = true;
      
      for (const auto & map_node : maps) {
        std::string map_name = map_node.first.as<std::string>();
        std::string map_file = map_node.second.as<std::string>();
        std::string full_map_path = maps_path + "/" + map_file;
        
        if (isMapAlreadyLoaded(env_name, map_name)) {
          RCLCPP_WARN(get_logger(), "Map %s/%s already loaded", env_name.c_str(), map_name.c_str());
          continue;
        }
        
        try {
          auto new_map = std::make_unique<MultimapMapEntry>(
            full_map_path, env_name, map_name, new_environment.global_frame, shared_from_this());
          maps_.push_back(std::move(new_map));
          new_environment.map_name.push_back(map_name);
        } catch (const std::exception & e) {
          RCLCPP_ERROR(get_logger(), "Failed to load map %s: %s", full_map_path.c_str(), e.what());
          all_maps_loaded = false;
        }
      }
      
      if (all_maps_loaded && !new_environment.map_name.empty()) {
        environments_.environments.push_back(new_environment);
      }
    }
    
    RCLCPP_INFO(get_logger(), "Successfully loaded %zu environments", environments_.environments.size());
    return true;
    
  } catch (const YAML::Exception & e) {
    RCLCPP_ERROR(get_logger(), "YAML parsing error: %s", e.what());
    return false;
  }
}

bool MultimapServer::isMapAlreadyLoaded(const std::string & ns, const std::string & map_name) const
{
  std::string full_name = ns + "/" + map_name;
  for (const auto & map : maps_) {
    if (map->getMapFullName() == full_name) {
      return true;
    }
  }
  return false;
}

void MultimapServer::publishEnvironments()
{
  if (environments_pub_ && environments_pub_->is_activated()) {
    environments_pub_->publish(environments_);
  }
}

void MultimapServer::loadMapCallback(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<navbim_msgs::srv::LoadMultiMap::Request> request,
  std::shared_ptr<navbim_msgs::srv::LoadMultiMap::Response> response)
{
  if (isMapAlreadyLoaded(request->ns, request->map_name)) {
    response->success = false;
    response->msg = "Map " + request->ns + "/" + request->map_name + " is already loaded";
    return;
  }

  try {
    auto new_map = std::make_unique<MultimapMapEntry>(
      request->map_url, request->ns, request->map_name, request->global_frame, shared_from_this());
    maps_.push_back(std::move(new_map));

    // Update or create environment
    bool env_exists = false;
    for (auto & env : environments_.environments) {
      if (env.name == request->ns) {
        env.map_name.push_back(request->map_name);
        env_exists = true;
        if (!request->global_frame.empty() && request->global_frame != env.global_frame) {
          RCLCPP_WARN(get_logger(), 
            "Global frame mismatch for environment %s. Using existing: %s", 
            request->ns.c_str(), env.global_frame.c_str());
        }
        break;
      }
    }

    if (!env_exists) {
      navbim_msgs::msg::Environment new_env;
      new_env.name = request->ns;
      new_env.global_frame = request->global_frame;
      new_env.map_name.push_back(request->map_name);
      environments_.environments.push_back(new_env);
    }

    response->success = true;
    response->msg = "Successfully loaded map: " + request->map_url;
    
  } catch (const std::exception & e) {
    response->success = false;
    response->msg = "Failed to load map: " + std::string(e.what());
  }
}

void MultimapServer::loadEnvironmentsCallback(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<navbim_msgs::srv::LoadEnvironments::Request> request,
  std::shared_ptr<navbim_msgs::srv::LoadEnvironments::Response> response)
{
  if (loadEnvironmentsFromYAML(request->environments_url)) {
    response->success = true;
    response->msg = "Successfully loaded environments from: " + request->environments_url;
  } else {
    response->success = false;
    response->msg = "Failed to load environments from: " + request->environments_url;
  }
}

void MultimapServer::dumpMapCallback(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<navbim_msgs::srv::DumpMultiMap::Request> request,
  std::shared_ptr<navbim_msgs::srv::DumpMultiMap::Response> response)
{
  std::string full_name = request->ns + "/" + request->map_name;
  
  // Remove from maps vector
  auto it = std::find_if(maps_.begin(), maps_.end(),
    [&full_name](const std::unique_ptr<MultimapMapEntry> & map) {
      return map->getMapFullName() == full_name;
    });
  
  if (it == maps_.end()) {
    response->success = false;
    response->msg = "Map " + full_name + " not found";
    return;
  }
  
  maps_.erase(it);
  
  // Remove from environments
  for (auto & env : environments_.environments) {
    if (env.name == request->ns) {
      auto map_it = std::find(env.map_name.begin(), env.map_name.end(), request->map_name);
      if (map_it != env.map_name.end()) {
        env.map_name.erase(map_it);
      }
      break;
    }
  }
  
  response->success = true;
  response->msg = "Successfully removed map: " + full_name;
}

void MultimapServer::dumpEnvironmentsCallback(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  maps_.clear();
  environments_.environments.clear();
  
  response->success = true;
  response->message = "All environments and maps have been dumped";
}

}  // namespace navbim_multimap_server

RCLCPP_COMPONENTS_REGISTER_NODE(navbim_multimap_server::MultimapServer)