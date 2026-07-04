#ifndef NAVBIM_MULTIMAP_SERVER__MULTIMAP_SERVER_HPP_
#define NAVBIM_MULTIMAP_SERVER__MULTIMAP_SERVER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "navbim_msgs/msg/environment.hpp"
#include "navbim_msgs/msg/environments.hpp"
#include "navbim_msgs/srv/load_multi_map.hpp"
#include "navbim_msgs/srv/load_environments.hpp"
#include "navbim_msgs/srv/dump_multi_map.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "navbim_multimap_server/map_io.hpp"
#include "rmw/types.h"

namespace navbim_multimap_server
{

/**
 * @brief Single map container class for multimap functionality
 */
class MultimapMapEntry
{
public:
  /**
   * @brief Constructor for a map entry
   * @param map_url Path to the map YAML file
   * @param ns Namespace/environment name
   * @param map_name Desired name for the map
   * @param global_frame Global coordinate frame
   * @param node Shared pointer to the ROS2 node
   */
  MultimapMapEntry(
    const std::string & map_url,
    const std::string & ns,
    const std::string & map_name,
    const std::string & global_frame,
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node);

  /**
   * @brief Get the full map name (namespace/map_name)
   * @return Full map identifier
   */
  std::string getMapFullName() const;

  /**
   * @brief Get the namespace of this map
   * @return Namespace/environment name
   */
  std::string getNamespace() const;

  /**
   * @brief Get the map name
   * @return Map name
   */
  std::string getMapName() const;

private:
  std::string map_full_name_;
  std::string namespace_;
  std::string map_name_;
  nav_msgs::msg::OccupancyGrid map_;
  nav_msgs::msg::MapMetaData meta_data_;
  
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<nav_msgs::msg::MapMetaData>::SharedPtr metadata_pub_;
  rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr map_service_;
  
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;

  /**
   * @brief Service callback for map requests
   */
  void mapCallback(
    const std::shared_ptr<nav_msgs::srv::GetMap::Request> request,
    std::shared_ptr<nav_msgs::srv::GetMap::Response> response);
};

/**
 * @brief Main multimap server class
 */
class MultimapServer : public nav2_util::LifecycleNode
{
public:
  /**
   * @brief Constructor
   * @param options Node options
   */
  explicit MultimapServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor
   */
  ~MultimapServer();

protected:
  /**
   * @brief Configure lifecycle transition
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Activate lifecycle transition
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Deactivate lifecycle transition
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Cleanup lifecycle transition
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Shutdown lifecycle transition
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  // Map storage
  std::vector<std::unique_ptr<MultimapMapEntry>> maps_;
  navbim_msgs::msg::Environments environments_;

  // Publishers
  rclcpp_lifecycle::LifecyclePublisher<navbim_msgs::msg::Environments>::SharedPtr environments_pub_;

  // Services
  rclcpp::Service<navbim_msgs::srv::LoadMultiMap>::SharedPtr load_map_service_;
  rclcpp::Service<navbim_msgs::srv::LoadEnvironments>::SharedPtr load_environments_service_;
  rclcpp::Service<navbim_msgs::srv::DumpMultiMap>::SharedPtr dump_map_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr dump_environments_service_;

  // Parameters
  std::string environments_file_;

  /**
   * @brief Load environments from YAML file
   * @param filename Path to the environments configuration file
   * @return True if successful
   */
  bool loadEnvironmentsFromYAML(const std::string & filename);

  /**
   * @brief Check if a map is already loaded
   * @param ns Namespace
   * @param map_name Map name
   * @return True if map is already loaded
   */
  bool isMapAlreadyLoaded(const std::string & ns, const std::string & map_name) const;

  /**
   * @brief Timer callback for publishing environments
   */
  void publishEnvironments();

  // Service callbacks
  void loadMapCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<navbim_msgs::srv::LoadMultiMap::Request> request,
    std::shared_ptr<navbim_msgs::srv::LoadMultiMap::Response> response);

  void loadEnvironmentsCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<navbim_msgs::srv::LoadEnvironments::Request> request,
    std::shared_ptr<navbim_msgs::srv::LoadEnvironments::Response> response);

  void dumpMapCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<navbim_msgs::srv::DumpMultiMap::Request> request,
    std::shared_ptr<navbim_msgs::srv::DumpMultiMap::Response> response);

  void dumpEnvironmentsCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
};

}  // namespace navbim_multimap_server

#endif  // NAVBIM_MULTIMAP_SERVER__MULTIMAP_SERVER_HPP_