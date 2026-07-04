#ifndef NAVBIM_TOPOMAP_SERVER__TOPOMAP_SERVER_HPP_
#define NAVBIM_TOPOMAP_SERVER__TOPOMAP_SERVER_HPP_

#include <memory>
#include <string>
#include <mutex>

#include "nav2_util/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "navbim_msgs/srv/get_floor_nodes.hpp"
#include "navbim_msgs/srv/get_room_nodes.hpp"
#include "navbim_msgs/srv/get_room_neighbors.hpp"
#include "navbim_msgs/srv/get_adjacent_transition_nodes.hpp"
#include "navbim_msgs/srv/load_topomap.hpp"
#include "navbim_msgs/srv/get_room_by_coordinates.hpp"
#include "navbim_msgs/srv/get_topological_map.hpp"
#include "navbim_msgs/srv/get_min_z.hpp"
#include "navbim_msgs/srv/is_point_in_room_polygon.hpp"
#include "navbim_msgs/srv/update_edge_data.hpp"
#include "navbim_msgs/srv/save_topological_map.hpp"
#include "navbim_msgs/srv/clear_topological_map_paths.hpp"

#include "navbim_util/topological_map_types.hpp"
#include "navbim_topomap_server/topological_map_loader.hpp"
#include "navbim_topomap_server/topological_map_utils.hpp"

#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace navbim_topomap_server
{

// Use types from navbim_util
using navbim_util::TopologicalGraph;
using navbim_util::Vertex;

// For compatibility: TopologicalMap in topomap_server refers to the graph type
using TopologicalMap = TopologicalGraph;

/**
 * @brief C++ lifecycle node for topological map server using Boost.Graph
 * 
 * This node provides services for querying topological map data and integrates
 * seamlessly with Nav2's lifecycle management system.
 */
class TopomapServer : public nav2_util::LifecycleNode
{
public:
  /**
   * @brief Constructor
   * @param options Node options
   */
  explicit TopomapServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor
   */
  ~TopomapServer();

protected:
  /**
   * @brief Lifecycle callback for configure transition
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Lifecycle callback for activate transition
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Lifecycle callback for deactivate transition
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Lifecycle callback for cleanup transition
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Lifecycle callback for shutdown transition
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Lifecycle callback for error transition
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_error(const rclcpp_lifecycle::State & state) override;

private:
  /**
   * @brief Service callback to get all floor nodes
   */
  void getFloorNodesCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<navbim_msgs::srv::GetFloorNodes::Request> request,
    std::shared_ptr<navbim_msgs::srv::GetFloorNodes::Response> response);

  /**
   * @brief Service callback to get all room nodes
   */
  void getRoomNodesCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<navbim_msgs::srv::GetRoomNodes::Request> request,
    std::shared_ptr<navbim_msgs::srv::GetRoomNodes::Response> response);

  /**
   * @brief Service callback to get room neighbors
   */
  void getRoomNeighborsCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<navbim_msgs::srv::GetRoomNeighbors::Request> request,
    std::shared_ptr<navbim_msgs::srv::GetRoomNeighbors::Response> response);

  /**
   * @brief Service callback to get adjacent transition nodes for a room
   */
  void getAdjacentTransitionNodesCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<navbim_msgs::srv::GetAdjacentTransitionNodes::Request> request,
    std::shared_ptr<navbim_msgs::srv::GetAdjacentTransitionNodes::Response> response);

  /**
   * @brief Service callback to load topological map from file
   */
  void loadTopomapCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<navbim_msgs::srv::LoadTopomap::Request> request,
    std::shared_ptr<navbim_msgs::srv::LoadTopomap::Response> response);

  /**
   * @brief Service callback to get room by coordinates
   */
  void getRoomByCoordinatesCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<navbim_msgs::srv::GetRoomByCoordinates::Request> request,
    std::shared_ptr<navbim_msgs::srv::GetRoomByCoordinates::Response> response);

  /**
   * @brief Service callback to get complete topological map
   */
  void getTopologicalMapCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<navbim_msgs::srv::GetTopologicalMap::Request> request,
    std::shared_ptr<navbim_msgs::srv::GetTopologicalMap::Response> response);

  /**
   * @brief Service callback to get minimum z-coordinate by node name
   */
  void getMinZCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<navbim_msgs::srv::GetMinZ::Request> request,
    std::shared_ptr<navbim_msgs::srv::GetMinZ::Response> response);

  /**
   * @brief Service callback to check if point is in room polygon
   */
  void isPointInRoomPolygonCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<navbim_msgs::srv::IsPointInRoomPolygon::Request> request,
    std::shared_ptr<navbim_msgs::srv::IsPointInRoomPolygon::Response> response);

  /**
   * @brief Service callback to update edge data with planned path
   */
  void updateEdgeDataCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<navbim_msgs::srv::UpdateEdgeData::Request> request,
    std::shared_ptr<navbim_msgs::srv::UpdateEdgeData::Response> response);

  /**
   * @brief Service callback to save topological map to disk
   */
  void saveTopologicalMapCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<navbim_msgs::srv::SaveTopologicalMap::Request> request,
    std::shared_ptr<navbim_msgs::srv::SaveTopologicalMap::Response> response);

  /**
   * @brief Service callback to clear all pre-planned paths from transition edges
   */
  void clearTopologicalMapPathsCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<navbim_msgs::srv::ClearTopologicalMapPaths::Request> request,
    std::shared_ptr<navbim_msgs::srv::ClearTopologicalMapPaths::Response> response);

  /**
   * @brief Load topological map from file
   * @param filename Path to the JSON file
   * @return True if loaded successfully
   */
  bool loadFromFile(const std::string & filename);

  /**
   * @brief Clear the topological map graph
   */
  void clearGraph();

  /**
   * @brief Convert internal NodeProperties to ROS TopomapNode message
   * @param node_props Internal node properties
   * @return TopomapNode message
   */
  navbim_msgs::msg::TopomapNode convertToTopomapNodeMsg(const NodeProperties & node_props) const;

  /**
   * @brief Convert internal EdgeProperties to ROS TopomapEdge message
   * @param edge_props Internal edge properties
   * @param source_vertex Source vertex descriptor
   * @param target_vertex Target vertex descriptor
   * @return TopomapEdge message
   */
  navbim_msgs::msg::TopomapEdge convertToTopomapEdgeMsg(
    const EdgeProperties & edge_props,
    Vertex source_vertex,
    Vertex target_vertex) const;

  // Topological map data
  TopologicalMap graph_;
  std::string topomap_file_;
  std::mutex graph_mutex_;  // Thread-safe access to graph

  // Metadata for complete topomap serialization
  std::string building_name_;
  std::vector<std::string> floor_names_;

  // Parameters
  std::string global_frame_;
  bool save_at_shutdown_;

  // Service servers (using nav2::ServiceServer)
  rclcpp::Service<navbim_msgs::srv::GetFloorNodes>::SharedPtr get_floor_nodes_service_;
  rclcpp::Service<navbim_msgs::srv::GetRoomNodes>::SharedPtr get_room_nodes_service_;
  rclcpp::Service<navbim_msgs::srv::GetRoomNeighbors>::SharedPtr get_room_neighbors_service_;
  rclcpp::Service<navbim_msgs::srv::GetAdjacentTransitionNodes>::SharedPtr get_adjacent_transition_nodes_service_;
  rclcpp::Service<navbim_msgs::srv::LoadTopomap>::SharedPtr load_topomap_service_;
  rclcpp::Service<navbim_msgs::srv::GetRoomByCoordinates>::SharedPtr get_room_by_coordinates_service_;
  rclcpp::Service<navbim_msgs::srv::GetTopologicalMap>::SharedPtr get_topological_map_service_;
  rclcpp::Service<navbim_msgs::srv::GetMinZ>::SharedPtr get_min_z_service_;
  rclcpp::Service<navbim_msgs::srv::IsPointInRoomPolygon>::SharedPtr is_point_in_room_polygon_service_;
  rclcpp::Service<navbim_msgs::srv::UpdateEdgeData>::SharedPtr update_edge_data_service_;
  rclcpp::Service<navbim_msgs::srv::SaveTopologicalMap>::SharedPtr save_topological_map_service_;
  rclcpp::Service<navbim_msgs::srv::ClearTopologicalMapPaths>::SharedPtr clear_topological_map_paths_service_;
};

}  // namespace navbim_topomap_server

#endif  // NAVBIM_TOPOMAP_SERVER__TOPOMAP_SERVER_HPP_
