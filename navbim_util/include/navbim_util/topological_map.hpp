#ifndef NAVBIM_UTIL__TOPOLOGICAL_MAP_HPP_
#define NAVBIM_UTIL__TOPOLOGICAL_MAP_HPP_

#include <string>
#include <vector>
#include <unordered_map>
#include <set>
#include <optional>
#include <cmath>

#include "navbim_util/topological_map_types.hpp"
#include "navbim_msgs/msg/topomap.hpp"
#include "navbim_msgs/msg/topomap_node.hpp"
#include "navbim_msgs/msg/topomap_edge.hpp"

namespace navbim_util
{

/**
 * @brief TopologicalMap class for path planning in BIM environments
 * 
 * This class provides a C++ interface to the topological map retrieved from
 * the topomap_server. It deserializes the Topomap message into a Boost.Graph
 * structure and provides query/manipulation methods for path planning.
 */
class TopologicalMap
{
public:
  /**
   * @brief Constructor
   */
  TopologicalMap();

  /**
   * @brief Destructor
   */
  ~TopologicalMap() = default;

  /**
   * @brief Load topological map from Topomap message
   * @param topomap_msg The Topomap message from topomap_server
   * @return True if loaded successfully
   */
  bool loadFromMessage(const navbim_msgs::msg::Topomap & topomap_msg);

  /**
   * @brief Convert topological graph to Topomap message
   * @param graph The topological graph to convert
   * @param topomap_msg Output Topomap message (metadata will be empty)
   */
  static void convertGraphToMessage(
    const TopologicalGraph & graph,
    navbim_msgs::msg::Topomap & topomap_msg);

  /**
   * @brief Get the underlying Boost.Graph
   * @return Reference to the graph
   */
  TopologicalGraph & getGraph() { return graph_; }
  const TopologicalGraph & getGraph() const { return graph_; }

  /**
   * @brief Get metadata
   */
  const std::string & getSourceFile() const { return source_file_; }
  const std::string & getBuildingName() const { return building_name_; }
  uint32_t getTotalFloors() const { return total_floors_; }
  const std::vector<std::string> & getFloorNames() const { return floor_names_; }

  /**
   * @brief Find vertex by node ID
   * @param node_id Node ID as string
   * @return Optional vertex descriptor
   */
  std::optional<Vertex> findVertexById(const std::string & node_id) const;

  /**
   * @brief Find vertex by node name
   * @param node_name Node name
   * @return Optional vertex descriptor
   */
  std::optional<Vertex> findVertexByName(const std::string & node_name) const;

  /**
   * @brief Get all vertices of a specific type
   * @param type Node type ("floor", "room", "transition", "stair")
   * @return Vector of vertices
   */
  std::vector<Vertex> getVerticesByType(const std::string & type) const;

  /**
   * @brief Check if graph is empty
   * @return True if no vertices
   */
  bool isEmpty() const;

  /**
   * @brief Get number of vertices
   */
  size_t numVertices() const;

  /**
   * @brief Get number of edges
   */
  size_t numEdges() const;

  /**
   * @brief Clear the graph
   */
  void clear();

  /**
   * @brief Get the next available node ID
   * 
   * Returns the next available node ID for creating new nodes.
   * This is cached for O(1) performance.
   * 
   * @return Next available node ID (max_node_id + 1)
   */
  int getMaxNodeId() const { return cached_max_node_id_; }

  /**
   * @brief Get the next available edge ID
   * 
   * Returns the next available edge ID for creating new edges.
   * This is cached for O(1) performance.
   * 
   * @return Next available edge ID (max_edge_id + 1)
   */
  int getMaxEdgeId() const { return cached_max_edge_id_; }

  /**
   * @brief Get cached floor vertices
   * 
   * Returns a set of all floor-type vertices for efficient pruning.
   * This is cached for O(1) access.
   * 
   * @return Set of floor vertices
   */
  const std::set<Vertex> & getFloorVertices() const { return floor_vertices_; }

  /**
   * @brief Calculate Euclidean distance between two nodes in 3D space
   * @param v1 First vertex descriptor
   * @param v2 Second vertex descriptor
   * @return Euclidean distance
   */
  double euclideanDistanceBetweenNodes(Vertex v1, Vertex v2) const;

  /**
   * @brief Calculate cost between two nodes with optional Z-axis penalty
   * 
   * This is useful for path planning where vertical movement may be more
   * expensive than horizontal movement (e.g., stairs, ramps).
   * 
   * @param v1 First vertex descriptor
   * @param v2 Second vertex descriptor
   * @param penalize_z_movement Penalty factor for vertical movement (default 1.0)
   * @return Penalized cost
   */
  double costBetweenNodes(Vertex v1, Vertex v2, double penalize_z_movement = 1.0) const;

  /**
   * @brief Check if two floor specifications refer to the same floor
   * 
   * Handles both single floor strings and lists of floors (e.g., for transitions
   * that span multiple floors).
   * 
   * @param floor1 First floor specification (optional vector of floor names)
   * @param floor2 Second floor specification (optional vector of floor names)
   * @return True if floors overlap, false otherwise
   */
  static bool isOnSameFloor(
    const std::optional<std::vector<std::string>> & floor1,
    const std::optional<std::vector<std::string>> & floor2);

private:
  /**
   * @brief Convert ROS Polygon message to Boost.Geometry polygon
   * @param polygon_msg ROS polygon message with holes
   * @return Boost.Geometry polygon
   */
  Polygon convertPolygonFromMessage(const navbim_msgs::msg::Polygon & polygon_msg) const;

  /**
   * @brief Build lookup maps for efficient vertex queries
   */
  void buildLookupMaps();

  /**
   * @brief Update cached max node and edge IDs
   * 
   * This method scans the graph to find the maximum node and edge IDs
   * and caches them for O(1) retrieval. Called automatically after loading.
   */
  void updateMaxIds();

  // Graph data
  TopologicalGraph graph_;
  
  // Metadata
  std::string source_file_;
  std::string building_name_;
  uint32_t total_floors_;
  std::vector<std::string> floor_names_;

  // Lookup maps for efficient queries
  std::unordered_map<std::string, Vertex> id_to_vertex_;    // node_id -> vertex
  std::unordered_map<std::string, Vertex> name_to_vertex_;  // node_name -> vertex

  // Cached max IDs for O(1) ID generation
  int cached_max_node_id_;
  int cached_max_edge_id_;

  // Cached floor vertices for O(1) pruning
  std::set<Vertex> floor_vertices_;
};

}  // namespace navbim_util

// Standalone geometry utility functions that work with TopologicalGraph directly
namespace navbim_util
{

/**
 * @brief Calculate Euclidean distance between two nodes in 3D space
 * @param graph The topological graph
 * @param v1 First vertex descriptor
 * @param v2 Second vertex descriptor
 * @return Euclidean distance
 */
inline double euclideanDistanceBetweenNodes(
  const TopologicalGraph & graph,
  Vertex v1,
  Vertex v2)
{
  const auto & p1 = graph[v1].position;
  const auto & p2 = graph[v2].position;
  
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  double dz = p1.z - p2.z;
  
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

/**
 * @brief Calculate cost between two nodes with optional Z-axis penalty
 * 
 * This is useful for path planning where vertical movement may be more
 * expensive than horizontal movement (e.g., stairs, ramps).
 * 
 * @param graph The topological graph
 * @param v1 First vertex descriptor
 * @param v2 Second vertex descriptor
 * @param penalize_z_movement Penalty factor for vertical movement (default 1.0)
 * @return Penalized cost
 */
inline double costBetweenNodes(
  const TopologicalGraph & graph,
  Vertex v1,
  Vertex v2,
  double penalize_z_movement = 1.0)
{
  const auto & p1 = graph[v1].position;
  const auto & p2 = graph[v2].position;
  
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  double dz = (p1.z - p2.z) * penalize_z_movement;
  
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

/**
 * @brief Check if two floor specifications refer to the same floor
 * 
 * Handles both single floor strings and lists of floors (e.g., for transitions
 * that span multiple floors).
 * 
 * @param floor1 First floor specification (optional vector of floor names)
 * @param floor2 Second floor specification (optional vector of floor names)
 * @return True if floors overlap, false otherwise
 */
inline bool isOnSameFloor(
  const std::optional<std::vector<std::string>> & floor1,
  const std::optional<std::vector<std::string>> & floor2)
{
  if (!floor1.has_value() || !floor2.has_value()) {
    return false;
  }
  
  // Check if any floor in floor1 matches any floor in floor2
  for (const auto & f1 : *floor1) {
    for (const auto & f2 : *floor2) {
      if (f1 == f2) {
        return true;
      }
    }
  }
  
  return false;
}

}  // namespace navbim_util

#endif  // NAVBIM_UTIL__TOPOLOGICAL_MAP_HPP_
