#ifndef NAVBIM_TOPOMAP_SERVER__TOPOLOGICAL_MAP_UTILS_HPP_
#define NAVBIM_TOPOMAP_SERVER__TOPOLOGICAL_MAP_UTILS_HPP_

#include <vector>
#include <string>
#include <optional>
#include <boost/geometry.hpp>

#include "navbim_util/topological_map_types.hpp"

namespace navbim_topomap_server
{

// Use types from navbim_util
using navbim_util::TopologicalGraph;
using navbim_util::NodeProperties;
using navbim_util::Polygon;
using navbim_util::Point2D;
using navbim_util::Vertex;

// For compatibility: TopologicalMap in topomap_server refers to the graph type
using TopologicalMap = TopologicalGraph;

/**
 * @brief Utility functions for querying topological maps
 */
class TopologicalMapUtils
{
public:
  /**
   * @brief Find floor node by height (z-coordinate)
   * @param map The topological map
   * @param height The z-coordinate to search for
   * @return Vertex descriptor of the floor node, or nullopt if not found
   */
  static std::optional<Vertex> findFloorByHeight(
    const TopologicalMap & map,
    double height);

  /**
   * @brief Find room node by coordinates (point-in-polygon test)
   * @param map The topological map
   * @param x X-coordinate
   * @param y Y-coordinate
   * @param z Z-coordinate (used to determine floor first)
   * @return Vertex descriptor of the room node, or nullopt if not found
   */
  static std::optional<Vertex> findRoomByCoordinates(
    const TopologicalMap & map,
    double x,
    double y,
    double z);

  /**
   * @brief Get all floor nodes from the map
   * @param map The topological map
   * @return Vector of vertex descriptors for all floor nodes
   */
  static std::vector<Vertex> getFloorNodes(const TopologicalMap & map);

  /**
   * @brief Get all room nodes from the map
   * @param map The topological map
   * @return Vector of vertex descriptors for all room nodes
   */
  static std::vector<Vertex> getRoomNodes(const TopologicalMap & map);

  /**
   * @brief Get all room nodes on a specific floor
   * @param map The topological map
   * @param floor_id ID of the floor node
   * @return Vector of vertex descriptors for rooms on that floor
   */
  static std::vector<Vertex> getRoomNodesOnFloor(
    const TopologicalMap & map,
    const std::string & floor_id);

  /**
   * @brief Get neighboring rooms of a given room
   * @param map The topological map
   * @param room_id ID of the room node
   * @return Vector of vertex descriptors for neighboring rooms
   */
  static std::vector<Vertex> getRoomNeighbors(
    const TopologicalMap & map,
    const std::string & room_id);

  /**
   * @brief Get adjacent transition nodes for a given room
   * @param map The topological map
   * @param room_name Name of the room node
   * @return Vector of vertex descriptors for adjacent transition nodes
   */
  static std::vector<Vertex> getAdjacentTransitionNodes(
    const TopologicalMap & map,
    const std::string & room_name);

  /**
   * @brief Check if a point is inside a room's polygon
   * @param polygon The room polygon
   * @param x X-coordinate
   * @param y Y-coordinate
   * @return True if point is inside polygon, false otherwise
   */
  static bool isPointInPolygon(
    const Polygon & polygon,
    double x,
    double y);

  /**
   * @brief Get vertex by node ID (searches node.id field as string)
   * @param map The topological map
   * @param node_id The node ID to search for (as string, e.g., "3")
   * @return Vertex descriptor if found, or nullopt if not found
   */
  static std::optional<Vertex> getVertexById(
    const TopologicalMap & map,
    const std::string & node_id);

  /**
   * @brief Get vertex by node name (searches node.name field)
   * @param map The topological map
   * @param node_name The node name to search for (e.g., "Room_1")
   * @return Vertex descriptor if found, or nullopt if not found
   */
  static std::optional<Vertex> getVertexByName(
    const TopologicalMap & map,
    const std::string & node_name);

  /**
   * @brief Get minimum z-coordinate across all nodes in the map
   * @param map The topological map
   * @return Minimum z value, or nullopt if map is empty
   */
  static std::optional<double> getMinZ(const TopologicalMap & map);

  /**
   * @brief Get maximum z-coordinate across all nodes in the map
   * @param map The topological map
   * @return Maximum z value, or nullopt if map is empty
   */
  static std::optional<double> getMaxZ(const TopologicalMap & map);

private:
  /**
   * @brief Helper to check if two floors are approximately equal in height
   * @param z1 First z-coordinate
   * @param z2 Second z-coordinate
   * @param tolerance Tolerance for comparison (default 0.1m)
   * @return True if heights are approximately equal
   */
  static bool isApproximatelyEqual(
    double z1,
    double z2,
    double tolerance = 0.1);
};

}  // namespace navbim_topomap_server

#endif  // NAVBIM_TOPOMAP_SERVER__TOPOLOGICAL_MAP_UTILS_HPP_
