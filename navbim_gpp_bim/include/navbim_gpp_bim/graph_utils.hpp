#ifndef NAVBIM_GPP_BIM__GRAPH_UTILS_HPP_
#define NAVBIM_GPP_BIM__GRAPH_UTILS_HPP_

#include <array>
#include <optional>
#include <string>
#include <vector>

#include "navbim_util/topological_map.hpp"

namespace navbim_gpp_bim
{

/**
 * @brief Insert start and goal nodes into topological graph
 * 
 * Creates a modified copy of the graph with start/goal nodes and connecting edges.
 * Uses cached max IDs from TopologicalMap for O(1) ID generation.
 * 
 * @param topological_map The topological map wrapper (provides cached max IDs)
 * @param start_room Vertex descriptor of start room
 * @param end_room Vertex descriptor of end room
 * @param start_coords 3D coordinates [x, y, z] of start position
 * @param end_coords 3D coordinates [x, y, z] of goal position
 * @param penalize_z_movement Penalty factor for vertical movement (default 1.0)
 * @return Modified graph with start and goal nodes
 */
navbim_util::TopologicalGraph insertStartAndGoal(
  const navbim_util::TopologicalMap & topological_map,
  navbim_util::Vertex start_room,
  navbim_util::Vertex end_room,
  const std::array<double, 3> & start_coords,
  const std::array<double, 3> & end_coords,
  double penalize_z_movement = 1.0);

/**
 * @brief Prunes the topological graph by removing unnecessary vertices and edges.
 *
 * This function prunes a graph with Start and Goal nodes to remove dead-ends.
 *
 * @param graph The topological graph to prune (modified in place)
 * @return A reference to the pruned graph
 */
navbim_util::TopologicalGraph & pruneGraph(
  navbim_util::TopologicalGraph & graph);

/**
 * @brief Prunes the topological graph with cached floor vertices and optional early exit.
 *
 * Optimized version that uses pre-cached floor vertices and checks for early exit
 * when start and goal are in the same room. Returns set of disabled vertices instead
 * of physically removing them (Phase 5 optimization - saves ~333ms).
 *
 * @param graph The topological graph (NOT modified - disabled vertices returned)
 * @param floor_vertices Pre-cached set of floor vertices (from TopologicalMap)
 * @param start_room Optional start room vertex for early exit check
 * @param goal_room Optional goal room vertex for early exit check
 * @return Set of vertices that should be disabled/ignored during planning
 */
std::set<navbim_util::Vertex> pruneGraph(
  const navbim_util::TopologicalGraph & graph,
  const std::set<navbim_util::Vertex> & floor_vertices,
  std::optional<navbim_util::Vertex> start_room = std::nullopt,
  std::optional<navbim_util::Vertex> goal_room = std::nullopt);

/**
 * @brief Find vertex by node ID
 * @param graph The topological graph
 * @param node_id Node ID to find
 * @return Optional vertex descriptor
 */
std::optional<navbim_util::Vertex> findVertexById(
  const navbim_util::TopologicalGraph & graph,
  int node_id);

/**
 * @brief Find floor vertex by height (z-coordinate)
 * 
 * Finds the floor node where min_z <= height < max_z.
 * If multiple floors match, chooses the one with smallest (height - min_z).
 * 
 * @param graph The topological graph
 * @param height The z-coordinate to search for
 * @return Optional vertex descriptor of the floor node
 */
std::optional<navbim_util::Vertex> findFloorByHeight(
  const navbim_util::TopologicalGraph & graph,
  double height);

/**
 * @brief Find room vertex by coordinates using point-in-polygon test
 * 
 * First finds the correct floor by height, then searches rooms on that floor.
 * Uses expanding buffer strategy (0.0 to 0.5m in 0.1m increments) for tolerance.
 * 
 * @param graph The topological graph
 * @param x X-coordinate
 * @param y Y-coordinate
 * @param z Z-coordinate (used to determine floor first)
 * @return Optional vertex descriptor of the room node
 */
std::optional<navbim_util::Vertex> findRoomByCoordinates(
  const navbim_util::TopologicalGraph & graph,
  double x,
  double y,
  double z);

/**
 * @brief Check if a point is inside a polygon
 * 
 * Uses Boost.Geometry for point-in-polygon test.
 * 
 * @param polygon The room polygon
 * @param x X-coordinate
 * @param y Y-coordinate
 * @return True if point is inside polygon, false otherwise
 */
bool isPointInPolygon(
  const navbim_util::Polygon & polygon,
  double x,
  double y);

}  // namespace navbim_gpp_bim

#endif  // NAVBIM_GPP_BIM__GRAPH_UTILS_HPP_
