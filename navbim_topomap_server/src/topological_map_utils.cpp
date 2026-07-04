#include "navbim_topomap_server/topological_map_utils.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <set>
#include <boost/graph/adjacency_list.hpp>
#include <boost/geometry/algorithms/buffer.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>

namespace navbim_topomap_server
{

// Use types from navbim_util for convenience
using navbim_util::Point2D;
using navbim_util::Polygon;
using navbim_util::NodeProperties;
using navbim_util::Vertex;
using navbim_util::Edge;
using navbim_util::VertexIterator;
using navbim_util::EdgeIterator;

std::optional<Vertex> TopologicalMapUtils::findFloorByHeight(
  const TopologicalMap & map,
  double height)
{
  // Find the floor node where min_z <= height < max_z
  // If multiple floors match, choose the one with smallest (height - min_z)
  // Python: data.get("min_z", float('-inf')) and data.get("max_z", float('inf'))
  
  double smallest_z_diff = std::numeric_limits<double>::infinity();
  std::optional<Vertex> best_floor;
  
  auto vertices = boost::vertices(map);
  for (auto it = vertices.first; it != vertices.second; ++it) {
    const auto & node = map[*it];
    
    // Check if this is a floor node with min_z and max_z
    if (node.type == "floor" && node.min_z.has_value() && node.max_z.has_value()) {
      double min_z = *node.min_z;
      double max_z = *node.max_z;
      
      // Check if height is within floor bounds
      if (min_z <= height && height < max_z) {
        double z_diff = height - min_z;
        if (z_diff < smallest_z_diff) {
          smallest_z_diff = z_diff;
          best_floor = *it;
        }
      }
    }
  }
  
  return best_floor;
}

std::optional<Vertex> TopologicalMapUtils::findRoomByCoordinates(
  const TopologicalMap & map,
  double x,
  double y,
  double z)
{
  // First, find the floor at this height
  auto floor_opt = findFloorByHeight(map, z);
  if (!floor_opt.has_value()) {
    return std::nullopt;
  }
  
  const auto & floor_node = map[*floor_opt];
  std::string floor_name = floor_node.name;
  
  // Use buffer approach: start with 0.0, increase to 0.5 in 0.1 increments
  // Python: buffer_radius = 0.0; while buffer_radius <= 0.5: ... buffer_radius += 0.1
  double buffer_radius = 0.0;
  while (buffer_radius <= 0.5) {
    auto vertices = boost::vertices(map);
    for (auto it = vertices.first; it != vertices.second; ++it) {
      const auto & node = map[*it];
      
      // Check if this is a room on the correct floor
      if (node.type == "room" && 
          node.floor.has_value() && 
          node.polygon.has_value()) {
        
        // Check if floor_name is in the floor list (Python: floor in data.get("floor", []))
        const auto & floor_list = *node.floor;
        bool floor_matches = std::find(floor_list.begin(), floor_list.end(), floor_name) != floor_list.end();
        
        if (floor_matches) {
          // Apply buffer and check if point is inside
          if (buffer_radius > 0.0) {
            // Buffer the polygon - use multi_polygon as output
            boost::geometry::model::multi_polygon<Polygon> buffered_result;
            boost::geometry::strategy::buffer::distance_symmetric<double> distance_strategy(buffer_radius);
            boost::geometry::strategy::buffer::join_round join_strategy;
            boost::geometry::strategy::buffer::end_round end_strategy;
            boost::geometry::strategy::buffer::point_circle circle_strategy;
            boost::geometry::strategy::buffer::side_straight side_strategy;
            
            boost::geometry::buffer(
              *node.polygon, buffered_result,
              distance_strategy, side_strategy,
              join_strategy, end_strategy, circle_strategy);
            
            // Check if point is in any of the buffered polygons
            Point2D point(x, y);
            for (const auto & buffered_polygon : buffered_result) {
              if (boost::geometry::within(point, buffered_polygon)) {
                return *it;
              }
            }
          } else {
            // No buffer, use original polygon
            if (isPointInPolygon(*node.polygon, x, y)) {
              return *it;
            }
          }
        }
      }
    }
    buffer_radius += 0.1;
  }
  
  return std::nullopt;
}

std::vector<Vertex> TopologicalMapUtils::getFloorNodes(const TopologicalMap & map)
{
  std::vector<Vertex> floor_nodes;
  
  auto vertices = boost::vertices(map);
  for (auto it = vertices.first; it != vertices.second; ++it) {
    const auto & node = map[*it];
    if (node.type == "floor") {
      floor_nodes.push_back(*it);
    }
  }
  
  return floor_nodes;
}

std::vector<Vertex> TopologicalMapUtils::getRoomNodes(const TopologicalMap & map)
{
  std::vector<Vertex> room_nodes;
  
  auto vertices = boost::vertices(map);
  for (auto it = vertices.first; it != vertices.second; ++it) {
    const auto & node = map[*it];
    if (node.type == "room") {
      room_nodes.push_back(*it);
    }
  }
  
  return room_nodes;
}

std::vector<Vertex> TopologicalMapUtils::getRoomNodesOnFloor(
  const TopologicalMap & map,
  const std::string & floor_name)
{
  std::vector<Vertex> room_nodes;
  
  auto vertices = boost::vertices(map);
  for (auto it = vertices.first; it != vertices.second; ++it) {
    const auto & node = map[*it];
    
    // Check if this is a room on the specified floor
    if (node.type == "room" && node.floor.has_value()) {
      const auto & floor_list = *node.floor;
      bool floor_matches = std::find(floor_list.begin(), floor_list.end(), floor_name) != floor_list.end();
      
      if (floor_matches) {
        room_nodes.push_back(*it);
      }
    }
  }
  
  return room_nodes;
}

std::vector<Vertex> TopologicalMapUtils::getRoomNeighbors(
  const TopologicalMap & map,
  const std::string & room_id)
{
  std::vector<Vertex> neighbors;
  std::set<Vertex> unique_neighbors;  // Use set to avoid duplicates
  
  // First, find the vertex for this room or transition
  auto node_vertex_opt = getVertexById(map, room_id);
  if (!node_vertex_opt.has_value()) {
    return neighbors;
  }
  
  Vertex node_vertex = *node_vertex_opt;
  const auto & node = map[node_vertex];
  std::string node_type = node.type;
  
  // Check that node is room or transition
  if (node_type != "room" && node_type != "transition") {
    return neighbors;
  }
  
  if (node_type == "room") {
    // Room nodes are connected to transition nodes, which are connected to other room nodes
    // Python: for transition_neighbor_id in graph.neighbors(node_id):
    auto adj_vertices = boost::adjacent_vertices(node_vertex, map);
    for (auto it = adj_vertices.first; it != adj_vertices.second; ++it) {
      const auto & transition_node = map[*it];
      
      // Check if the neighbor is a transition node
      if (transition_node.type == "transition") {
        // Get all room neighbors connected to this transition node, omitting the original room
        auto transition_adj = boost::adjacent_vertices(*it, map);
        for (auto room_it = transition_adj.first; room_it != transition_adj.second; ++room_it) {
          if (*room_it != node_vertex) {  // Skip the original room
            const auto & room_node = map[*room_it];
            if (room_node.type == "room") {
              unique_neighbors.insert(*room_it);
            }
          }
        }
      }
    }
  } else if (node_type == "transition") {
    // Transition nodes are connected directly to room nodes
    auto adj_vertices = boost::adjacent_vertices(node_vertex, map);
    for (auto it = adj_vertices.first; it != adj_vertices.second; ++it) {
      const auto & neighbor_node = map[*it];
      if (neighbor_node.type == "room") {
        unique_neighbors.insert(*it);
      }
    }
  }
  
  // Convert set to vector
  neighbors.assign(unique_neighbors.begin(), unique_neighbors.end());
  return neighbors;
}

std::vector<Vertex> TopologicalMapUtils::getAdjacentTransitionNodes(
  const TopologicalMap & map,
  const std::string & room_name)
{
  std::vector<Vertex> transition_nodes;
  
  // Find the vertex for this room by name
  auto room_vertex_opt = getVertexByName(map, room_name);
  if (!room_vertex_opt.has_value()) {
    return transition_nodes;
  }
  
  Vertex room_vertex = *room_vertex_opt;
  const auto & room_node = map[room_vertex];
  
  // Check that node is a room
  if (room_node.type != "room") {
    return transition_nodes;
  }
  
  // Get all adjacent vertices and filter for transition nodes
  auto adj_vertices = boost::adjacent_vertices(room_vertex, map);
  for (auto it = adj_vertices.first; it != adj_vertices.second; ++it) {
    const auto & neighbor_node = map[*it];
    
    // Check if the neighbor is a transition node
    if (neighbor_node.type == "transition") {
      transition_nodes.push_back(*it);
    }
  }
  
  return transition_nodes;
}

bool TopologicalMapUtils::isPointInPolygon(
  const Polygon & polygon,
  double x,
  double y)
{
  // Create a point from the coordinates
  Point2D point(x, y);
  
  // Use Boost.Geometry's within algorithm
  // within returns true if point is inside the polygon
  return boost::geometry::within(point, polygon);
}

std::optional<Vertex> TopologicalMapUtils::getVertexById(
  const TopologicalMap & map,
  const std::string & node_id)
{ 
  auto vertices = boost::vertices(map);
  for (auto it = vertices.first; it != vertices.second; ++it) {
    const auto & node = map[*it];
    if (std::to_string(node.id) == node_id) {
      return *it;
    }
  }
  return std::nullopt;
}

std::optional<Vertex> TopologicalMapUtils::getVertexByName(
  const TopologicalMap & map,
  const std::string & node_name)
{
  auto vertices = boost::vertices(map);
  for (auto it = vertices.first; it != vertices.second; ++it) {
    const auto & node = map[*it];
    if (node.name == node_name) {
      return *it;
    }
  }
  
  return std::nullopt;
}

std::optional<double> TopologicalMapUtils::getMinZ(const TopologicalMap & map)
{
  if (boost::num_vertices(map) == 0) {
    return std::nullopt;
  }
  
  double min_z = std::numeric_limits<double>::max();
  bool found = false;
  
  auto vertices = boost::vertices(map);
  for (auto it = vertices.first; it != vertices.second; ++it) {
    const auto & node = map[*it];
    
    // For floors, use min_z field
    if (node.type == "floor" && node.min_z.has_value()) {
      min_z = std::min(min_z, *node.min_z);
      found = true;
    }
    // For rooms and transitions, use bbox.min_z
    else if ((node.type == "room" || node.type == "transition") && node.bbox.has_value()) {
      min_z = std::min(min_z, node.bbox->min_z);
      found = true;
    }
  }
  
  return found ? std::optional<double>(min_z) : std::nullopt;
}

std::optional<double> TopologicalMapUtils::getMaxZ(const TopologicalMap & map)
{
  if (boost::num_vertices(map) == 0) {
    return std::nullopt;
  }
  
  double max_z = std::numeric_limits<double>::lowest();
  bool found = false;
  
  auto vertices = boost::vertices(map);
  for (auto it = vertices.first; it != vertices.second; ++it) {
    const auto & node = map[*it];
    
    // For floors, use max_z field
    if (node.type == "floor" && node.max_z.has_value()) {
      max_z = std::max(max_z, *node.max_z);
      found = true;
    }
    // For rooms and transitions, use bbox.max_z
    else if ((node.type == "room" || node.type == "transition") && node.bbox.has_value()) {
      max_z = std::max(max_z, node.bbox->max_z);
      found = true;
    }
  }
  
  return found ? std::optional<double>(max_z) : std::nullopt;
}

bool TopologicalMapUtils::isApproximatelyEqual(
  double z1,
  double z2,
  double tolerance)
{
  return std::abs(z1 - z2) <= tolerance;
}

}  // namespace navbim_topomap_server
