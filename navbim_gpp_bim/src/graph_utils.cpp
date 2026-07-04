#include "navbim_gpp_bim/graph_utils.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <set>
#include <unordered_set>

#include <boost/graph/connected_components.hpp>
#include <boost/graph/copy.hpp>
#include <boost/graph/biconnected_components.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/geometry/algorithms/buffer.hpp>
#include <boost/geometry/algorithms/within.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>

namespace navbim_gpp_bim
{

using navbim_util::TopologicalGraph;
using navbim_util::TopologicalMap;
using navbim_util::Vertex;
using navbim_util::Edge;
using navbim_util::NodeProperties;
using navbim_util::EdgeProperties;
using navbim_util::euclideanDistanceBetweenNodes;
using navbim_util::costBetweenNodes;
using navbim_util::isOnSameFloor;

TopologicalGraph insertStartAndGoal(
  const TopologicalMap & topological_map,
  Vertex start_room,
  Vertex end_room,
  const std::array<double, 3> & start_coords,
  const std::array<double, 3> & end_coords,
  double penalize_z_movement)
{
  // Create a copy of the graph using optimized assignment operator
  // This is much faster than boost::copy_graph()
  TopologicalGraph graph = topological_map.getGraph();
  
  // Get dynamic START_ID and GOAL_ID from cached max node ID (O(1) operation!)
  const int START_ID = topological_map.getMaxNodeId() + 1;
  const int GOAL_ID = topological_map.getMaxNodeId() + 2;
  
  // Get next edge ID from cached max edge ID (O(1) operation!)
  int next_edge_id = topological_map.getMaxEdgeId() + 1;
  
  // Get the room IDs (node.id field, not vertex descriptor)
  int start_room_id = graph[start_room].id;
  int end_room_id = graph[end_room].id;
  
  // Add START node
  Vertex start_vertex = boost::add_vertex(graph);
  graph[start_vertex].id = START_ID;
  graph[start_vertex].name = "start";
  graph[start_vertex].type = "transition";
  graph[start_vertex].position = navbim_util::Position(start_coords[0], start_coords[1], start_coords[2]);
  graph[start_vertex].floor = graph[start_room].floor;
  
  // Add GOAL node
  Vertex goal_vertex = boost::add_vertex(graph);
  graph[goal_vertex].id = GOAL_ID;
  graph[goal_vertex].name = "goal";
  graph[goal_vertex].type = "transition";
  graph[goal_vertex].position = navbim_util::Position(end_coords[0], end_coords[1], end_coords[2]);
  graph[goal_vertex].floor = graph[end_room].floor;
  
  // Helper function to add transition edge
  auto add_transition_edge = [&graph, penalize_z_movement](
    Vertex v1, Vertex v2, int edge_id, int room_id) {
    double dist = euclideanDistanceBetweenNodes(graph, v1, v2);
    double cost = costBetweenNodes(graph, v1, v2, penalize_z_movement);
    
    auto [edge, inserted] = boost::add_edge(v1, v2, graph);
    if (inserted) {
      graph[edge].id = edge_id;
      graph[edge].type = "transition";
      graph[edge].room_id = room_id;
      graph[edge].estimated_distance = dist;
      graph[edge].planned_distance = -1.0;
      graph[edge].estimated_cost = cost;
      graph[edge].planned_cost = -1.0;
      graph[edge].path = nullptr;  // Clear the shared_ptr
    }
  };
  
  // Add edge from start to goal if they are in the same room
  if (start_room == end_room) {
    add_transition_edge(start_vertex, goal_vertex, next_edge_id++, start_room_id);
  }
  
  // Add transition edges from start to adjacent doors and stairs
  // Direct iteration without intermediate vector allocation
  for (auto neighbor : boost::make_iterator_range(boost::adjacent_vertices(start_room, graph))) {
    if (graph[neighbor].type != "transition") continue;
    add_transition_edge(start_vertex, neighbor, next_edge_id++, start_room_id);
  }
  
  // Add transition edges from goal to adjacent doors and stairs
  // Direct iteration without intermediate vector allocation
  for (auto neighbor : boost::make_iterator_range(boost::adjacent_vertices(end_room, graph))) {
    if (graph[neighbor].type != "transition") continue;
    add_transition_edge(goal_vertex, neighbor, next_edge_id++, end_room_id);
  }
  
  // Add room edges for start and goal nodes
  auto [start_room_edge, start_inserted] = boost::add_edge(start_vertex, start_room, graph);
  if (start_inserted) {
    graph[start_room_edge].id = next_edge_id++;
    graph[start_room_edge].type = "room";
  }
  
  auto [goal_room_edge, goal_inserted] = boost::add_edge(goal_vertex, end_room, graph);
  if (goal_inserted) {
    graph[goal_room_edge].id = next_edge_id++;
    graph[goal_room_edge].type = "room";
  }
  
  return graph;
}

// Vertex filter predicate for filtered_graph
// Allows hiding vertices without actually removing them
template<typename Graph>
struct EnabledVertexPredicate {
  const std::set<typename boost::graph_traits<Graph>::vertex_descriptor>* disabled;
  
  EnabledVertexPredicate() : disabled(nullptr) {}
  explicit EnabledVertexPredicate(
    const std::set<typename boost::graph_traits<Graph>::vertex_descriptor>* d) 
    : disabled(d) {}
  
  template<typename Vertex>
  bool operator()(const Vertex& v) const {
    if (!disabled) return true;
    return disabled->find(v) == disabled->end();
  }
};

// Filters out edges connected to disabled vertices
template<typename Graph>
struct EnabledEdgePredicate {
  const Graph* graph;
  const std::set<typename boost::graph_traits<Graph>::vertex_descriptor>* disabled;
  
  EnabledEdgePredicate() : graph(nullptr), disabled(nullptr) {}
  EnabledEdgePredicate(
    const Graph* g,
    const std::set<typename boost::graph_traits<Graph>::vertex_descriptor>* d)
    : graph(g), disabled(d) {}
  
  template<typename Edge>
  bool operator()(const Edge& e) const {
    if (!graph || !disabled) return true;
    auto src = boost::source(e, *graph);
    auto tgt = boost::target(e, *graph);
    // Edge is enabled only if both endpoints are enabled
    return (disabled->find(src) == disabled->end()) && 
           (disabled->find(tgt) == disabled->end());
  }
};

std::set<navbim_util::Vertex> pruneGraph(
  const navbim_util::TopologicalGraph & graph,
  const std::set<navbim_util::Vertex> & floor_vertices,
  std::optional<navbim_util::Vertex> start_room,
  std::optional<navbim_util::Vertex> goal_room)
{
  // Find START and GOAL vertices by name
  std::optional<Vertex> start_opt, goal_opt;
  auto vertices = boost::vertices(graph);
  for (auto it = vertices.first; it != vertices.second; ++it) {
    if (graph[*it].name == "start") {
      start_opt = *it;
    } else if (graph[*it].name == "goal") {
      goal_opt = *it;
    }
    if (start_opt.has_value() && goal_opt.has_value()) {
      break;  // Found both, no need to continue
    }
  }
  
  if (!start_opt.has_value() || !goal_opt.has_value()) {
    std::cerr << "Error: START or GOAL node not found in graph!" << std::endl;
    return {};  // Return empty set
  }
  
  Vertex start_vertex = *start_opt;
  Vertex goal_vertex = *goal_opt;
  
  // Set of vertices to be disabled/removed - accumulated throughout pruning phases
  std::set<Vertex> disabled_vertices;
  
  // Early exit check for same-room navigation
  bool same_room_navigation = false;
  if (start_room.has_value() && goal_room.has_value() && 
      start_room.value() == goal_room.value()) {
    // Check if there's a direct edge from start to goal
    auto [edge, found] = boost::edge(start_vertex, goal_vertex, graph);
    if (found) {
      same_room_navigation = true;
    }
  }
  
  // Use cached floor vertices (Phase 1)
  disabled_vertices = floor_vertices;
  
  // Helper to create filtered graph with current disabled set
  auto make_filtered = [&graph, &disabled_vertices]() {
    EnabledVertexPredicate<TopologicalGraph> vpred(&disabled_vertices);
    EnabledEdgePredicate<TopologicalGraph> epred(&graph, &disabled_vertices);
    return boost::make_filtered_graph(graph, epred, vpred);
  };
  
  // Check if start and goal are connected in filtered graph
  {
    auto filtered = make_filtered();
    std::vector<int> component(boost::num_vertices(graph));  // Use original graph size
    boost::connected_components(filtered, &component[0]);
    
    if (component[start_vertex] != component[goal_vertex]) {
      std::cerr << "Start and goal are not connected in the original graph!" << std::endl;
      return disabled_vertices;  // Return current disabled set
    }
  }
  
  // If same-room navigation, skip expensive pruning phases 2-4
  if (!same_room_navigation) {
    // Phase 2: Same-floor optimization
    if (isOnSameFloor(graph[start_vertex].floor, graph[goal_vertex].floor)) {
      std::set<Vertex> other_floor_vertices;
      
      vertices = boost::vertices(graph);
      for (auto it = vertices.first; it != vertices.second; ++it) {
        if (!isOnSameFloor(graph[*it].floor, graph[start_vertex].floor)) {
          other_floor_vertices.insert(*it);
        }
      }
      
      if (!other_floor_vertices.empty()) {
        std::set<Vertex> test_disabled = disabled_vertices;
        test_disabled.insert(other_floor_vertices.begin(), other_floor_vertices.end());
        
        EnabledVertexPredicate<TopologicalGraph> test_vpred(&test_disabled);
        auto test_filtered = boost::make_filtered_graph(graph, boost::keep_all(), test_vpred);
        
        std::vector<int> component(boost::num_vertices(graph));
        boost::connected_components(test_filtered, &component[0]);
        
        std::unordered_map<Vertex, int> vertex_to_component;
        size_t idx = 0;
        auto test_filt_vertices = boost::vertices(test_filtered);
        for (auto it = test_filt_vertices.first; it != test_filt_vertices.second; ++it, ++idx) {
          vertex_to_component[*it] = component[*it];
        }
        
        if (vertex_to_component[start_vertex] == vertex_to_component[goal_vertex]) {
          disabled_vertices.insert(other_floor_vertices.begin(), other_floor_vertices.end());
        }
      }
    }
    
    // Phase 3: Remove unreachable components
    {
      auto filtered = make_filtered();
      std::vector<int> component(boost::num_vertices(graph));
      int num_components = boost::connected_components(filtered, &component[0]);
      
      if (num_components > 1) {
        std::unordered_map<Vertex, int> vertex_to_component;
        auto filt_vertices = boost::vertices(filtered);
        for (auto it = filt_vertices.first; it != filt_vertices.second; ++it) {
          vertex_to_component[*it] = component[*it];
        }
        
        int target_component = vertex_to_component[start_vertex];
        
        filt_vertices = boost::vertices(filtered);
        for (auto it = filt_vertices.first; it != filt_vertices.second; ++it) {
          if (vertex_to_component[*it] != target_component) {
            disabled_vertices.insert(*it);
          }
        }
      }
    }
    
    // Phase 4: Remove dead-end transitions
    {
      auto filtered = make_filtered();
      std::vector<Vertex> transition_nodes;
      auto filt_vertices = boost::vertices(filtered);
      for (auto it = filt_vertices.first; it != filt_vertices.second; ++it) {
        const auto & node = graph[*it];
        if (node.type == "transition" && node.name != "start" && node.name != "goal") {
          transition_nodes.push_back(*it);
        }
      }
      
      for (const auto & v : transition_nodes) {
        if (disabled_vertices.find(v) != disabled_vertices.end()) {
          continue;
        }
        
        int num_room_edges = 0;
        auto adj_range = boost::adjacent_vertices(v, graph);
        for (auto it = adj_range.first; it != adj_range.second; ++it) {
          if (disabled_vertices.find(*it) != disabled_vertices.end()) {
            continue;
          }
          auto [edge, found] = boost::edge(v, *it, graph);
          if (found && graph[edge].type == "room") {
            num_room_edges++;
          }
        }
        if (num_room_edges < 2) {
          disabled_vertices.insert(v);
          continue;
        }

        /* if (graph[v].subtype.has_value() && graph[v].subtype.value() == "door") {
          // Check if all adjacent transition edges are located in the same room
          std::set<int> room_ids;
          auto adj_range = boost::adjacent_vertices(v, graph);
          for (auto it = adj_range.first; it != adj_range.second; ++it) {
            auto [edge, found] = boost::edge(v, *it, graph);
            if (disabled_vertices.find(*it) != disabled_vertices.end()) {
              continue;
            }
            if (found && graph[edge].type == "transition") {
              if (graph[edge].room_id.has_value()) {
                room_ids.insert(graph[edge].room_id.value());
              } else {
                // That should not happen, but if it does, be conservative and keep the node
                std::cerr << "Warning: Transition edge " << graph[edge].id << " from node " << graph[v].id 
                          << " to node " << graph[*it].id << " does not have a room_id!" << std::endl;
                room_ids.insert(-1);
                room_ids.insert(-2);
                break;
              }
            }
          }
          if (room_ids.size() <= 1) {
            disabled_vertices.insert(v);
            continue;
          }
        } */
        
        std::set<Vertex> test_disabled = disabled_vertices;
        test_disabled.insert(v);
        
        EnabledVertexPredicate<TopologicalGraph> test_vpred(&test_disabled);
        auto test_filtered = boost::make_filtered_graph(graph, boost::keep_all(), test_vpred);
        
        std::vector<int> component(boost::num_vertices(graph));
        int num_components = boost::connected_components(test_filtered, &component[0]);
        
        // The removal would disconnect the graph, check if start and goal are still in the same component
        if (num_components > 1) {
          std::unordered_map<Vertex, int> vertex_to_component;
          auto test_filt_vertices = boost::vertices(test_filtered);
          for (auto it = test_filt_vertices.first; it != test_filt_vertices.second; ++it) {
            vertex_to_component[*it] = component[*it];
          }
          // Start and goal are still in the same component, disable the other component(s)
          if (vertex_to_component[start_vertex] == vertex_to_component[goal_vertex]) {
            disabled_vertices.insert(v);
            int target_component = vertex_to_component[start_vertex];
            test_filt_vertices = boost::vertices(test_filtered);
            for (auto it = test_filt_vertices.first; it != test_filt_vertices.second; ++it) {
              if (vertex_to_component[*it] != target_component) {
                disabled_vertices.insert(*it);
              }
            }
          }
        }
      }
    }
  }

  return disabled_vertices;
}

std::optional<navbim_util::Vertex> findVertexById(
  const navbim_util::TopologicalGraph & graph,
  int node_id)
{
  auto vertices = boost::vertices(graph);
  for (auto vit = vertices.first; vit != vertices.second; ++vit) {
    if (graph[*vit].id == node_id) {
      return *vit;
    }
  }
  return std::nullopt;
}

std::optional<Vertex> findFloorByHeight(
  const TopologicalGraph & graph,
  double height)
{
  // Find the floor node where min_z <= height < max_z
  // If multiple floors match, choose the one with smallest (height - min_z)
  
  double smallest_z_diff = std::numeric_limits<double>::infinity();
  std::optional<Vertex> best_floor;
  
  for (auto v : boost::make_iterator_range(boost::vertices(graph))) {
    const auto & node = graph[v];
    
    // Check if this is a floor node with min_z and max_z
    if (node.type == "floor" && node.min_z.has_value() && node.max_z.has_value()) {
      double min_z = *node.min_z;
      double max_z = *node.max_z;
      
      // Check if height is within floor bounds
      if (min_z <= height && height < max_z) {
        double z_diff = height - min_z;
        if (z_diff < smallest_z_diff) {
          smallest_z_diff = z_diff;
          best_floor = v;
        }
      }
    }
  }
  
  return best_floor;
}

bool isPointInPolygon(
  const navbim_util::Polygon & polygon,
  double x,
  double y)
{
  navbim_util::Point2D point(x, y);
  return boost::geometry::within(point, polygon);
}

std::optional<Vertex> findRoomByCoordinates(
  const TopologicalGraph & graph,
  double x,
  double y,
  double z)
{
  // First, find the floor at this height
  auto floor_opt = findFloorByHeight(graph, z);
  if (!floor_opt.has_value()) {
    return std::nullopt;
  }
  
  const auto & floor_node = graph[*floor_opt];
  std::string floor_name = floor_node.name;
  
  // Use buffer approach: start with 0.0, increase to 0.5 in 0.1 increments
  double buffer_radius = 0.0;
  while (buffer_radius <= 0.5) {
    for (auto v : boost::make_iterator_range(boost::vertices(graph))) {
      const auto & node = graph[v];
      
      // Check if this is a room on the correct floor
      if (node.type == "room" && 
          node.floor.has_value() && 
          node.polygon.has_value()) {
        
        // Check if floor_name is in the floor list
        const auto & floor_list = *node.floor;
        bool floor_matches = std::find(floor_list.begin(), floor_list.end(), floor_name) != floor_list.end();
        
        if (floor_matches) {
          // Apply buffer and check if point is inside
          if (buffer_radius > 0.0) {
            // Buffer the polygon - use multi_polygon as output
            boost::geometry::model::multi_polygon<navbim_util::Polygon> buffered_result;
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
            navbim_util::Point2D point(x, y);
            for (const auto & buffered_polygon : buffered_result) {
              if (boost::geometry::within(point, buffered_polygon)) {
                return v;
              }
            }
          } else {
            // No buffer, use original polygon
            if (isPointInPolygon(*node.polygon, x, y)) {
              return v;
            }
          }
        }
      }
    }
    buffer_radius += 0.1;
  }
  
  return std::nullopt;
}

}  // namespace navbim_gpp_bim
