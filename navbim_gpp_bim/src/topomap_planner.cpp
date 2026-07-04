// Copyright (c) 2025 NavBIM Contributors
// Licensed under the Apache License, Version 2.0

#include "navbim_gpp_bim/topomap_planner.hpp"
#include "navbim_util/topological_map.hpp"

#include <boost/heap/fibonacci_heap.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/copy.hpp>
#include <boost/graph/filtered_graph.hpp>

#include <rclcpp/logging.hpp>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <cmath>
#include <limits>
#include <algorithm>

namespace navbim_gpp_bim
{

// Vertex filter predicate for filtered_graph
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

namespace
{

/**
 * @brief Edge in open set with priority
 */
struct OpenSetEdge
{
  navbim_util::Vertex u;  ///< First vertex
  navbim_util::Vertex v;  ///< Second vertex
  double f_score;  ///< Priority (f-score)

  bool operator<(const OpenSetEdge& other) const
  {
    // Note: Boost heap is a max-heap by default, so we invert comparison
    return f_score > other.f_score;
  }
};

/**
 * @brief Hash function for edge (unordered pair of vertices)
 */
struct EdgeHash
{
  std::size_t operator()(const std::pair<navbim_util::Vertex, navbim_util::Vertex>& edge) const
  {
    // Combine hashes (order-independent)
    auto h1 = std::hash<size_t>{}(edge.first);
    auto h2 = std::hash<size_t>{}(edge.second);
    return h1 ^ (h2 << 1);
  }
};

/**
 * @brief Get floor name from edge in original graph via room_id
 */
std::string getFloorOfEdge(
  const navbim_util::TopologicalGraph& graph,
  navbim_util::Vertex u,
  navbim_util::Vertex v)
{
  // Get edge to find room_id
  auto edge = boost::edge(u, v, graph);
  if (!edge.second) {
    return "";
  }
  
  const auto& edge_props = graph[edge.first];
  
  // Get room_id from edge
  if (!edge_props.room_id.has_value()) {
    // No room_id means this is likely a stair/elevator edge
    // Get floor from source vertex instead
    if (graph[u].floor.has_value() && !graph[u].floor.value().empty()) {
      return graph[u].floor.value()[0];
    }
    return "";
  }
  
  int room_id = edge_props.room_id.value();
  
  // Find the room node by ID
  auto vertices = boost::vertices(graph);
  for (auto vit = vertices.first; vit != vertices.second; ++vit) {
    if (graph[*vit].id == room_id) {
      // Found the room node, get its floor
      if (graph[*vit].floor.has_value() && !graph[*vit].floor.value().empty()) {
        return graph[*vit].floor.value()[0];
      }
    }
  }
  
  return "";
}

/**
 * @brief Get room name from edge in original graph via room_id
 */
std::string getRoomNameOfEdge(
  const navbim_util::TopologicalGraph& graph,
  navbim_util::Vertex u,
  navbim_util::Vertex v)
{
  // Get edge to find room_id
  auto edge = boost::edge(u, v, graph);
  if (!edge.second) {
    return "";
  }
  
  const auto& edge_props = graph[edge.first];
  
  // Get room_id from edge
  if (!edge_props.room_id.has_value()) {
    // No room_id means this is likely a stair/elevator edge
    // Use the vertex name as fallback
    return graph[u].name;
  }
  
  int room_id = edge_props.room_id.value();
  
  // Find the room node by ID
  auto vertices = boost::vertices(graph);
  for (auto vit = vertices.first; vit != vertices.second; ++vit) {
    if (graph[*vit].id == room_id) {
      // Found the room node, return its name
      return graph[*vit].name;
    }
  }
  
  return "";
}

}  // anonymous namespace

TopomapPlannerResult planTopomapPath(
  navbim_util::TopologicalGraph& graph,
  navbim_util::Vertex start_vertex,
  navbim_util::Vertex goal_vertex,
  SecondLevelPlannerCallback second_level_planner,
  const std::set<navbim_util::Vertex>& disabled_vertices,
  std::shared_ptr<nav2_util::ServiceClient<navbim_msgs::srv::UpdateEdgeData, nav2_util::LifecycleNode::SharedPtr>> update_edge_client,
  double penalize_z_movement,
  bool reuse_paths,
  const rclcpp::Logger& logger,
  TopomapPlannerTimings* timings)
{
  using namespace std::chrono;
  auto astar_start = high_resolution_clock::now();

  // Initialize timings if provided
  if (timings) {
    timings->topomap_astar_time = 0.0;
    timings->second_level_total_time = 0.0;
    timings->second_level_count = 0;
  }

  // Create transition graph using filtered graph (no copy, no physical removal!)
  // Use the disabled_vertices set to filter out unwanted nodes
  EnabledVertexPredicate<navbim_util::TopologicalGraph> vpred(&disabled_vertices);
  EnabledEdgePredicate<navbim_util::TopologicalGraph> epred(&graph, &disabled_vertices);
  auto transition_graph = boost::make_filtered_graph(graph, epred, vpred);

  // Log filtering statistics
  RCLCPP_DEBUG(logger, "Planning with %zu disabled vertices (filtered from %zu total)",
               disabled_vertices.size(), boost::num_vertices(graph));

  // Verify start and goal are not disabled
  if (disabled_vertices.find(start_vertex) != disabled_vertices.end()) {
    RCLCPP_ERROR(logger, "Start vertex is disabled!");
    return {{}, {}, std::numeric_limits<double>::infinity(),
            std::numeric_limits<double>::infinity(), false};
  }
  if (disabled_vertices.find(goal_vertex) != disabled_vertices.end()) {
    RCLCPP_ERROR(logger, "Goal vertex is disabled!");
    return {{}, {}, std::numeric_limits<double>::infinity(),
            std::numeric_limits<double>::infinity(), false};
  }

  // Start and goal vertices are the same in filtered graph (no remapping needed)
  navbim_util::Vertex tr_start = start_vertex;
  navbim_util::Vertex tr_goal = goal_vertex;

  // Initialize g_scores
  std::unordered_map<navbim_util::Vertex, double> g_score;
  auto all_vertices = boost::vertices(transition_graph);
  for (auto vit = all_vertices.first; vit != all_vertices.second; ++vit) {
    g_score[*vit] = std::numeric_limits<double>::infinity();
  }
  g_score[tr_start] = 0.0;

  // Came_from map to reconstruct path
  std::unordered_map<navbim_util::Vertex, navbim_util::Vertex> came_from;

  // Closed set for visited edges
  std::unordered_set<std::pair<navbim_util::Vertex, navbim_util::Vertex>,
                     EdgeHash> closed_set;

  // Open set with Fibonacci heap
  using Heap = boost::heap::fibonacci_heap<OpenSetEdge>;
  using HeapHandle = typename Heap::handle_type;
  Heap open_heap;

  // Map edge to heap handle for decrease-key operations
  std::unordered_map<std::pair<navbim_util::Vertex, navbim_util::Vertex>,
                     HeapHandle, EdgeHash> edge_to_handle;

  // Heuristic functions
  auto h1_score = [&](navbim_util::Vertex u, navbim_util::Vertex v) -> double {
    // Cost of transition edge (use estimated or planned cost)
    auto edge = boost::edge(u, v, transition_graph);
    if (!edge.second) {
      return std::numeric_limits<double>::infinity();
    }
    const auto& edge_data = transition_graph[edge.first];
    if (edge_data.planned_cost >= 0.0) {
      return edge_data.planned_cost;
    }
    return edge_data.estimated_cost;
  };

  auto h2_score = [&](navbim_util::Vertex v) -> double {
    // Heuristic from node to goal (use original graph for cost calculation)
    return navbim_util::costBetweenNodes(graph, v, tr_goal,
                                         penalize_z_movement);
  };

  // Initialize open set with neighbors of start
  auto start_edges = boost::out_edges(tr_start, transition_graph);
  for (auto eit = start_edges.first; eit != start_edges.second; ++eit) {
    navbim_util::Vertex neighbor = boost::target(*eit, transition_graph);
    double f_score = h1_score(tr_start, neighbor) + h2_score(neighbor);
    
    auto u = std::min(tr_start, neighbor);
    auto v = std::max(tr_start, neighbor);
    auto handle = open_heap.push({u, v, f_score});
    edge_to_handle[{u, v}] = handle;
  }

  // Track best score to goal
  double best_score = std::numeric_limits<double>::infinity();

  // Main A* loop
  while (!open_heap.empty()) {
    // Pop edge with lowest f-score
    OpenSetEdge current_edge = open_heap.top();
    open_heap.pop();

    navbim_util::Vertex u = current_edge.u;
    navbim_util::Vertex v = current_edge.v;

    // Remove from handle map
    edge_to_handle.erase({u, v});

    // Get positions for second-level planning
    std::array<double, 3> start_coords = {
      transition_graph[u].position.x,
      transition_graph[u].position.y,
      transition_graph[u].position.z
    };
    std::array<double, 3> end_coords = {
      transition_graph[v].position.x,
      transition_graph[v].position.y,
      transition_graph[v].position.z
    };

    // Get edge data
    auto edge_descriptor = boost::edge(u, v, transition_graph);
    if (!edge_descriptor.second) {
      RCLCPP_ERROR(logger, "Edge not found in graph");
      continue;
    }
    auto& edge_data = transition_graph[edge_descriptor.first];
    // Get floor and room from original graph using node IDs
    // Find vertices in original graph that match transition graph nodes
    int u_id = transition_graph[u].id;
    int v_id = transition_graph[v].id;
    navbim_util::Vertex orig_u = 0;
    navbim_util::Vertex orig_v = 0;
    bool found_u = false, found_v = false;
    auto orig_vertices = boost::vertices(graph);
    for (auto vit = orig_vertices.first; vit != orig_vertices.second; ++vit) {
      if (graph[*vit].id == u_id) {
        orig_u = *vit;
        found_u = true;
      }
      if (graph[*vit].id == v_id) {
        orig_v = *vit;
        found_v = true;
      }
      if (found_u && found_v) break;
    }
    std::string floor_name, room_name;
    if (found_u && found_v) {
      floor_name = getFloorOfEdge(graph, orig_u, orig_v);
      room_name = getRoomNameOfEdge(graph, orig_u, orig_v);
    } else {
      RCLCPP_WARN(logger, "Could not find original vertices for edge (%d, %d)",
                  transition_graph[u].id, transition_graph[v].id);
    }

    // Plan path through this edge
    std::optional<std::vector<std::array<double, 3>>> path_opt;
    double path_distance = -1.0;
    double path_cost = -1.0;

    // Check if we should use cached path
    // Always reuse stairs/ramps (expensive and unchanging)
    // For other edges, only reuse if reuse_paths is enabled
    bool is_preplanned = (edge_data.subtype.has_value() && 
                         (edge_data.subtype.value() == "stair" || 
                          edge_data.subtype.value() == "ramp"));
    bool should_reuse = edge_data.path && edge_data.path->size() >= 2 && (reuse_paths || is_preplanned);
    
    if (should_reuse) {
      path_opt = *edge_data.path;  // Dereference shared_ptr to get vector
      path_distance = edge_data.planned_distance;
      path_cost = edge_data.planned_cost;
    } else {
      // Warn if room_name is invalid for edges that need planning
      if ((room_name.empty() || room_name == "/") && !is_preplanned) {
        RCLCPP_WARN(logger, 
          "Edge (%d, %d) needs planning but has invalid room_name '%s'. Edge type: '%s', subtype: '%s'",
          u_id, v_id, room_name.c_str(), edge_data.type.c_str(),
          edge_data.subtype.has_value() ? edge_data.subtype.value().c_str() : "none");
      }
      
      // Call second-level planner
      auto second_start = high_resolution_clock::now();
      auto result_tuple = second_level_planner(floor_name, room_name, start_coords, end_coords);
      auto second_end = high_resolution_clock::now();
      
      // Unpack the tuple
      path_opt = std::get<0>(result_tuple);
      path_distance = std::get<1>(result_tuple);
      path_cost = std::get<2>(result_tuple);

      if (timings) {
        timings->second_level_total_time +=
            duration_cast<duration<double>>(second_end - second_start).count();
        timings->second_level_count++;
      }
      
      // Always update topomap server with newly computed paths (only if successful)
      // Skip edges connected to start or goal (they are query-specific)
      if (path_opt.has_value() && update_edge_client && 
          u != tr_start && u != tr_goal && v != tr_start && v != tr_goal) {
        auto request = std::make_shared<navbim_msgs::srv::UpdateEdgeData::Request>();
        request->source_id = transition_graph[u].id;
        request->target_id = transition_graph[v].id;
        request->planned_distance = path_distance;
        request->planned_cost = path_cost;
        request->path.header.frame_id = "ifc";
        for (const auto& wp : path_opt.value()) {
          geometry_msgs::msg::PoseStamped ps;
          ps.pose.position.x = wp[0]; ps.pose.position.y = wp[1]; ps.pose.position.z = wp[2];
          ps.pose.orientation.w = 1.0;
          request->path.poses.push_back(ps);
        }
        update_edge_client->invoke(request);
      }
    }

    // Check if planning failed
    if (!path_opt.has_value()) {
      RCLCPP_WARN(
        logger,
        "Second-level planning failed for edge (%d, %d) in room '%s' in floor '%s'. "
        "Start coords: (%.2f, %.2f, %.2f), Goal coords: (%.2f, %.2f, %.2f)",
        transition_graph[u].id,
        transition_graph[v].id,
        room_name.c_str(),
        floor_name.c_str(),
        start_coords[0], start_coords[1], start_coords[2],
        end_coords[0], end_coords[1], end_coords[2]);
      // Infinite cost indicates failure for this edge
      edge_data.planned_distance = std::numeric_limits<double>::infinity();
      edge_data.planned_cost = std::numeric_limits<double>::infinity();
      edge_data.path = nullptr;  // Clear the shared_ptr
    } else {
      // Store in edge data (using shared_ptr to enable efficient reuse)
      edge_data.planned_distance = path_distance;
      edge_data.planned_cost = path_cost;
      edge_data.path = std::make_shared<std::vector<std::array<double, 3>>>(path_opt.value());
    }
    // Mark edge as explored
    closed_set.insert({u, v});

    // Determine predecessor and successor based on g_scores
    navbim_util::Vertex pre, suc;
    if (g_score[u] <= g_score[v]) {
      pre = u;
      suc = v;
    } else {
      pre = v;
      suc = u;
    }

    // Update g_score of successor
    double tentative_g_score = g_score[pre] + edge_data.planned_cost;

    if (tentative_g_score < g_score[suc]) {
      // Better path found
      came_from[suc] = pre;
      double diff = g_score[suc] - tentative_g_score;
      g_score[suc] = tentative_g_score;

      // Recursively update g_scores of successors
      std::unordered_set<navbim_util::Vertex> updated_nodes;
      updated_nodes.insert(suc);

      std::function<void(navbim_util::Vertex, double)> update_g_score_recursively;
      update_g_score_recursively = [&](navbim_util::Vertex node, double delta) {
        // Find successors (nodes that have 'node' as predecessor)
        for (const auto& [successor, predecessor] : came_from) {
          if (predecessor == node) {
            g_score[successor] -= delta;
            updated_nodes.insert(successor);
            update_g_score_recursively(successor, delta);
          }
        }
      };

      update_g_score_recursively(suc, diff);

      // Update f-scores in open set for edges adjacent to updated nodes
      std::unordered_set<std::pair<navbim_util::Vertex, navbim_util::Vertex>,
                         EdgeHash> updated_edges;

      for (auto node : updated_nodes) {
        auto node_edges = boost::out_edges(node, transition_graph);
        for (auto eit = node_edges.first; eit != node_edges.second; ++eit) {
          navbim_util::Vertex neighbor = boost::target(*eit, transition_graph);

          // Skip predecessor
          if (came_from.count(node) && came_from[node] == neighbor) {
            continue;
          }

          auto edge_u = std::min(node, neighbor);
          auto edge_v = std::max(node, neighbor);
          auto edge_pair = std::make_pair(edge_u, edge_v);

          if (updated_edges.count(edge_pair)) {
            continue;
          }
          updated_edges.insert(edge_pair);

          // Update if in open set
          if (edge_to_handle.count(edge_pair)) {
            // Determine pre/suc for this edge
            navbim_util::Vertex edge_pre, edge_suc;
            if (g_score[edge_u] <= g_score[edge_v]) {
              edge_pre = edge_u;
              edge_suc = edge_v;
            } else {
              edge_pre = edge_v;
              edge_suc = edge_u;
            }

            double new_f_score = g_score[edge_pre] +
                                 h1_score(edge_pre, edge_suc) +
                                 h2_score(edge_suc);

            auto handle = edge_to_handle[edge_pair];
            if (new_f_score < (*handle).f_score) {
              // Update priority (decrease key)
              open_heap.update(handle, {edge_u, edge_v, new_f_score});
            }
          }
        }
      }
    }

    // Update best score if goal reached
    if (suc == tr_goal && g_score[suc] < best_score) {
      best_score = g_score[suc];
    }

    // Add neighbors to open set
    auto suc_edges = boost::out_edges(suc, transition_graph);
    for (auto eit = suc_edges.first; eit != suc_edges.second; ++eit) {
      navbim_util::Vertex neighbor = boost::target(*eit, transition_graph);

      // Skip predecessor
      if (neighbor == pre) {
        continue;
      }

      auto edge_u = std::min(suc, neighbor);
      auto edge_v = std::max(suc, neighbor);
      auto edge_pair = std::make_pair(edge_u, edge_v);

      // Skip if already closed
      if (closed_set.count(edge_pair)) {
        continue;
      }

      // Add to open set if not present
      if (!edge_to_handle.count(edge_pair)) {
        double f_score = g_score[suc] +
                         h1_score(suc, neighbor) +
                         h2_score(neighbor);
        auto handle = open_heap.push({edge_u, edge_v, f_score});
        edge_to_handle[edge_pair] = handle;
      }
    }

    // Early termination check
    if (!open_heap.empty()) {
      double min_f_score = open_heap.top().f_score;
      if (min_f_score >= best_score) {
        // All remaining paths are worse than best path
        break;
      }
    }
  }

  auto astar_end = high_resolution_clock::now();
  if (timings) {
    // Calculate pure A* time (excluding second-level planning)
    double total_time = duration_cast<duration<double>>(astar_end - astar_start).count();
    timings->topomap_astar_time = total_time - timings->second_level_total_time;
  }

  // Reconstruct path if goal was reached
  if (best_score < std::numeric_limits<double>::infinity()) {
    std::vector<navbim_util::Vertex> coarse_path;
    std::vector<std::array<double, 3>> detailed_path;

    // Reconstruct coarse path
    navbim_util::Vertex current = tr_goal;
    coarse_path.push_back(current);
    while (came_from.count(current)) {
      current = came_from[current];
      coarse_path.push_back(current);
    }
    std::reverse(coarse_path.begin(), coarse_path.end());

    // Build detailed path
    double total_distance = 0.0;
    double total_cost = 0.0;

    for (size_t i = 0; i < coarse_path.size() - 1; ++i) {
      navbim_util::Vertex u = coarse_path[i];
      navbim_util::Vertex v = coarse_path[i + 1];

      auto edge = boost::edge(u, v, transition_graph);
      if (!edge.second) {
        RCLCPP_ERROR(logger, "Edge not found during path reconstruction");
        continue;
      }

      const auto& edge_data = transition_graph[edge.first];
      
      // Skip if path is not available
      if (!edge_data.path) {
        RCLCPP_WARN(logger, "Edge path is null during reconstruction");
        continue;
      }
      
      const auto& segment_path = *edge_data.path;  // Dereference shared_ptr

      // Check direction and append path
      if (transition_graph[u].id < transition_graph[v].id) {
        // Forward direction
        for (const auto& coord : segment_path) {
          detailed_path.push_back(coord);
        }
      } else {
        // Reverse direction
        for (auto rit = segment_path.rbegin(); rit != segment_path.rend(); ++rit) {
          detailed_path.push_back(*rit);
        }
      }

      total_distance += edge_data.planned_distance;
      total_cost += edge_data.planned_cost;
    }

    return {coarse_path, detailed_path, total_distance, total_cost, true};
  }

  // No path found
  RCLCPP_INFO(logger, "No path found between start and goal");
  return {{}, {}, std::numeric_limits<double>::infinity(),
          std::numeric_limits<double>::infinity(), false};
}

}  // namespace navbim_gpp_bim
