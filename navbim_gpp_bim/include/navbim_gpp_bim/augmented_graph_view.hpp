#ifndef NAVBIM_GPP_BIM__AUGMENTED_GRAPH_VIEW_HPP_
#define NAVBIM_GPP_BIM__AUGMENTED_GRAPH_VIEW_HPP_

#include <array>
#include <optional>
#include <unordered_map>
#include <vector>

#include "navbim_util/topological_map.hpp"

namespace navbim_gpp_bim
{

/**
 * @brief Virtual edge data structure
 * 
 * Represents an edge connecting virtual nodes (start/goal) to the base graph
 * without actually modifying the graph.
 */
struct VirtualEdge {
  int edge_id;
  navbim_util::Vertex source;  // May be virtual or real vertex
  navbim_util::Vertex target;  // May be virtual or real vertex
  navbim_util::EdgeProperties properties;
};

/**
 * @brief Virtual nodes metadata
 * 
 * Lightweight structure containing start/goal node information without
 * modifying the original graph. Total size ~100-200 bytes.
 */
struct VirtualNodes {
  // Virtual vertex descriptors (assigned as num_vertices and num_vertices+1)
  navbim_util::Vertex start_vertex;
  navbim_util::Vertex goal_vertex;
  
  // Virtual node properties
  navbim_util::NodeProperties start_props;
  navbim_util::NodeProperties goal_props;
  
  // Virtual edges (typically ~10 edges: start->transitions, goal->transitions, room edges)
  std::vector<VirtualEdge> edges;
  
  // Quick lookup for virtual vertex properties
  std::unordered_map<navbim_util::Vertex, const navbim_util::NodeProperties*> vertex_to_props;
  
  // Adjacency information for virtual vertices
  std::unordered_map<navbim_util::Vertex, std::vector<VirtualEdge>> out_edges_map;
  std::unordered_map<navbim_util::Vertex, std::vector<VirtualEdge>> in_edges_map;
  
  /**
   * @brief Initialize virtual nodes
   */
  void initialize() {
    vertex_to_props[start_vertex] = &start_props;
    vertex_to_props[goal_vertex] = &goal_props;
    
    // Build adjacency maps
    for (const auto& edge : edges) {
      out_edges_map[edge.source].push_back(edge);
      in_edges_map[edge.target].push_back(edge);
    }
  }
  
  /**
   * @brief Check if a vertex is virtual
   */
  bool isVirtual(navbim_util::Vertex v) const {
    return v == start_vertex || v == goal_vertex;
  }
};

/**
 * @brief Create virtual nodes for start and goal
 * 
 * This replaces insertStartAndGoal but doesn't copy the graph.
 * Instead, it creates lightweight metadata about virtual nodes.
 * 
 * @param topological_map The original topological map (not modified)
 * @param start_room Vertex descriptor of start room in base graph
 * @param end_room Vertex descriptor of end room in base graph
 * @param start_coords 3D coordinates of start position
 * @param end_coords 3D coordinates of goal position
 * @param penalize_z_movement Penalty factor for vertical movement
 * @return VirtualNodes structure (~100-200 bytes)
 */
VirtualNodes createVirtualNodes(
  const navbim_util::TopologicalMap & topological_map,
  navbim_util::Vertex start_room,
  navbim_util::Vertex end_room,
  const std::array<double, 3> & start_coords,
  const std::array<double, 3> & end_coords,
  double penalize_z_movement = 1.0);

/**
 * @brief Result of materializing virtual nodes into a graph
 * 
 * Contains the vertex descriptors that were added, for later cleanup.
 */
struct MaterializedNodes {
  navbim_util::Vertex start_vertex;
  navbim_util::Vertex goal_vertex;
};

/**
 * @brief Materialize virtual nodes into an existing graph (in-place)
 * 
 * Modifies the graph by adding virtual start/goal nodes and their edges.
 * This is much faster than copying the entire graph.
 * 
 * IMPORTANT: Caller must remove the virtual nodes after use by calling
 * boost::clear_vertex() and boost::remove_vertex() on the returned descriptors.
 * Or use ScopedGraphMaterialization for automatic cleanup.
 * 
 * @param graph The graph to modify (will be mutated)
 * @param virtual_nodes The virtual nodes metadata
 * @return MaterializedNodes containing vertex descriptors for cleanup
 */
MaterializedNodes materializeGraphInPlace(
  navbim_util::TopologicalGraph & graph,
  const VirtualNodes & virtual_nodes);

/**
 * @brief RAII wrapper for in-place graph materialization
 * 
 * Automatically cleans up virtual nodes when destroyed, ensuring the graph
 * is always restored to its original state even if exceptions occur.
 */
class ScopedGraphMaterialization {
public:
  ScopedGraphMaterialization(
    navbim_util::TopologicalGraph & graph,
    const VirtualNodes & virtual_nodes);
  
  ~ScopedGraphMaterialization();
  
  // Disable copy and move to ensure single ownership
  ScopedGraphMaterialization(const ScopedGraphMaterialization&) = delete;
  ScopedGraphMaterialization& operator=(const ScopedGraphMaterialization&) = delete;
  ScopedGraphMaterialization(ScopedGraphMaterialization&&) = delete;
  ScopedGraphMaterialization& operator=(ScopedGraphMaterialization&&) = delete;
  
  const MaterializedNodes& nodes() const { return nodes_; }
  
private:
  navbim_util::TopologicalGraph& graph_;
  MaterializedNodes nodes_;
};

}  // namespace navbim_gpp_bim

#endif  // NAVBIM_GPP_BIM__AUGMENTED_GRAPH_VIEW_HPP_
