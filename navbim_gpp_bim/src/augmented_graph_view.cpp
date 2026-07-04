#include "navbim_gpp_bim/augmented_graph_view.hpp"

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <iostream>

namespace navbim_gpp_bim
{

using navbim_util::TopologicalGraph;
using navbim_util::Vertex;
using navbim_util::euclideanDistanceBetweenNodes;
using navbim_util::costBetweenNodes;

VirtualNodes createVirtualNodes(
  const navbim_util::TopologicalMap & topological_map,
  Vertex start_room,
  Vertex end_room,
  const std::array<double, 3> & start_coords,
  const std::array<double, 3> & end_coords,
  double penalize_z_movement)
{
  VirtualNodes virtual_nodes;
  const auto& graph = topological_map.getGraph();
  
  // Assign virtual vertex descriptors
  // They come after all real vertices
  size_t num_vertices = boost::num_vertices(graph);
  virtual_nodes.start_vertex = num_vertices;
  virtual_nodes.goal_vertex = num_vertices + 1;
  
  // Get dynamic IDs from cached max IDs (O(1) operation!)
  const int START_ID = topological_map.getMaxNodeId() + 1;
  const int GOAL_ID = topological_map.getMaxNodeId() + 2;
  int next_edge_id = topological_map.getMaxEdgeId() + 1;
  
  // Get room IDs
  int start_room_id = graph[start_room].id;
  int end_room_id = graph[end_room].id;
  
  // Create START node properties
  virtual_nodes.start_props.id = START_ID;
  virtual_nodes.start_props.name = "start";
  virtual_nodes.start_props.type = "transition";
  virtual_nodes.start_props.position = navbim_util::Position(
    start_coords[0], start_coords[1], start_coords[2]);
  virtual_nodes.start_props.floor = graph[start_room].floor;
  
  // Create GOAL node properties
  virtual_nodes.goal_props.id = GOAL_ID;
  virtual_nodes.goal_props.name = "goal";
  virtual_nodes.goal_props.type = "transition";
  virtual_nodes.goal_props.position = navbim_util::Position(
    end_coords[0], end_coords[1], end_coords[2]);
  virtual_nodes.goal_props.floor = graph[end_room].floor;
  
  // Helper to create edge properties
  auto make_edge_props = [&](int edge_id, int room_id, 
                              const navbim_util::NodeProperties& props1,
                              const navbim_util::NodeProperties& props2) {
    navbim_util::EdgeProperties props;
    props.id = edge_id;
    props.type = "transition";
    props.room_id = room_id;
    
    // Calculate distance and cost
    double dx = props1.position.x - props2.position.x;
    double dy = props1.position.y - props2.position.y;
    double dz = props1.position.z - props2.position.z;
    props.estimated_distance = std::sqrt(dx*dx + dy*dy + dz*dz);
    
    double dz_penalized = dz * penalize_z_movement;
    props.estimated_cost = std::sqrt(dx*dx + dy*dy + dz_penalized*dz_penalized);
    
    props.planned_distance = -1.0;
    props.planned_cost = -1.0;
    
    return props;
  };
  
  // Add edge from start to goal if same room
  if (start_room == end_room) {
    VirtualEdge edge;
    edge.edge_id = next_edge_id++;
    edge.source = virtual_nodes.start_vertex;
    edge.target = virtual_nodes.goal_vertex;
    edge.properties = make_edge_props(
      edge.edge_id, start_room_id,
      virtual_nodes.start_props, virtual_nodes.goal_props);
    virtual_nodes.edges.push_back(edge);
  }
  
  // Add edges from start to adjacent transitions of start_room
  for (auto neighbor : boost::make_iterator_range(boost::adjacent_vertices(start_room, graph))) {
    if (graph[neighbor].type != "transition") continue;
    
    // Find the room_id from the edge in the base graph
    int edge_room_id = start_room_id;  // Default fallback
    for (auto edge_desc : boost::make_iterator_range(boost::out_edges(start_room, graph))) {
      if (boost::target(edge_desc, graph) == neighbor) {
        if (graph[edge_desc].room_id.has_value()) {
          edge_room_id = graph[edge_desc].room_id.value();
        }
        break;
      }
    }
    
    VirtualEdge edge;
    edge.edge_id = next_edge_id++;
    edge.source = virtual_nodes.start_vertex;
    edge.target = neighbor;  // Real vertex
    edge.properties = make_edge_props(
      edge.edge_id, edge_room_id,
      virtual_nodes.start_props, graph[neighbor]);
    virtual_nodes.edges.push_back(edge);
  }
  
  // Add edges from goal to adjacent transitions of end_room
  for (auto neighbor : boost::make_iterator_range(boost::adjacent_vertices(end_room, graph))) {
    if (graph[neighbor].type != "transition") continue;
    
    // Find the room_id from the edge in the base graph
    int edge_room_id = end_room_id;  // Default fallback
    for (auto edge_desc : boost::make_iterator_range(boost::out_edges(end_room, graph))) {
      if (boost::target(edge_desc, graph) == neighbor) {
        if (graph[edge_desc].room_id.has_value()) {
          edge_room_id = graph[edge_desc].room_id.value();
        }
        break;
      }
    }
    
    VirtualEdge edge;
    edge.edge_id = next_edge_id++;
    edge.source = virtual_nodes.goal_vertex;
    edge.target = neighbor;  // Real vertex
    edge.properties = make_edge_props(
      edge.edge_id, edge_room_id,
      virtual_nodes.goal_props, graph[neighbor]);
    virtual_nodes.edges.push_back(edge);
  }
  
  // Add room edges
  {
    VirtualEdge edge;
    edge.edge_id = next_edge_id++;
    edge.source = virtual_nodes.start_vertex;
    edge.target = start_room;
    edge.properties.id = edge.edge_id;
    edge.properties.type = "room";
    virtual_nodes.edges.push_back(edge);
  }
  
  {
    VirtualEdge edge;
    edge.edge_id = next_edge_id++;
    edge.source = virtual_nodes.goal_vertex;
    edge.target = end_room;
    edge.properties.id = edge.edge_id;
    edge.properties.type = "room";
    virtual_nodes.edges.push_back(edge);
  }
  
  // Initialize lookup maps
  virtual_nodes.initialize();
  
  return virtual_nodes;
}

ScopedGraphMaterialization::ScopedGraphMaterialization(
  navbim_util::TopologicalGraph & graph,
  const VirtualNodes & virtual_nodes)
: graph_(graph)
{
  nodes_ = materializeGraphInPlace(graph_, virtual_nodes);
}

ScopedGraphMaterialization::~ScopedGraphMaterialization()
{
  // CRITICAL: Always cleanup, even if exception occurs
  // Wrap in try-catch to prevent exceptions from escaping destructor
  try {
    // IMPORTANT: The graph uses vecS for vertices, so remove_vertex invalidates
    // all descriptors with index >= removed vertex. We must remove in REVERSE order
    // of creation to avoid using invalidated descriptors.
    
    // GOAL was created last, so remove it first
    boost::clear_vertex(nodes_.goal_vertex, graph_);
    boost::remove_vertex(nodes_.goal_vertex, graph_);
    
    // Then remove START (created first)
    boost::clear_vertex(nodes_.start_vertex, graph_);
    boost::remove_vertex(nodes_.start_vertex, graph_);
  } catch (const std::exception& e) {
    // Log error but DO NOT THROW from destructor
    // This would cause std::terminate() and crash the process
    std::cerr << "ERROR: Failed to cleanup START/GOAL vertices in destructor: " 
              << e.what() << std::endl;
  } catch (...) {
    std::cerr << "ERROR: Unknown exception during START/GOAL cleanup in destructor" 
              << std::endl;
  }
}

MaterializedNodes materializeGraphInPlace(
  navbim_util::TopologicalGraph & graph,
  const VirtualNodes & virtual_nodes)
{
  // SAFETY CHECK: Verify no existing START or GOAL nodes in graph
  // This catches cleanup failures from previous requests
  auto vertices = boost::vertices(graph);
  for (auto it = vertices.first; it != vertices.second; ++it) {
    const auto& node = graph[*it];
    if (node.name == "start" || node.name == "goal") {
      std::cerr << "WARNING: Found existing '" << node.name 
                << "' node (id=" << node.id << ") in graph before materialization! "
                << "This indicates a cleanup failure from a previous request. "
                << "Attempting to remove it..." << std::endl;
      
      // Try to remove the stale node
      try {
        boost::clear_vertex(*it, graph);
        boost::remove_vertex(*it, graph);
        std::cerr << "Successfully removed stale '" << node.name << "' node." << std::endl;
        
        // Restart iteration since we modified the graph
        // This is safe because we break out after first removal
        vertices = boost::vertices(graph);
        it = vertices.first;
        if (it == vertices.second) break;  // Graph now empty
        continue;
      } catch (const std::exception& e) {
        std::cerr << "ERROR: Failed to remove stale '" << node.name 
                  << "' node: " << e.what() << std::endl;
        throw;  // Re-throw to caller
      }
    }
  }
  
  // Add virtual START node
  Vertex start_v = boost::add_vertex(graph);
  graph[start_v] = virtual_nodes.start_props;
  
  // Add virtual GOAL node
  Vertex goal_v = boost::add_vertex(graph);
  graph[goal_v] = virtual_nodes.goal_props;
  
  // Map virtual vertex descriptors to materialized ones
  std::unordered_map<Vertex, Vertex> virtual_to_real;
  virtual_to_real[virtual_nodes.start_vertex] = start_v;
  virtual_to_real[virtual_nodes.goal_vertex] = goal_v;
  
  // Add all virtual edges
  for (const auto& vedge : virtual_nodes.edges) {
    Vertex src = (vedge.source == virtual_nodes.start_vertex) ? start_v :
                 (vedge.source == virtual_nodes.goal_vertex) ? goal_v :
                 vedge.source;  // Real vertex, use as-is
    
    Vertex tgt = (vedge.target == virtual_nodes.start_vertex) ? start_v :
                 (vedge.target == virtual_nodes.goal_vertex) ? goal_v :
                 vedge.target;  // Real vertex, use as-is
    
    auto [edge, inserted] = boost::add_edge(src, tgt, graph);
    if (inserted) {
      graph[edge] = vedge.properties;
    }
  }
  
  return {start_v, goal_v};
}

}  // namespace navbim_gpp_bim
