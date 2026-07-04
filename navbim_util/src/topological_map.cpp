#include "navbim_util/topological_map.hpp"

#include <boost/graph/graph_utility.hpp>
#include <cmath>

namespace navbim_util
{

TopologicalMap::TopologicalMap()
: total_floors_(0),
  cached_max_node_id_(-1),
  cached_max_edge_id_(-1)
{
}

bool TopologicalMap::loadFromMessage(const navbim_msgs::msg::Topomap & topomap_msg)
{
  try {
    // Clear existing graph
    clear();
    
    // Store metadata
    source_file_ = topomap_msg.source_file;
    building_name_ = topomap_msg.building_name;
    total_floors_ = topomap_msg.total_floors;
    floor_names_ = topomap_msg.floor_names;
    
    // Map from node ID string to vertex descriptor
    std::unordered_map<std::string, Vertex> id_map;
    
    // Add all nodes
    for (const auto & node_msg : topomap_msg.nodes.nodes) {
      // Create vertex with properties
      Vertex v = boost::add_vertex(graph_);
      auto & props = graph_[v];
      
      // Parse ID (stored as string in message, int in graph)
      props.id = std::stoi(node_msg.id);
      props.name = node_msg.name;
      props.type = node_msg.type;
      
      // Optional properties
      if (!node_msg.ifc_guid.empty()) {
        props.ifc_guid = node_msg.ifc_guid;
      }
      
      if (!node_msg.subtype.empty()) {
        props.subtype = node_msg.subtype;
      }
      
      // Position
      props.position.x = node_msg.position.x;
      props.position.y = node_msg.position.y;
      props.position.z = node_msg.position.z;
      
      // Floor-specific properties
      if (node_msg.type == "floor") {
        props.min_z = node_msg.min_z;
        props.max_z = node_msg.max_z;
      }
      
      // Floor association
      if (!node_msg.floor.empty()) {
        props.floor = node_msg.floor;
      }
      
      // Bounding box (for rooms and transitions)
      if (node_msg.type == "room" || node_msg.type == "transition") {
        // Check if bbox has valid data (non-zero)
        if (node_msg.bbox.min.x != 0.0 || node_msg.bbox.max.x != 0.0) {
          BoundingBox bbox;
          bbox.min_x = node_msg.bbox.min.x;
          bbox.min_y = node_msg.bbox.min.y;
          bbox.min_z = node_msg.bbox.min.z;
          bbox.max_x = node_msg.bbox.max.x;
          bbox.max_y = node_msg.bbox.max.y;
          bbox.max_z = node_msg.bbox.max.z;
          props.bbox = bbox;
        }
      }
      
      // Polygon (for rooms) - convert from ROS message to Boost.Geometry
      if (node_msg.type == "room" && !node_msg.polygon.outer.points.empty()) {
        props.polygon = convertPolygonFromMessage(node_msg.polygon);
      }
      
      // Store in lookup map
      id_map[node_msg.id] = v;
    }
    
    // Add all edges
    for (const auto & edge_msg : topomap_msg.edges.edges) {
      // Find source and target vertices
      auto source_it = id_map.find(edge_msg.source);
      auto target_it = id_map.find(edge_msg.target);
      
      if (source_it == id_map.end() || target_it == id_map.end()) {
        // Skip edge if vertices not found
        continue;
      }
      
      // Add edge
      auto [edge, added] = boost::add_edge(source_it->second, target_it->second, graph_);
      
      if (added) {
        auto & edge_props = graph_[edge];
        
        // Parse ID
        edge_props.id = std::stoi(edge_msg.id);
        edge_props.type = edge_msg.type;
        
        // Subtype (optional)
        if (!edge_msg.subtype.empty()) {
          edge_props.subtype = edge_msg.subtype;
        }
        
        // Room association
        if (!edge_msg.room_id.empty()) {
          edge_props.room_id = std::stoi(edge_msg.room_id);
        }
        
        // Distance and cost metrics
        edge_props.estimated_distance = edge_msg.estimated_distance;
        edge_props.planned_distance = edge_msg.planned_distance;
        edge_props.estimated_cost = edge_msg.estimated_cost;
        edge_props.planned_cost = edge_msg.planned_cost;
        
        // Convert path waypoints
        if (!edge_msg.path.poses.empty()) {
          // Initialize shared_ptr if needed and populate waypoints
          if (!edge_props.path) {
            edge_props.path = std::make_shared<std::vector<std::array<double, 3>>>();
          }
          edge_props.path->reserve(edge_msg.path.poses.size());
          for (const auto & pose : edge_msg.path.poses) {
            std::array<double, 3> waypoint = {
              pose.pose.position.x,
              pose.pose.position.y,
              pose.pose.position.z
            };
            edge_props.path->push_back(waypoint);
          }
        }
      }
    }
    
    // Build lookup maps for efficient queries
    buildLookupMaps();
    
    // Update cached max IDs for efficient ID generation
    updateMaxIds();
    
    return true;
    
  } catch (const std::exception & e) {
    // Clear on error
    clear();
    return false;
  }
}

Polygon TopologicalMap::convertPolygonFromMessage(
  const navbim_msgs::msg::Polygon & polygon_msg) const
{
  Polygon poly;
  
  // Convert outer boundary
  auto & outer = poly.outer();
  for (const auto & pt : polygon_msg.outer.points) {
    boost::geometry::append(outer, Point2D(pt.x, pt.y));
  }
  
  // Ensure outer ring is closed (first point == last point)
  if (!outer.empty() && 
      (outer.front().x() != outer.back().x() || outer.front().y() != outer.back().y())) {
    boost::geometry::append(outer, outer.front());
  }
  
  // Convert holes (inner rings)
  auto & inners = poly.inners();
  for (const auto & hole_msg : polygon_msg.holes) {
    typename Polygon::ring_type inner;
    for (const auto & pt : hole_msg.points) {
      boost::geometry::append(inner, Point2D(pt.x, pt.y));
    }
    
    // Ensure inner ring is closed
    if (!inner.empty() && 
        (inner.front().x() != inner.back().x() || inner.front().y() != inner.back().y())) {
      boost::geometry::append(inner, inner.front());
    }
    
    inners.push_back(inner);
  }
  
  return poly;
}

void TopologicalMap::buildLookupMaps()
{
  id_to_vertex_.clear();
  name_to_vertex_.clear();
  
  auto vertices = boost::vertices(graph_);
  for (auto vit = vertices.first; vit != vertices.second; ++vit) {
    const auto & node = graph_[*vit];
    
    // ID lookup
    id_to_vertex_[std::to_string(node.id)] = *vit;
    
    // Name lookup (if name is not empty)
    if (!node.name.empty()) {
      name_to_vertex_[node.name] = *vit;
    }
  }
}

void TopologicalMap::updateMaxIds()
{
  cached_max_node_id_ = -1;
  cached_max_edge_id_ = -1;
  
  // Clear and rebuild floor vertices cache
  floor_vertices_.clear();
  
  // Find max node ID and build floor vertices cache
  auto vertices = boost::vertices(graph_);
  for (auto vit = vertices.first; vit != vertices.second; ++vit) {
    const auto & node = graph_[*vit];
    if (node.id > cached_max_node_id_) {
      cached_max_node_id_ = node.id;
    }
    
    // Cache floor vertices
    if (node.type == "floor") {
      floor_vertices_.insert(*vit);
    }
  }
  
  // Find max edge ID
  auto edges = boost::edges(graph_);
  for (auto eit = edges.first; eit != edges.second; ++eit) {
    const auto & edge = graph_[*eit];
    if (edge.id > cached_max_edge_id_) {
      cached_max_edge_id_ = edge.id;
    }
  }
}

std::optional<Vertex> TopologicalMap::findVertexById(const std::string & node_id) const
{
  auto it = id_to_vertex_.find(node_id);
  if (it != id_to_vertex_.end()) {
    return it->second;
  }
  return std::nullopt;
}

std::optional<Vertex> TopologicalMap::findVertexByName(const std::string & node_name) const
{
  auto it = name_to_vertex_.find(node_name);
  if (it != name_to_vertex_.end()) {
    return it->second;
  }
  return std::nullopt;
}

std::vector<Vertex> TopologicalMap::getVerticesByType(const std::string & type) const
{
  std::vector<Vertex> result;
  
  auto vertices = boost::vertices(graph_);
  for (auto vit = vertices.first; vit != vertices.second; ++vit) {
    const auto & node = graph_[*vit];
    if (node.type == type) {
      result.push_back(*vit);
    }
  }
  
  return result;
}

bool TopologicalMap::isEmpty() const
{
  return boost::num_vertices(graph_) == 0;
}

size_t TopologicalMap::numVertices() const
{
  return boost::num_vertices(graph_);
}

size_t TopologicalMap::numEdges() const
{
  return boost::num_edges(graph_);
}

void TopologicalMap::clear()
{
  graph_.clear();
  source_file_.clear();
  building_name_.clear();
  total_floors_ = 0;
  floor_names_.clear();
  id_to_vertex_.clear();
  name_to_vertex_.clear();
  cached_max_node_id_ = -1;
  cached_max_edge_id_ = -1;
}

double TopologicalMap::euclideanDistanceBetweenNodes(Vertex v1, Vertex v2) const
{
  const auto & p1 = graph_[v1].position;
  const auto & p2 = graph_[v2].position;
  
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  double dz = p1.z - p2.z;
  
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double TopologicalMap::costBetweenNodes(
  Vertex v1,
  Vertex v2,
  double penalize_z_movement) const
{
  const auto & p1 = graph_[v1].position;
  const auto & p2 = graph_[v2].position;
  
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  double dz = (p1.z - p2.z) * penalize_z_movement;
  
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool TopologicalMap::isOnSameFloor(
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

void TopologicalMap::convertGraphToMessage(
  const TopologicalGraph & graph,
  navbim_msgs::msg::Topomap & topomap_msg)
{
  // Clear output message
  topomap_msg = navbim_msgs::msg::Topomap();
  
  // Set metadata (empty for converted graphs - metadata comes from original source)
  topomap_msg.source_file = "";
  topomap_msg.building_name = "";
  topomap_msg.total_floors = 0;
  
  // Convert all nodes
  auto vertices = boost::vertices(graph);
  for (auto vit = vertices.first; vit != vertices.second; ++vit) {
    navbim_msgs::msg::TopomapNode node_msg;
    const auto & node = graph[*vit];
    
    node_msg.id = std::to_string(node.id);
    node_msg.name = node.name;
    node_msg.type = node.type;
    node_msg.floor = node.floor.value_or(std::vector<std::string>());
    
    // Position
    node_msg.position.x = node.position.x;
    node_msg.position.y = node.position.y;
    node_msg.position.z = node.position.z;
    
    // Floor-specific properties
    if (node.type == "floor") {
      node_msg.min_z = node.min_z.value_or(0.0);
      node_msg.max_z = node.max_z.value_or(0.0);
    }
    
    // Bounding box
    if (node.bbox.has_value()) {
      node_msg.bbox.min.x = node.bbox->min_x;
      node_msg.bbox.min.y = node.bbox->min_y;
      node_msg.bbox.min.z = node.bbox->min_z;
      node_msg.bbox.max.x = node.bbox->max_x;
      node_msg.bbox.max.y = node.bbox->max_y;
      node_msg.bbox.max.z = node.bbox->max_z;
    }
    
    // Polygon - convert from Boost.Geometry to ROS message
    if (node.polygon.has_value()) {
      const auto & poly = node.polygon.value();
      
      // Outer ring
      for (const auto & point : poly.outer()) {
        geometry_msgs::msg::Point32 pt;
        pt.x = static_cast<float>(point.x());
        pt.y = static_cast<float>(point.y());
        pt.z = 0.0f;
        node_msg.polygon.outer.points.push_back(pt);
      }
      
      // Inner rings (holes)
      for (const auto & inner : poly.inners()) {
        geometry_msgs::msg::Polygon hole;
        for (const auto & point : inner) {
          geometry_msgs::msg::Point32 pt;
          pt.x = static_cast<float>(point.x());
          pt.y = static_cast<float>(point.y());
          pt.z = 0.0f;
          hole.points.push_back(pt);
        }
        node_msg.polygon.holes.push_back(hole);
      }
    }
    
    if (node.ifc_guid.has_value()) {
      node_msg.ifc_guid = node.ifc_guid.value();
    }
    
    if (node.subtype.has_value()) {
      node_msg.subtype = node.subtype.value();
    }
    
    topomap_msg.nodes.nodes.push_back(node_msg);
  }
  
  // Convert all edges
  auto edges = boost::edges(graph);
  for (auto eit = edges.first; eit != edges.second; ++eit) {
    navbim_msgs::msg::TopomapEdge edge_msg;
    const auto & edge = graph[*eit];
    Vertex source = boost::source(*eit, graph);
    Vertex target = boost::target(*eit, graph);
    
    edge_msg.id = std::to_string(edge.id);
    edge_msg.source = std::to_string(graph[source].id);
    edge_msg.target = std::to_string(graph[target].id);
    edge_msg.type = edge.type;
    
    // Distance and cost metrics
    edge_msg.estimated_distance = edge.estimated_distance;
    edge_msg.planned_distance = edge.planned_distance;
    edge_msg.estimated_cost = edge.estimated_cost;
    edge_msg.planned_cost = edge.planned_cost;
    
    // Convert path waypoints to nav_msgs/Path
    if (edge.path && !edge.path->empty()) {
      edge_msg.path.header.frame_id = "ifc";
      for (const auto & waypoint : *edge.path) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "ifc";
        pose.pose.position.x = waypoint[0];
        pose.pose.position.y = waypoint[1];
        pose.pose.position.z = waypoint[2];
        pose.pose.orientation.w = 1.0;  // Identity orientation
        edge_msg.path.poses.push_back(pose);
      }
    }
    
    if (edge.subtype.has_value()) {
      edge_msg.subtype = edge.subtype.value();
    }
    
    if (edge.room_id.has_value()) {
      edge_msg.room_id = edge.room_id.value();
    }
    
    topomap_msg.edges.edges.push_back(edge_msg);
  }
}

}  // namespace navbim_util
