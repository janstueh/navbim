#include "navbim_topomap_server/topological_map_loader.hpp"
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace navbim_topomap_server
{

// Use types from navbim_util for convenience
using navbim_util::Point2D;
using navbim_util::Polygon;
using navbim_util::BoundingBox;
using navbim_util::Position;
using navbim_util::NodeProperties;
using navbim_util::EdgeProperties;
using navbim_util::Vertex;
using navbim_util::Edge;
using navbim_util::VertexIterator;
using navbim_util::EdgeIterator;

bool TopologicalMapLoader::loadFromFile(const std::string & filename, TopologicalMap & graph)
{
  try {
    // Read JSON file
    std::ifstream file(filename);
    if (!file.is_open()) {
      std::cerr << "Failed to open file: " << filename << std::endl;
      return false;
    }

    nlohmann::json json_data;
    file >> json_data;
    file.close();

    return loadFromJson(json_data, graph);

  } catch (const std::exception & e) {
    std::cerr << "Exception loading topological map from file: " << e.what() << std::endl;
    return false;
  }
}

bool TopologicalMapLoader::loadFromJson(const nlohmann::json & json_data, TopologicalMap & graph)
{
  try {
    // Clear existing graph
    graph.clear();

    // Map from JSON node ID to graph vertex descriptor
    std::map<int, Vertex> id_to_vertex;

    // Parse nodes
    if (!json_data.contains("nodes") || !json_data["nodes"].is_array()) {
      std::cerr << "JSON data missing 'nodes' array" << std::endl;
      return false;
    }

    for (const auto & json_node : json_data["nodes"]) {
      // Add vertex to graph
      Vertex v = boost::add_vertex(graph);

      // Parse node properties
      parseNodeProperties(json_node, graph[v]);

      // Map ID to vertex
      id_to_vertex[graph[v].id] = v;
    }

    // Parse edges/links
    if (!json_data.contains("links") || !json_data["links"].is_array()) {
      std::cerr << "JSON data missing 'links' array" << std::endl;
      return false;
    }

    for (const auto & json_edge : json_data["links"]) {
      // Get source and target IDs
      if (!json_edge.contains("source") || !json_edge.contains("target")) {
        std::cerr << "Edge missing source or target" << std::endl;
        continue;
      }

      int source_id = json_edge["source"].get<int>();
      int target_id = json_edge["target"].get<int>();

      // Find vertices
      auto source_it = id_to_vertex.find(source_id);
      auto target_it = id_to_vertex.find(target_id);

      if (source_it == id_to_vertex.end() || target_it == id_to_vertex.end()) {
        std::cerr << "Edge references non-existent node: " 
                  << source_id << " -> " << target_id << std::endl;
        continue;
      }

      // Add edge to graph
      auto [e, added] = boost::add_edge(source_it->second, target_it->second, graph);

      if (added) {
        // Parse edge properties
        parseEdgeProperties(json_edge, graph[e]);
      }
    }

    return true;

  } catch (const std::exception & e) {
    std::cerr << "Exception loading topological map from JSON: " << e.what() << std::endl;
    return false;
  }
}

void TopologicalMapLoader::parseNodeProperties(
  const nlohmann::json & json_node, 
  NodeProperties & props)
{
  // Required properties
  if (json_node.contains("id")) {
    props.id = json_node["id"].get<int>();
  }

  if (json_node.contains("type")) {
    props.type = json_node["type"].get<std::string>();
  }

  // Optional properties
  if (json_node.contains("name")) {
    props.name = json_node["name"].get<std::string>();
  }

  if (json_node.contains("subtype")) {
    props.subtype = json_node["subtype"].get<std::string>();
  }

  if (json_node.contains("ifc_guid")) {
    props.ifc_guid = json_node["ifc_guid"].get<std::string>();
  }

  // Position
  if (json_node.contains("position")) {
    parsePosition(json_node["position"], props.position);
  }

  // Floor-specific properties
  if (json_node.contains("min_z")) {
    props.min_z = json_node["min_z"].get<double>();
  }

  if (json_node.contains("max_z")) {
    props.max_z = json_node["max_z"].get<double>();
  }

  // Bounding box
  if (json_node.contains("bbox")) {
    BoundingBox bbox;
    parseBoundingBox(json_node["bbox"], bbox);
    props.bbox = bbox;
  }

  // Polygon (GeoJSON format)
  if (json_node.contains("polygon")) {
    Polygon polygon;
    if (parsePolygon(json_node["polygon"], polygon)) {
      props.polygon = polygon;
    }
  }

  // Floor list
  if (json_node.contains("floor")) {
    std::vector<std::string> floor_list;
    if (json_node["floor"].is_array()) {
      for (const auto & floor_item : json_node["floor"]) {
        floor_list.push_back(floor_item.get<std::string>());
      }
    } else if (json_node["floor"].is_string()) {
      floor_list.push_back(json_node["floor"].get<std::string>());
    }
    if (!floor_list.empty()) {
      props.floor = floor_list;
    }
  }
}

void TopologicalMapLoader::parseEdgeProperties(
  const nlohmann::json & json_edge, 
  EdgeProperties & props)
{
  // Required properties
  if (json_edge.contains("id")) {
    props.id = json_edge["id"].get<int>();
  }

  if (json_edge.contains("type")) {
    props.type = json_edge["type"].get<std::string>();
  }

  // Optional properties
  if (json_edge.contains("subtype")) {
    props.subtype = json_edge["subtype"].get<std::string>();
  }

  if (json_edge.contains("room_id")) {
    props.room_id = json_edge["room_id"].get<int>();
  }

  // Distance and cost
  if (json_edge.contains("estimated_distance")) {
    props.estimated_distance = json_edge["estimated_distance"].get<double>();
  }

  if (json_edge.contains("planned_distance")) {
    props.planned_distance = json_edge["planned_distance"].get<double>();
  }

  if (json_edge.contains("estimated_cost")) {
    props.estimated_cost = json_edge["estimated_cost"].get<double>();
  }

  if (json_edge.contains("planned_cost")) {
    props.planned_cost = json_edge["planned_cost"].get<double>();
  }

  // Path waypoints
  if (json_edge.contains("path") && json_edge["path"].is_array()) {
    // Initialize shared_ptr if needed
    if (!props.path) {
      props.path = std::make_shared<std::vector<std::array<double, 3>>>();
    }
    props.path->reserve(json_edge["path"].size());
    for (const auto & waypoint : json_edge["path"]) {
      if (waypoint.is_array() && waypoint.size() >= 3) {
        std::array<double, 3> point = {
          waypoint[0].get<double>(),
          waypoint[1].get<double>(),
          waypoint[2].get<double>()
        };
        props.path->push_back(point);
      }
    }
  }
}

bool TopologicalMapLoader::parsePolygon(
  const nlohmann::json & json_polygon, 
  Polygon & polygon)
{
  try {
    // GeoJSON Polygon format: 
    // {
    //   "type": "Polygon",
    //   "coordinates": [[[x1, y1], [x2, y2], ...]]
    // }
    if (!json_polygon.contains("coordinates") || 
        !json_polygon["coordinates"].is_array() ||
        json_polygon["coordinates"].empty()) {
      return false;
    }

    const auto & rings = json_polygon["coordinates"];
    if (rings.empty() || !rings[0].is_array()) {
      return false;
    }

    // Parse outer ring (first ring in coordinates array)
    const auto & outer_ring = rings[0];
    
    std::vector<Point2D> points;
    for (const auto & coord : outer_ring) {
      if (coord.is_array() && coord.size() >= 2) {
        double x = coord[0].get<double>();
        double y = coord[1].get<double>();
        points.push_back(Point2D(x, y));
      }
    }

    // Assign points to polygon outer ring
    boost::geometry::assign_points(polygon, points);

    return true;

  } catch (const std::exception & e) {
    std::cerr << "Error parsing polygon: " << e.what() << std::endl;
    return false;
  }
}

void TopologicalMapLoader::parseBoundingBox(
  const nlohmann::json & json_bbox, 
  BoundingBox & bbox)
{
  if (json_bbox.contains("min_x")) bbox.min_x = json_bbox["min_x"].get<double>();
  if (json_bbox.contains("max_x")) bbox.max_x = json_bbox["max_x"].get<double>();
  if (json_bbox.contains("min_y")) bbox.min_y = json_bbox["min_y"].get<double>();
  if (json_bbox.contains("max_y")) bbox.max_y = json_bbox["max_y"].get<double>();
  if (json_bbox.contains("min_z")) bbox.min_z = json_bbox["min_z"].get<double>();
  if (json_bbox.contains("max_z")) bbox.max_z = json_bbox["max_z"].get<double>();
}

void TopologicalMapLoader::parsePosition(
  const nlohmann::json & json_position, 
  Position & position)
{
  if (json_position.contains("x")) position.x = json_position["x"].get<double>();
  if (json_position.contains("y")) position.y = json_position["y"].get<double>();
  if (json_position.contains("z")) position.z = json_position["z"].get<double>();
}

// Save functions
bool TopologicalMapLoader::saveToFile(
  const std::string & filename, 
  const TopologicalMap & graph)
{
  try {
    // Convert graph to JSON
    nlohmann::json json_data = toJson(graph);

    // Write to file
    std::ofstream file(filename);
    if (!file.is_open()) {
      std::cerr << "Failed to open file for writing: " << filename << std::endl;
      return false;
    }

    // Write with pretty printing (4 spaces indent)
    file << json_data.dump(4) << std::endl;
    file.close();

    return true;

  } catch (const std::exception & e) {
    std::cerr << "Exception saving topological map to file: " << e.what() << std::endl;
    return false;
  }
}

nlohmann::json TopologicalMapLoader::toJson(const TopologicalMap & graph)
{
  nlohmann::json result;
  
  try {
    // Create JSON structure with NetworkX node-link format
    result["directed"] = false;  // Assume undirected graph
    result["multigraph"] = false;
    result["graph"] = nlohmann::json::object();
    result["nodes"] = nlohmann::json::array();
    result["links"] = nlohmann::json::array();

    // Map from vertex descriptor to node ID for edge processing
    std::map<Vertex, int> vertex_to_id;

    // Iterate through all vertices to create nodes
    auto vertices = boost::vertices(graph);
    for (auto it = vertices.first; it != vertices.second; ++it) {
      Vertex v = *it;
      const NodeProperties & props = graph[v];

      // Create node JSON object
      nlohmann::json json_node;
      
      // Required properties
      json_node["id"] = props.id;
      json_node["type"] = props.type;

      // Optional properties
      if (!props.name.empty()) {
        json_node["name"] = props.name;
      }

      if (props.subtype.has_value() && !props.subtype->empty()) {
        json_node["subtype"] = props.subtype.value();
      }

      if (props.ifc_guid.has_value() && !props.ifc_guid->empty()) {
        json_node["ifc_guid"] = props.ifc_guid.value();
      }

      // Position
      json_node["position"] = {
        {"x", props.position.x},
        {"y", props.position.y},
        {"z", props.position.z}
      };

      // Floor-specific properties
      if (props.min_z.has_value()) {
        json_node["min_z"] = props.min_z.value();
      }

      if (props.max_z.has_value()) {
        json_node["max_z"] = props.max_z.value();
      }

      // Bounding box
      if (props.bbox.has_value()) {
        json_node["bbox"] = {
          {"min_x", props.bbox->min_x},
          {"max_x", props.bbox->max_x},
          {"min_y", props.bbox->min_y},
          {"max_y", props.bbox->max_y},
          {"min_z", props.bbox->min_z},
          {"max_z", props.bbox->max_z}
        };
      }

      // Polygon (GeoJSON format)
      if (props.polygon.has_value()) {
        json_node["polygon"] = polygonToGeoJson(props.polygon.value());
      }

      // Floor list
      if (props.floor.has_value() && !props.floor->empty()) {
        json_node["floor"] = props.floor.value();
      }

      // Add node to array
      result["nodes"].push_back(json_node);

      // Store mapping for edge processing
      vertex_to_id[v] = props.id;
    }

    // Iterate through all edges to create links
    auto edges = boost::edges(graph);
    for (auto it = edges.first; it != edges.second; ++it) {
      Edge e = *it;
      const EdgeProperties & props = graph[e];

      // Get source and target vertices
      Vertex source = boost::source(e, graph);
      Vertex target = boost::target(e, graph);

      // Create edge JSON object
      nlohmann::json json_edge;

      // Required properties
      json_edge["id"] = props.id;
      json_edge["source"] = vertex_to_id[source];
      json_edge["target"] = vertex_to_id[target];
      json_edge["type"] = props.type;

      // Optional properties
      if (props.subtype.has_value() && !props.subtype->empty()) {
        json_edge["subtype"] = props.subtype.value();
      }

      if (props.room_id.has_value()) {
        json_edge["room_id"] = props.room_id.value();
      }

      // Distance and cost (only for transition edges, not room/floor edges)
      if (props.type == "transition") {
        if (props.estimated_distance != 0.0) {
          json_edge["estimated_distance"] = props.estimated_distance;
        }

        if (props.planned_distance != 0.0) {
          json_edge["planned_distance"] = props.planned_distance;
        }

        if (props.estimated_cost != 0.0) {
          json_edge["estimated_cost"] = props.estimated_cost;
        }

        if (props.planned_cost != 0.0) {
          json_edge["planned_cost"] = props.planned_cost;
        }

        // Path waypoints (only for transition edges)
        if (props.path && !props.path->empty()) {
          nlohmann::json path_array = nlohmann::json::array();
          for (const auto & waypoint : *props.path) {
            path_array.push_back({waypoint[0], waypoint[1], waypoint[2]});
          }
          json_edge["path"] = path_array;
        }
      }

      // Add edge to array
      result["links"].push_back(json_edge);
    }

    return result;

  } catch (const std::exception & e) {
    std::cerr << "Exception converting graph to JSON: " << e.what() << std::endl;
    return nlohmann::json::object();
  }
}

nlohmann::json TopologicalMapLoader::polygonToGeoJson(const Polygon & polygon)
{
  nlohmann::json result;
  
  try {
    // GeoJSON Polygon format:
    // {
    //   "type": "Polygon",
    //   "coordinates": [[[x1, y1], [x2, y2], ...]]
    // }
    
    result["type"] = "Polygon";
    
    nlohmann::json coordinates = nlohmann::json::array();
    nlohmann::json outer_ring = nlohmann::json::array();

    // Extract outer ring points
    for (const auto & point : boost::geometry::exterior_ring(polygon)) {
      outer_ring.push_back({
        boost::geometry::get<0>(point),
        boost::geometry::get<1>(point)
      });
    }

    // GeoJSON polygons should have outer ring as first element in coordinates array
    coordinates.push_back(outer_ring);
    
    result["coordinates"] = coordinates;

    return result;

  } catch (const std::exception & e) {
    std::cerr << "Error converting polygon to GeoJSON: " << e.what() << std::endl;
    return nlohmann::json::object();
  }
}

}  // namespace navbim_topomap_server
