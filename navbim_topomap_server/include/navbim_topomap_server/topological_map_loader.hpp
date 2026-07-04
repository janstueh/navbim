#ifndef NAVBIM_TOPOMAP_SERVER__TOPOLOGICAL_MAP_LOADER_HPP_
#define NAVBIM_TOPOMAP_SERVER__TOPOLOGICAL_MAP_LOADER_HPP_

#include "navbim_util/topological_map_types.hpp"
#include <nlohmann/json.hpp>
#include <string>
#include <memory>

namespace navbim_topomap_server
{

// Use types from navbim_util
using navbim_util::TopologicalGraph;
using navbim_util::NodeProperties;
using navbim_util::EdgeProperties;
using navbim_util::Polygon;
using navbim_util::BoundingBox;
using navbim_util::Position;
using navbim_util::Vertex;

// For compatibility: TopologicalMap in topomap_server refers to the graph type
using TopologicalMap = TopologicalGraph;

/**
 * @brief Loads topological maps from JSON files
 * 
 * This class handles parsing NetworkX node-link format JSON files and
 * constructing Boost.Graph topological maps with all node and edge properties.
 */
class TopologicalMapLoader
{
public:
  /**
   * @brief Load a topological map from a JSON file
   * 
   * @param filename Path to the JSON file
   * @param graph Reference to the graph to populate
   * @return true if loaded successfully, false otherwise
   */
  static bool loadFromFile(const std::string & filename, TopologicalMap & graph);

  /**
   * @brief Load a topological map from JSON data
   * 
   * @param json_data nlohmann::json object containing the graph data
   * @param graph Reference to the graph to populate
   * @return true if loaded successfully, false otherwise
   */
  static bool loadFromJson(const nlohmann::json & json_data, TopologicalMap & graph);

  /**
   * @brief Save a topological map to a JSON file
   * 
   * @param filename Path to save the JSON file
   * @param graph The graph to save
   * @return true if saved successfully, false otherwise
   */
  static bool saveToFile(const std::string & filename, const TopologicalMap & graph);

  /**
   * @brief Convert graph to JSON data
   * 
   * @param graph The graph to convert
   * @return nlohmann::json object containing the graph data
   */
  static nlohmann::json toJson(const TopologicalMap & graph);

private:
  /**
   * @brief Parse node properties from JSON
   * 
   * @param json_node JSON object containing node data
   * @param props Reference to NodeProperties to populate
   */
  static void parseNodeProperties(const nlohmann::json & json_node, NodeProperties & props);

  /**
   * @brief Parse edge properties from JSON
   * 
   * @param json_edge JSON object containing edge data
   * @param props Reference to EdgeProperties to populate
   */
  static void parseEdgeProperties(const nlohmann::json & json_edge, EdgeProperties & props);

  /**
   * @brief Parse GeoJSON polygon to Boost.Geometry polygon
   * 
   * @param json_polygon JSON object containing GeoJSON polygon data
   * @param polygon Reference to Polygon to populate
   * @return true if parsed successfully, false otherwise
   */
  static bool parsePolygon(const nlohmann::json & json_polygon, Polygon & polygon);

  /**
   * @brief Convert Polygon to GeoJSON format
   * 
   * @param polygon The polygon to convert
   * @return nlohmann::json object in GeoJSON format
   */
  static nlohmann::json polygonToGeoJson(const Polygon & polygon);

  /**
   * @brief Parse bounding box from JSON
   * 
   * @param json_bbox JSON object containing bounding box data
   * @param bbox Reference to BoundingBox to populate
   */
  static void parseBoundingBox(const nlohmann::json & json_bbox, BoundingBox & bbox);

  /**
   * @brief Parse position from JSON
   * 
   * @param json_position JSON object containing position data
   * @param position Reference to Position to populate
   */
  static void parsePosition(const nlohmann::json & json_position, Position & position);
};

}  // namespace navbim_topomap_server

#endif  // NAVBIM_TOPOMAP_SERVER__TOPOLOGICAL_MAP_LOADER_HPP_
