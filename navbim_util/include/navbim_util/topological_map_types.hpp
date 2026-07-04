#ifndef NAVBIM_UTIL__TOPOLOGICAL_MAP_TYPES_HPP_
#define NAVBIM_UTIL__TOPOLOGICAL_MAP_TYPES_HPP_

#include <string>
#include <vector>
#include <map>
#include <optional>
#include <array>

#include <boost/graph/adjacency_list.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

namespace navbim_util
{

// Geometry types using Boost.Geometry
using Point2D = boost::geometry::model::d2::point_xy<double>;
using Polygon = boost::geometry::model::polygon<Point2D>;

/**
 * @brief 3D position structure
 */
struct Position {
  double x;
  double y;
  double z;
  
  Position() : x(0.0), y(0.0), z(0.0) {}
  Position(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
};

/**
 * @brief Bounding box structure
 */
struct BoundingBox {
  double min_x;
  double max_x;
  double min_y;
  double max_y;
  double min_z;
  double max_z;
  
  BoundingBox()
    : min_x(0.0), max_x(0.0), min_y(0.0), 
      max_y(0.0), min_z(0.0), max_z(0.0) {}
};

/**
 * @brief Node properties for the topological graph
 * 
 * This structure contains all possible properties that a node in the
 * topological map might have. Different node types (floor, room, transition,
 * stair, START, GOAL) use different subsets of these properties.
 */
struct NodeProperties {
  int id;                                    // Node ID (required)
  std::string name;                          // Node name (optional)
  std::string type;                          // Node type: "floor", "room", "transition", "stair", "START", "GOAL"
  std::optional<std::string> subtype;        // Subtype: "door" for transitions
  Position position;                         // Node position
  std::optional<std::string> ifc_guid;       // IFC GUID from BIM model
  
  // Properties for floors
  std::optional<double> min_z;               // Minimum Z height (floors)
  std::optional<double> max_z;               // Maximum Z height (floors)
  
  // Properties for rooms and transitions
  std::optional<BoundingBox> bbox;           // Bounding box
  std::optional<Polygon> polygon;            // 2D polygon geometry (Shapely equivalent)
  std::optional<std::vector<std::string>> floor; // Associated floor names
  
  // Default constructor
  NodeProperties() : id(-1), type("unknown") {}
};

/**
 * @brief Edge properties for the topological graph
 */
struct EdgeProperties {
  int id;                                    // Edge ID (required)
  std::string type;                          // Edge type: "floor", "room", "transition"
  std::optional<std::string> subtype;        // Subtype: "stair" for vertical transitions
  
  // Distance and cost metrics
  double estimated_distance;                 // Estimated distance (before planning)
  double planned_distance;                   // Actual planned distance (-1.0 if not planned)
  double estimated_cost;                     // Estimated cost
  double planned_cost;                       // Actual planned cost (-1.0 if not planned)
  
  // Path waypoints (if path has been planned)
  // Using shared_ptr to avoid expensive deep copies when reusing cached paths
  std::shared_ptr<std::vector<std::array<double, 3>>> path;   // List of [x, y, z] waypoints
  std::string path_frame_id;                 // Coordinate frame for path waypoints

  // Room association
  std::optional<int> room_id;                // ID of room this edge belongs to

  // Default constructor
  EdgeProperties()
    : id(-1),
      type("unknown"),
      estimated_distance(0.0),
      planned_distance(-1.0),
      estimated_cost(0.0),
      planned_cost(-1.0),
      path(nullptr),
      path_frame_id("ifc") {}
};

/**
 * @brief Topological map graph type definition
 * 
 * Uses Boost.Graph adjacency_list:
 * - OutEdgeList: vecS (vector) - edges stored in vector
 * - VertexList: vecS (vector) - vertices stored in vector
 * - Directed: undirectedS - undirected graph
 * - VertexProperties: NodeProperties (bundled properties)
 * - EdgeProperties: EdgeProperties (bundled properties)
 */
using TopologicalGraph = boost::adjacency_list<
  boost::vecS,        // OutEdgeList
  boost::vecS,        // VertexList
  boost::undirectedS, // Directed
  NodeProperties,     // VertexProperties (bundled)
  EdgeProperties      // EdgeProperties (bundled)
>;

// Type aliases for graph components
using Vertex = TopologicalGraph::vertex_descriptor;
using Edge = TopologicalGraph::edge_descriptor;
using VertexIterator = TopologicalGraph::vertex_iterator;
using EdgeIterator = TopologicalGraph::edge_iterator;

}  // namespace navbim_util

#endif  // NAVBIM_UTIL__TOPOLOGICAL_MAP_TYPES_HPP_
