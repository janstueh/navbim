#include "navbim_topomap_server/topomap_server.hpp"

#include <string>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "navbim_msgs/msg/topomap_node.hpp"
#include "navbim_msgs/msg/topomap_nodes.hpp"
#include "navbim_msgs/srv/save_topological_map.hpp"

using namespace std::placeholders;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace navbim_topomap_server
{

TopomapServer::TopomapServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("topomap_server", "", options)
{
}

TopomapServer::~TopomapServer()
{
}

CallbackReturn TopomapServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");
  
  try {
    // Declare and get parameters
    declare_parameter("topomap_file", "");
    get_parameter("topomap_file", topomap_file_);
    
    declare_parameter("global_frame", "ifc");
    get_parameter("global_frame", global_frame_);
    
    declare_parameter("save_at_shutdown", false);
    get_parameter("save_at_shutdown", save_at_shutdown_);
    
    // Create service servers
    get_floor_nodes_service_ = create_service<navbim_msgs::srv::GetFloorNodes>(
      "topomap_server/get_floor_nodes",
      std::bind(&TopomapServer::getFloorNodesCallback, this, _1, _2, _3));
    
    get_room_nodes_service_ = create_service<navbim_msgs::srv::GetRoomNodes>(
      "topomap_server/get_room_nodes",
      std::bind(&TopomapServer::getRoomNodesCallback, this, _1, _2, _3));
    
    get_room_neighbors_service_ = create_service<navbim_msgs::srv::GetRoomNeighbors>(
      "topomap_server/get_room_neighbors",
      std::bind(&TopomapServer::getRoomNeighborsCallback, this, _1, _2, _3));
    
    get_adjacent_transition_nodes_service_ = create_service<navbim_msgs::srv::GetAdjacentTransitionNodes>(
      "topomap_server/get_adjacent_transition_nodes",
      std::bind(&TopomapServer::getAdjacentTransitionNodesCallback, this, _1, _2, _3));
    
    load_topomap_service_ = create_service<navbim_msgs::srv::LoadTopomap>(
      "topomap_server/load_topomap",
      std::bind(&TopomapServer::loadTopomapCallback, this, _1, _2, _3));
    
    get_room_by_coordinates_service_ = create_service<navbim_msgs::srv::GetRoomByCoordinates>(
      "topomap_server/get_room_by_coordinates",
      std::bind(&TopomapServer::getRoomByCoordinatesCallback, this, _1, _2, _3));
    
    get_topological_map_service_ = create_service<navbim_msgs::srv::GetTopologicalMap>(
      "topomap_server/get_topological_map",
      std::bind(&TopomapServer::getTopologicalMapCallback, this, _1, _2, _3));
    
    get_min_z_service_ = create_service<navbim_msgs::srv::GetMinZ>(
      "topomap_server/get_min_z",
      std::bind(&TopomapServer::getMinZCallback, this, _1, _2, _3));
    
    is_point_in_room_polygon_service_ = create_service<navbim_msgs::srv::IsPointInRoomPolygon>(
      "topomap_server/is_point_in_room_polygon",
      std::bind(&TopomapServer::isPointInRoomPolygonCallback, this, _1, _2, _3));
    
    update_edge_data_service_ = create_service<navbim_msgs::srv::UpdateEdgeData>(
      "topomap_server/update_edge_data",
      std::bind(&TopomapServer::updateEdgeDataCallback, this, _1, _2, _3));
    
    save_topological_map_service_ = create_service<navbim_msgs::srv::SaveTopologicalMap>(
      "topomap_server/save_topological_map",
      std::bind(&TopomapServer::saveTopologicalMapCallback, this, _1, _2, _3));
    
    clear_topological_map_paths_service_ = create_service<navbim_msgs::srv::ClearTopologicalMapPaths>(
      "topomap_server/clear_topological_map_paths",
      std::bind(&TopomapServer::clearTopologicalMapPathsCallback, this, _1, _2, _3));
    
    // Load initial topological map if specified
    if (!topomap_file_.empty()) {
      if (loadFromFile(topomap_file_)) {
        RCLCPP_INFO(get_logger(), "Loaded topological map from: %s", topomap_file_.c_str());
      } else {
        RCLCPP_WARN(get_logger(), "Failed to load topological map from: %s", topomap_file_.c_str());
      }
    }
    
    return CallbackReturn::SUCCESS;
    
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Error during configuration: %s", e.what());
    return CallbackReturn::FAILURE;
  }
}

CallbackReturn TopomapServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating TopomapServer");
  
  // Create bond connection (handled by nav2_util::LifecycleNode base class)
  createBond();
  
  return CallbackReturn::SUCCESS;
}

CallbackReturn TopomapServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating TopomapServer");
  
  // Destroy bond connection
  destroyBond();
  
  return CallbackReturn::SUCCESS;
}

CallbackReturn TopomapServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up TopomapServer");
  
  try {
    // Save topological map before cleanup if enabled
    if (save_at_shutdown_ && !topomap_file_.empty()) {
      std::lock_guard<std::mutex> lock(graph_mutex_);
      
      if (TopologicalMapLoader::saveToFile(topomap_file_, graph_)) {
        RCLCPP_INFO(get_logger(), "Saved topological map to: %s", topomap_file_.c_str());
      } else {
        RCLCPP_WARN(get_logger(), "Failed to save topological map to: %s", topomap_file_.c_str());
      }
    }
    
    // Reset services
    get_floor_nodes_service_.reset();
    get_room_nodes_service_.reset();
    get_room_neighbors_service_.reset();
    get_adjacent_transition_nodes_service_.reset();
    load_topomap_service_.reset();
    get_room_by_coordinates_service_.reset();
    get_topological_map_service_.reset();
    get_min_z_service_.reset();
    is_point_in_room_polygon_service_.reset();
    update_edge_data_service_.reset();
    save_topological_map_service_.reset();
    clear_topological_map_paths_service_.reset();
    
    // Clear graph
    clearGraph();
    
    return CallbackReturn::SUCCESS;
    
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Error during cleanup: %s", e.what());
    return CallbackReturn::FAILURE;
  }
}

CallbackReturn TopomapServer::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down TopomapServer");
  
  // Save topological map if enabled
  if (save_at_shutdown_ && !topomap_file_.empty()) {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    
    if (TopologicalMapLoader::saveToFile(topomap_file_, graph_)) {
      RCLCPP_INFO(get_logger(), "Saved topological map to: %s", topomap_file_.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "Failed to save topological map to: %s", topomap_file_.c_str());
    }
  }
  
  return CallbackReturn::SUCCESS;
}

CallbackReturn TopomapServer::on_error(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_ERROR(get_logger(), "TopomapServer entered error state");
  return CallbackReturn::SUCCESS;
}

void TopomapServer::getFloorNodesCallback(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<navbim_msgs::srv::GetFloorNodes::Request> /*request*/,
  std::shared_ptr<navbim_msgs::srv::GetFloorNodes::Response> response)
{
  std::lock_guard<std::mutex> lock(graph_mutex_);
  
  try {
    if (boost::num_vertices(graph_) == 0) {
      response->success = false;
      response->message = "Topological graph not initialized";
      return;
    }
    
    // Get floor nodes
    auto floor_vertices = TopologicalMapUtils::getFloorNodes(graph_);
    
    if (!floor_vertices.empty()) {
      navbim_msgs::msg::TopomapNodes topomap_nodes;
      
      for (const auto & vertex : floor_vertices) {
        const auto & node = graph_[vertex];
        
        navbim_msgs::msg::TopomapNode topomap_node;
        topomap_node.id = std::to_string(node.id);
        topomap_node.name = node.name;
        topomap_node.type = node.type;
        topomap_node.ifc_guid = node.ifc_guid.value_or("");
        topomap_node.min_z = node.min_z.value_or(0.0);
        topomap_node.max_z = node.max_z.value_or(0.0);
        topomap_node.subtype = node.subtype.value_or("");
        
        // Handle floor list
        if (node.floor.has_value()) {
          topomap_node.floor = *node.floor;
        }
        
        topomap_nodes.nodes.push_back(topomap_node);
      }
      
      response->nodes = topomap_nodes;
      response->success = true;
      response->message = "Floor nodes retrieved successfully";
    } else {
      response->success = false;
      response->message = "No floor nodes found";
    }
    
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Error in getFloorNodesCallback: %s", e.what());
    response->success = false;
    response->message = std::string("Error retrieving floor nodes: ") + e.what();
  }
}

void TopomapServer::getRoomNodesCallback(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<navbim_msgs::srv::GetRoomNodes::Request> /*request*/,
  std::shared_ptr<navbim_msgs::srv::GetRoomNodes::Response> response)
{
  std::lock_guard<std::mutex> lock(graph_mutex_);
  
  try {
    if (boost::num_vertices(graph_) == 0) {
      response->success = false;
      response->message = "Topological graph not initialized";
      return;
    }
    
    // Get room nodes
    auto room_vertices = TopologicalMapUtils::getRoomNodes(graph_);
    
    if (!room_vertices.empty()) {
      navbim_msgs::msg::TopomapNodes topomap_nodes;
      
      for (const auto & vertex : room_vertices) {
        const auto & node = graph_[vertex];
        
        navbim_msgs::msg::TopomapNode topomap_node;
        topomap_node.id = std::to_string(node.id);
        topomap_node.name = node.name;
        topomap_node.type = node.type;
        topomap_node.ifc_guid = node.ifc_guid.value_or("");
        topomap_node.subtype = node.subtype.value_or("");
        
        // Handle bounding box
        if (node.bbox.has_value()) {
          topomap_node.bbox.min.x = node.bbox->min_x;
          topomap_node.bbox.min.y = node.bbox->min_y;
          topomap_node.bbox.min.z = node.bbox->min_z;
          topomap_node.bbox.max.x = node.bbox->max_x;
          topomap_node.bbox.max.y = node.bbox->max_y;
          topomap_node.bbox.max.z = node.bbox->max_z;
        }
        
        // Handle floor list
        if (node.floor.has_value()) {
          topomap_node.floor = *node.floor;
        }
        
        topomap_nodes.nodes.push_back(topomap_node);
      }
      
      response->nodes = topomap_nodes;
      response->success = true;
      response->message = "Room nodes retrieved successfully";
    } else {
      response->success = false;
      response->message = "No room nodes found";
    }
    
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Error in getRoomNodesCallback: %s", e.what());
    response->success = false;
    response->message = std::string("Error retrieving room nodes: ") + e.what();
  }
}

void TopomapServer::getRoomNeighborsCallback(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<navbim_msgs::srv::GetRoomNeighbors::Request> request,
  std::shared_ptr<navbim_msgs::srv::GetRoomNeighbors::Response> response)
{
  std::lock_guard<std::mutex> lock(graph_mutex_);
  
  try {
    if (boost::num_vertices(graph_) == 0) {
      response->success = false;
      response->message = "Topological graph not initialized";
      return;
    }
    
    // Get room neighbors
    auto neighbor_vertices = TopologicalMapUtils::getRoomNeighbors(graph_, request->id);
    
    if (!neighbor_vertices.empty()) {
      navbim_msgs::msg::TopomapNodes topomap_nodes;
      
      for (const auto & vertex : neighbor_vertices) {
        const auto & node = graph_[vertex];
        
        navbim_msgs::msg::TopomapNode topomap_node;
        topomap_node.id = std::to_string(node.id);
        topomap_node.name = node.name;
        topomap_node.type = node.type;
        topomap_node.ifc_guid = node.ifc_guid.value_or("");
        topomap_node.min_z = node.min_z.value_or(0.0);
        topomap_node.max_z = node.max_z.value_or(0.0);
        topomap_node.subtype = node.subtype.value_or("");
        
        // Handle floor list
        if (node.floor.has_value()) {
          topomap_node.floor = *node.floor;
        }
        
        topomap_nodes.nodes.push_back(topomap_node);
      }
      
      response->nodes = topomap_nodes;
      response->success = true;
      response->message = "Room neighbors retrieved successfully";
    } else {
      response->success = false;
      response->message = std::string("No neighboring rooms found for room ") + request->id;
    }
    
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Error in getRoomNeighborsCallback: %s", e.what());
    response->success = false;
    response->message = std::string("Error retrieving room neighbors: ") + e.what();
  }
}

void TopomapServer::getAdjacentTransitionNodesCallback(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<navbim_msgs::srv::GetAdjacentTransitionNodes::Request> request,
  std::shared_ptr<navbim_msgs::srv::GetAdjacentTransitionNodes::Response> response)
{
  std::lock_guard<std::mutex> lock(graph_mutex_);
  
  try {
    if (boost::num_vertices(graph_) == 0) {
      response->success = false;
      response->message = "Topological graph not initialized";
      return;
    }
    
    // Get adjacent transition nodes
    auto transition_vertices = TopologicalMapUtils::getAdjacentTransitionNodes(graph_, request->room_name);
    
    if (!transition_vertices.empty()) {
      response->transition_nodes.nodes.clear();
      
      for (const auto & vertex : transition_vertices) {
        const auto & node = graph_[vertex];
        
        navbim_msgs::msg::TopomapNode topomap_node = convertToTopomapNodeMsg(node);
        response->transition_nodes.nodes.push_back(topomap_node);
      }
      
      response->success = true;
      response->message = "Adjacent transition nodes retrieved successfully";
    } else {
      response->success = true;  // Not an error - room may have no adjacent transitions
      response->message = std::string("No adjacent transition nodes found for room ") + request->room_name;
    }
    
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Error in getAdjacentTransitionNodesCallback: %s", e.what());
    response->success = false;
    response->message = std::string("Error retrieving adjacent transition nodes: ") + e.what();
  }
}

void TopomapServer::loadTopomapCallback(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<navbim_msgs::srv::LoadTopomap::Request> request,
  std::shared_ptr<navbim_msgs::srv::LoadTopomap::Response> response)
{
  std::lock_guard<std::mutex> lock(graph_mutex_);
  
  try {
    if (loadFromFile(request->filename)) {
      topomap_file_ = request->filename;  // Update current file
      response->success = true;
      response->message = std::string("Topological map loaded successfully from ") + request->filename;
    } else {
      response->success = false;
      response->message = std::string("Failed to load topological map from ") + request->filename;
    }
    
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Error in loadTopomapCallback: %s", e.what());
    response->success = false;
    response->message = std::string("Error loading topological map: ") + e.what();
  }
}

navbim_msgs::msg::TopomapNode TopomapServer::convertToTopomapNodeMsg(
  const NodeProperties & node_props) const
{
  navbim_msgs::msg::TopomapNode msg;
  
  // Basic properties
  msg.id = std::to_string(node_props.id);
  msg.name = node_props.name;
  msg.type = node_props.type;
  msg.ifc_guid = node_props.ifc_guid.value_or("");
  
  // Position
  msg.position.x = node_props.position.x;
  msg.position.y = node_props.position.y;
  msg.position.z = node_props.position.z;
  
  // Floor-specific properties
  msg.min_z = node_props.min_z.value_or(0.0);
  msg.max_z = node_props.max_z.value_or(0.0);
  
  // Subtype (for transitions)
  msg.subtype = node_props.subtype.value_or("");
  
  // Floor association (for rooms and transitions)
  if (node_props.floor.has_value()) {
    msg.floor = node_props.floor.value();
  }
  
  // Bounding box (for rooms and transitions)
  if (node_props.bbox.has_value()) {
    const auto & bbox = node_props.bbox.value();
    msg.bbox.min.x = bbox.min_x;
    msg.bbox.min.y = bbox.min_y;
    msg.bbox.min.z = bbox.min_z;
    msg.bbox.max.x = bbox.max_x;
    msg.bbox.max.y = bbox.max_y;
    msg.bbox.max.z = bbox.max_z;
  }
  
  // Polygon (for rooms) - convert from Boost.Geometry to ROS message
  if (node_props.polygon.has_value()) {
    const auto & boost_poly = node_props.polygon.value();
    
    // Convert outer boundary
    for (const auto & point : boost::geometry::exterior_ring(boost_poly)) {
      geometry_msgs::msg::Point32 pt;
      pt.x = boost::geometry::get<0>(point);
      pt.y = boost::geometry::get<1>(point);
      pt.z = 0.0;
      msg.polygon.outer.points.push_back(pt);
    }
    
    // Convert holes (inner rings)
    const auto & inners = boost::geometry::interior_rings(boost_poly);
    for (const auto & inner : inners) {
      geometry_msgs::msg::Polygon hole;
      for (const auto & point : inner) {
        geometry_msgs::msg::Point32 pt;
        pt.x = boost::geometry::get<0>(point);
        pt.y = boost::geometry::get<1>(point);
        pt.z = 0.0;
        hole.points.push_back(pt);
      }
      msg.polygon.holes.push_back(hole);
    }
  }
  
  return msg;
}

void TopomapServer::getRoomByCoordinatesCallback(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<navbim_msgs::srv::GetRoomByCoordinates::Request> request,
  std::shared_ptr<navbim_msgs::srv::GetRoomByCoordinates::Response> response)
{
  std::lock_guard<std::mutex> lock(graph_mutex_);
  
  try {
    if (boost::num_vertices(graph_) == 0) {
      response->success = false;
      response->message = "Topological graph not initialized";
      return;
    }
    
    // Extract coordinates from request
    double x = request->coordinates.x;
    double y = request->coordinates.y;
    double z = request->coordinates.z;
    
    // Find floor using height (z-coordinate)
    auto floor_vertex_opt = TopologicalMapUtils::findFloorByHeight(graph_, z);
    
    if (!floor_vertex_opt.has_value()) {
      response->success = false;
      response->message = std::string("No floor found for coordinates (") + 
                          std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z) + ")";
      return;
    }
    
    const auto & floor_node = graph_[*floor_vertex_opt];
    
    // Find room using x,y coordinates and the determined floor
    auto room_vertex_opt = TopologicalMapUtils::findRoomByCoordinates(graph_, x, y, z);
    
    if (!room_vertex_opt.has_value()) {
      response->success = false;
      response->message = std::string("No room found for coordinates (") + 
                          std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z) + 
                          ") on floor " + floor_node.name;
      return;
    }
    
    // Get room data
    const auto & room_node = graph_[*room_vertex_opt];
    
    // Convert to TopomapNode messages
    response->floor_node = convertToTopomapNodeMsg(floor_node);
    response->room_node = convertToTopomapNodeMsg(room_node);
    
    // Set successful response
    response->success = true;
    response->message = std::string("Found floor '") + floor_node.name + 
                        "' and room '" + room_node.name + "'";
    
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Error in getRoomByCoordinatesCallback: %s", e.what());
    response->success = false;
    response->message = std::string("Error determining room by coordinates: ") + e.what();
  }
}

void TopomapServer::getMinZCallback(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<navbim_msgs::srv::GetMinZ::Request> request,
  std::shared_ptr<navbim_msgs::srv::GetMinZ::Response> response)
{
  std::lock_guard<std::mutex> lock(graph_mutex_);
  
  try {
    if (boost::num_vertices(graph_) == 0) {
      response->success = false;
      response->message = "Topological graph not initialized";
      return;
    }
    
    // Find node by name and get its min_z
    auto vertex_opt = TopologicalMapUtils::getVertexByName(graph_, request->name);
    
    if (!vertex_opt.has_value()) {
      response->success = false;
      response->message = std::string("Node with name '") + request->name + "' not found in graph";
      response->min_z = 0.0;
      return;
    }
    
    const auto & node = graph_[*vertex_opt];
    double min_z = 0.0;
    bool found = false;
    
    // For rooms and transitions, use bbox.min_z
    if ((node.type == "room" || node.type == "transition") && node.bbox.has_value()) {
      min_z = node.bbox->min_z;
      found = true;
    }
    // For floors, use min_z field
    else if (node.type == "floor" && node.min_z.has_value()) {
      min_z = *node.min_z;
      found = true;
    }
    
    if (found) {
      response->success = true;
      response->message = std::string("Found node '") + request->name + 
                         "' with minimum z-coordinate " + std::to_string(min_z);
      response->min_z = min_z;
    } else {
      response->success = false;
      response->message = std::string("Node '") + request->name + "' found but has no min_z value";
      response->min_z = 0.0;
    }
    
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Error in getMinZCallback: %s", e.what());
    response->success = false;
    response->message = std::string("Error retrieving node min_z: ") + e.what();
    response->min_z = 0.0;
  }
}

void TopomapServer::isPointInRoomPolygonCallback(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<navbim_msgs::srv::IsPointInRoomPolygon::Request> request,
  std::shared_ptr<navbim_msgs::srv::IsPointInRoomPolygon::Response> response)
{
  std::lock_guard<std::mutex> lock(graph_mutex_);
  
  try {
    if (boost::num_vertices(graph_) == 0) {
      response->success = false;
      response->message = "Topological graph not initialized";
      return;
    }
    
    // Extract coordinates and room name from request
    double x = request->point.x;
    double y = request->point.y;
    std::string room_name = request->room_name;
    
    // Find room node by name
    auto vertex_opt = TopologicalMapUtils::getVertexByName(graph_, room_name);
    
    if (!vertex_opt.has_value()) {
      response->success = false;
      response->message = std::string("Room with name '") + room_name + "' not found in graph";
      response->is_inside = false;
      return;
    }
    
    const auto & node = graph_[*vertex_opt];
    
    if (node.type != "room") {
      response->success = false;
      response->message = std::string("Node '") + room_name + "' is not a room (type: " + node.type + ")";
      response->is_inside = false;
      return;
    }
    
    if (!node.polygon.has_value()) {
      response->success = false;
      response->message = std::string("Room '") + room_name + "' does not have a valid polygon";
      response->is_inside = false;
      return;
    }
    
    // Check if point is inside the room polygon
    bool is_inside = TopologicalMapUtils::isPointInPolygon(*node.polygon, x, y);
    
    response->is_inside = is_inside;
    response->success = true;
    if (is_inside) {
      response->message = std::string("Point (") + std::to_string(x) + ", " + std::to_string(y) + 
                         ") is inside room " + room_name;
    } else {
      response->message = std::string("Point (") + std::to_string(x) + ", " + std::to_string(y) + 
                         ") is not inside room " + room_name;
    }
    
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Error in isPointInRoomPolygonCallback: %s", e.what());
    response->success = false;
    response->message = std::string("Error checking point in room: ") + e.what();
    response->is_inside = false;
  }
}

bool TopomapServer::loadFromFile(const std::string & filename)
{
  try {
    TopologicalMap new_graph;
    bool success = TopologicalMapLoader::loadFromFile(filename, new_graph);
    if (success) {
      graph_ = std::move(new_graph);
      
      // Extract metadata
      topomap_file_ = filename;
      
      // Extract building name from filename (simplified - can be improved)
      size_t last_slash = filename.find_last_of("/\\");
      size_t last_dot = filename.find_last_of(".");
      if (last_slash != std::string::npos && last_dot != std::string::npos) {
        building_name_ = filename.substr(last_slash + 1, last_dot - last_slash - 1);
      } else {
        building_name_ = "unknown";
      }
      
      // Extract floor names by iterating through all floor nodes
      floor_names_.clear();
      auto vertices = boost::vertices(graph_);
      for (auto vit = vertices.first; vit != vertices.second; ++vit) {
        const auto & node = graph_[*vit];
        if (node.type == "floor") {
          floor_names_.push_back(node.name);
        }
      }
    }
    return success;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Error loading topological map from %s: %s", filename.c_str(), e.what());
    return false;
  }
}

void TopomapServer::clearGraph()
{
  std::lock_guard<std::mutex> lock(graph_mutex_);
  graph_.clear();
  building_name_.clear();
  floor_names_.clear();
}

navbim_msgs::msg::TopomapEdge TopomapServer::convertToTopomapEdgeMsg(
  const EdgeProperties & edge_props,
  Vertex source_vertex,
  Vertex target_vertex) const
{
  navbim_msgs::msg::TopomapEdge msg;
  
  // Basic properties
  msg.id = std::to_string(edge_props.id);
  msg.type = edge_props.type;
  msg.source = std::to_string(graph_[source_vertex].id);
  msg.target = std::to_string(graph_[target_vertex].id);
  
  // Subtype (optional)
  msg.subtype = edge_props.subtype.has_value() ? edge_props.subtype.value() : "";
  
  // Room association
  msg.room_id = edge_props.room_id.has_value() ? 
                std::to_string(edge_props.room_id.value()) : "";
  
  // Distance and cost metrics
  msg.estimated_distance = edge_props.estimated_distance;
  msg.planned_distance = edge_props.planned_distance;
  msg.estimated_cost = edge_props.estimated_cost;
  msg.planned_cost = edge_props.planned_cost;
  
  // Convert waypoints to nav_msgs/Path
  if (edge_props.path && !edge_props.path->empty()) {
    msg.path.header.frame_id = edge_props.path_frame_id;
    for (const auto & waypoint : *edge_props.path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = edge_props.path_frame_id;
      pose.pose.position.x = waypoint[0];
      pose.pose.position.y = waypoint[1];
      pose.pose.position.z = waypoint[2];
      // Orientation is identity (not stored in topological map)
      pose.pose.orientation.w = 1.0;
      msg.path.poses.push_back(pose);
    }
  }
  
  return msg;
}

void TopomapServer::getTopologicalMapCallback(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<navbim_msgs::srv::GetTopologicalMap::Request> /*request*/,
  std::shared_ptr<navbim_msgs::srv::GetTopologicalMap::Response> response)
{
  std::lock_guard<std::mutex> lock(graph_mutex_);
  
  try {
    if (boost::num_vertices(graph_) == 0) {
      response->success = false;
      response->message = "Topological graph not initialized";
      return;
    }
    
    // Set metadata
    response->topomap.source_file = topomap_file_;
    response->topomap.building_name = building_name_;
    response->topomap.total_floors = static_cast<uint32_t>(floor_names_.size());
    response->topomap.floor_names = floor_names_;
    
    // Convert all nodes
    auto vertices = boost::vertices(graph_);
    for (auto vit = vertices.first; vit != vertices.second; ++vit) {
      const auto & node = graph_[*vit];
      response->topomap.nodes.nodes.push_back(convertToTopomapNodeMsg(node));
    }
    
    // Convert all edges
    auto edges = boost::edges(graph_);
    for (auto eit = edges.first; eit != edges.second; ++eit) {
      const auto & edge = graph_[*eit];
      Vertex source = boost::source(*eit, graph_);
      Vertex target = boost::target(*eit, graph_);
      response->topomap.edges.edges.push_back(convertToTopomapEdgeMsg(edge, source, target));
    }
    
    response->success = true;
    response->message = std::string("Topological map retrieved: ") + 
                        std::to_string(boost::num_vertices(graph_)) + " nodes, " +
                        std::to_string(boost::num_edges(graph_)) + " edges";
    
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Error in getTopologicalMapCallback: %s", e.what());
    response->success = false;
    response->message = std::string("Error retrieving topological map: ") + e.what();
  }
}

void TopomapServer::updateEdgeDataCallback(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<navbim_msgs::srv::UpdateEdgeData::Request> request,
  std::shared_ptr<navbim_msgs::srv::UpdateEdgeData::Response> response)
{
  std::lock_guard<std::mutex> lock(graph_mutex_);
  
  try {
    // Find vertices by ID
    auto source_vertex_opt = TopologicalMapUtils::getVertexById(graph_, std::to_string(request->source_id));
    auto target_vertex_opt = TopologicalMapUtils::getVertexById(graph_, std::to_string(request->target_id));
    
    // Validate vertices found
    if (!source_vertex_opt.has_value()) {
      response->success = false;
      response->message = "Source vertex with ID " + std::to_string(request->source_id) + " not found";
      RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
      return;
    }
    
    Vertex source_vertex = *source_vertex_opt;
    Vertex target_vertex;
    
    if (!target_vertex_opt.has_value()) {
      target_vertex = boost::graph_traits<TopologicalMap>::null_vertex();
    } else {
      target_vertex = *target_vertex_opt;
    }
    
    if (target_vertex == boost::graph_traits<TopologicalMap>::null_vertex()) {
      response->success = false;
      response->message = "Target vertex with ID " + std::to_string(request->target_id) + " not found";
      RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
      return;
    }
    
    // Find edge (try both directions since graph is undirected)
    auto edge_pair = boost::edge(source_vertex, target_vertex, graph_);
    if (!edge_pair.second) {
      response->success = false;
      response->message = "Edge not found between vertices " + 
                         std::to_string(request->source_id) + " and " + 
                         std::to_string(request->target_id);
      RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
      return;
    }
    
    // Update edge properties
    auto& edge_props = graph_[edge_pair.first];
    edge_props.planned_distance = request->planned_distance;
    edge_props.planned_cost = request->planned_cost;
    
    // Convert nav_msgs/Path to vector of arrays (using shared_ptr)
    edge_props.path = std::make_shared<std::vector<std::array<double, 3>>>();
    edge_props.path->reserve(request->path.poses.size());
    edge_props.path_frame_id = request->path.header.frame_id.empty()
      ? "ifc" : request->path.header.frame_id;
    for (const auto& pose_stamped : request->path.poses) {
      edge_props.path->push_back({
        pose_stamped.pose.position.x,
        pose_stamped.pose.position.y,
        pose_stamped.pose.position.z
      });
    }
    
    response->success = true;
    response->message = "Successfully updated edge between vertices " + 
                       std::to_string(request->source_id) + " and " + 
                       std::to_string(request->target_id) + 
                       " (distance: " + std::to_string(request->planned_distance) + "m, " +
                       "cost: " + std::to_string(request->planned_cost) + ", " +
                       "waypoints: " + std::to_string(request->path.poses.size()) + ")";

    RCLCPP_DEBUG(this->get_logger(), "%s", response->message.c_str());

  } catch (const std::exception& e) {
    response->success = false;
    response->message = std::string("Exception during edge update: ") + e.what();
    RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
  }
}

void TopomapServer::saveTopologicalMapCallback(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<navbim_msgs::srv::SaveTopologicalMap::Request> /*request*/,
  std::shared_ptr<navbim_msgs::srv::SaveTopologicalMap::Response> response)
{
  std::lock_guard<std::mutex> lock(graph_mutex_);
  
  try {
    if (topomap_file_.empty()) {
      response->success = false;
      response->message = "No topological map file path configured";
      response->file_path = "";
      RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
      return;
    }
    
    if (TopologicalMapLoader::saveToFile(topomap_file_, graph_)) {
      response->success = true;
      response->message = "Successfully saved topological map";
      response->file_path = topomap_file_;
      RCLCPP_INFO(this->get_logger(), "Saved topological map to: %s", topomap_file_.c_str());
    } else {
      response->success = false;
      response->message = "Failed to save topological map to file";
      response->file_path = topomap_file_;
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    }
    
  } catch (const std::exception& e) {
    response->success = false;
    response->message = std::string("Exception during save: ") + e.what();
    response->file_path = "";
    RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
  }
}

void TopomapServer::clearTopologicalMapPathsCallback(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<navbim_msgs::srv::ClearTopologicalMapPaths::Request> /*request*/,
  std::shared_ptr<navbim_msgs::srv::ClearTopologicalMapPaths::Response> response)
{
  std::lock_guard<std::mutex> lock(graph_mutex_);
  
  try {
    int edges_cleared = 0;
    
    // Iterate through all edges
    auto edges = boost::edges(graph_);
    for (auto eit = edges.first; eit != edges.second; ++eit) {
      auto& edge_props = graph_[*eit];
      
      // Only clear transition edges (not room or floor edges)
      if (edge_props.type != "transition") {
        continue;
      }
      
      // Skip stair transitions - they should keep their paths
      std::string subtype = edge_props.subtype.value_or("");
      if (subtype == "stair") {
        continue;
      }
      
      // Clear the path data
      edge_props.planned_distance = -1.0;
      edge_props.planned_cost = -1.0;
      edge_props.path = nullptr;  // Clear the shared_ptr
      
      edges_cleared++;
    }
    
    response->success = true;
    response->edges_cleared = edges_cleared;
    response->message = "Successfully cleared " + std::to_string(edges_cleared) + " transition edges";
    
    RCLCPP_INFO(this->get_logger(), "Cleared pre-planned paths from %d transition edges", edges_cleared);
    
  } catch (const std::exception& e) {
    response->success = false;
    response->edges_cleared = 0;
    response->message = std::string("Exception during clear: ") + e.what();
    RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
  }
}

}  // namespace navbim_topomap_server

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(navbim_topomap_server::TopomapServer)
