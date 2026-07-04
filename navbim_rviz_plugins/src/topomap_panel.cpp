#include "navbim_rviz_plugins/topomap_panel.hpp"

#include <QApplication>
#include <QGridLayout>
#include <QGroupBox>
#include <QMessageBox>
#include <sstream>
#include <iomanip>
#include <cmath>

namespace navbim_rviz_plugins
{

TopomapPanel::TopomapPanel(QWidget* parent)
: rviz_common::Panel(parent),
  show_floor_nodes_(true),
  show_room_nodes_(true),
  show_transition_nodes_(true),
  show_floor_edges_(true),
  show_room_edges_(true),
  show_transition_edges_(true),
  show_labels_(true)
{
  // Create main layout
  QHBoxLayout* groups_layout = new QHBoxLayout();
  
  // Node visibility group (left side)
  QGroupBox* node_group = new QGroupBox("Node Visibility");
  QVBoxLayout* node_layout = new QVBoxLayout();
  
  floor_nodes_checkbox_ = new QCheckBox("Floor Nodes");
  floor_nodes_checkbox_->setChecked(show_floor_nodes_);
  connect(floor_nodes_checkbox_, &QCheckBox::toggled, this, &TopomapPanel::onFloorNodesToggled);
  node_layout->addWidget(floor_nodes_checkbox_);
  
  room_nodes_checkbox_ = new QCheckBox("Room Nodes");
  room_nodes_checkbox_->setChecked(show_room_nodes_);
  connect(room_nodes_checkbox_, &QCheckBox::toggled, this, &TopomapPanel::onRoomNodesToggled);
  node_layout->addWidget(room_nodes_checkbox_);
  
  transition_nodes_checkbox_ = new QCheckBox("Transition Nodes");
  transition_nodes_checkbox_->setChecked(show_transition_nodes_);
  connect(transition_nodes_checkbox_, &QCheckBox::toggled, this, &TopomapPanel::onTransitionNodesToggled);
  node_layout->addWidget(transition_nodes_checkbox_);
  
  labels_checkbox_ = new QCheckBox("Labels");
  labels_checkbox_->setChecked(show_labels_);
  connect(labels_checkbox_, &QCheckBox::toggled, this, &TopomapPanel::onLabelsToggled);
  node_layout->addWidget(labels_checkbox_);
  
  node_group->setLayout(node_layout);
  groups_layout->addWidget(node_group);
  
  // Edge visibility group (right side)
  QGroupBox* edge_group = new QGroupBox("Edge Visibility");
  QVBoxLayout* edge_layout = new QVBoxLayout();
  
  floor_edges_checkbox_ = new QCheckBox("Floor Edges");
  floor_edges_checkbox_->setChecked(show_floor_edges_);
  connect(floor_edges_checkbox_, &QCheckBox::toggled, this, &TopomapPanel::onFloorEdgesToggled);
  edge_layout->addWidget(floor_edges_checkbox_);
  
  room_edges_checkbox_ = new QCheckBox("Room Edges");
  room_edges_checkbox_->setChecked(show_room_edges_);
  connect(room_edges_checkbox_, &QCheckBox::toggled, this, &TopomapPanel::onRoomEdgesToggled);
  edge_layout->addWidget(room_edges_checkbox_);
  
  transition_edges_checkbox_ = new QCheckBox("Transition Edges");
  transition_edges_checkbox_->setChecked(show_transition_edges_);
  connect(transition_edges_checkbox_, &QCheckBox::toggled, this, &TopomapPanel::onTransitionEdgesToggled);
  edge_layout->addWidget(transition_edges_checkbox_);
  
  edge_group->setLayout(edge_layout);
  groups_layout->addWidget(edge_group);
  
  // Main vertical layout
  QVBoxLayout* main_layout = new QVBoxLayout();
  main_layout->addLayout(groups_layout);
  
  // Status label
  status_label_ = new QLabel("Ready");
  main_layout->addWidget(status_label_);
  
  // Add stretch to push everything to the top
  main_layout->addStretch();
  
  setLayout(main_layout);
}

TopomapPanel::~TopomapPanel()
{
}

void TopomapPanel::onInitialize()
{
  // Create ROS node for the panel
  node_ = rclcpp::Node::make_shared("topomap_panel_node");
  
  // Declare and get elevate_markers parameter from topomap_server namespace
  node_->declare_parameter("topomap_server.elevate_markers", 0.1);
  elevate_markers_ = node_->get_parameter("topomap_server.elevate_markers").as_double();
  
  // Create interactive marker servers - one for full map, one for pruned map
  server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
    "topomap_interactive_markers", node_);
  
  pruned_server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
    "pruned_topomap_interactive_markers", node_);
  
  // Setup menu handlers with entries
  menu_handler_.insert("Display Info", std::bind(&TopomapPanel::processFeedback, this, std::placeholders::_1));
  menu_handler_.insert("Hide Marker", std::bind(&TopomapPanel::processFeedback, this, std::placeholders::_1));
  
  pruned_menu_handler_.insert("Display Info", std::bind(&TopomapPanel::processFeedback, this, std::placeholders::_1));
  pruned_menu_handler_.insert("Hide Marker", std::bind(&TopomapPanel::processFeedback, this, std::placeholders::_1));
  
  // Create service client for topological map
  topomap_client_ = node_->create_client<navbim_msgs::srv::GetTopologicalMap>("topomap_server/get_topological_map");
  
  // Subscribe to pruned topological map from GPP BIM
  pruned_topomap_sub_ = node_->create_subscription<navbim_msgs::msg::Topomap>(
    "/gpp_bim/pruned_topomap", rclcpp::QoS(1).transient_local(),
    [this](const navbim_msgs::msg::Topomap::SharedPtr msg) {
      pruned_topomap_ = *msg;
      createPrunedInteractiveMarkers();  // Refresh pruned markers
    });
  
  // Subscribe to floor selection from BIM panel
  floor_selection_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/bim_panel/selected_floor", 10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      selected_floor_ = msg->data;
      createInteractiveMarkers();  // Refresh full map markers for new floor
      createPrunedInteractiveMarkers();  // Refresh pruned map markers for new floor
    });
  
  // Setup timer to spin the ROS2 node (RViz panels need their own spinner)
  ros_spinner_timer_ = new QTimer(this);
  connect(ros_spinner_timer_, &QTimer::timeout, this, [this]() {
    if (node_) {
      rclcpp::spin_some(node_);
    }
  });
  ros_spinner_timer_->start(10);  // Spin every 10ms
  
  // Load topological map
  loadTopologicalMap();
  
  status_label_->setText("Initialized");
}

void TopomapPanel::onFloorNodesToggled(bool checked)
{
  show_floor_nodes_ = checked;
  createInteractiveMarkers();
  createPrunedInteractiveMarkers();
}

void TopomapPanel::onRoomNodesToggled(bool checked)
{
  show_room_nodes_ = checked;
  createInteractiveMarkers();
  createPrunedInteractiveMarkers();
}

void TopomapPanel::onTransitionNodesToggled(bool checked)
{
  show_transition_nodes_ = checked;
  createInteractiveMarkers();
  createPrunedInteractiveMarkers();
}

void TopomapPanel::onFloorEdgesToggled(bool checked)
{
  show_floor_edges_ = checked;
  createInteractiveMarkers();
  createPrunedInteractiveMarkers();
}

void TopomapPanel::onRoomEdgesToggled(bool checked)
{
  show_room_edges_ = checked;
  createInteractiveMarkers();
  createPrunedInteractiveMarkers();
}

void TopomapPanel::onTransitionEdgesToggled(bool checked)
{
  show_transition_edges_ = checked;
  createInteractiveMarkers();
  createPrunedInteractiveMarkers();
}

void TopomapPanel::onLabelsToggled(bool checked)
{
  show_labels_ = checked;
  createInteractiveMarkers();
  createPrunedInteractiveMarkers();
}

void TopomapPanel::loadTopologicalMap()
{
  if (!topomap_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(node_->get_logger(), "Topomap service not available, will retry...");
    return;
  }

  auto request = std::make_shared<navbim_msgs::srv::GetTopologicalMap::Request>();
  
  topomap_client_->async_send_request(request,
    [this](rclcpp::Client<navbim_msgs::srv::GetTopologicalMap>::SharedFuture future) {
      try {
        auto response = future.get();
        if (response->success) {
          cached_topomap_ = response->topomap;
          
          // Create markers on Qt thread
          QMetaObject::invokeMethod(this, [this]() {
            createInteractiveMarkers();
          }, Qt::QueuedConnection);
          
          RCLCPP_INFO(node_->get_logger(), "Loaded %zu nodes, %zu edges",
                      cached_topomap_.nodes.nodes.size(),
                      cached_topomap_.edges.edges.size());
        } else {
          RCLCPP_ERROR(node_->get_logger(), "Failed to load map: %s", response->message.c_str());
        }
      } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Exception loading map: %s", e.what());
      }
    });
}

void TopomapPanel::createInteractiveMarkers()
{
  if (!server_) {
    RCLCPP_ERROR(node_->get_logger(), "Server is null!");
    return;
  }
  
  server_->clear();
  
  int created_nodes = 0;
  int skipped_nodes = 0;
  
  // Create node markers
  for (const auto& node : cached_topomap_.nodes.nodes) {
    std::string type = node.type == "floor" ? "floor" :
                       node.type == "room" ? "room" : "transition";
    
    // Filter by floor selection if a specific floor is selected (not empty and not "None")
    if (!selected_floor_.empty() && selected_floor_ != "None") {
      // Check if this node belongs to the selected floor
      bool node_on_floor = false;
      // For floor nodes, check the node name directly
      if (type == "floor") {
        node_on_floor = (node.name == selected_floor_);
      } else {
        // For other nodes, check the floor array
        for (const auto& floor_name : node.floor) {
          if (floor_name == selected_floor_) {
            node_on_floor = true;
            break;
          }
        }
      }
      if (!node_on_floor) {
        continue;
      }
    }
    
    // Check visibility settings
    if ((type == "floor" && !show_floor_nodes_) ||
        (type == "room" && !show_room_nodes_) ||
        (type == "transition" && !show_transition_nodes_)) {
      skipped_nodes++;
      continue;
    }
    
    std::string name = type + "/node_" + node.id;
    
    // Check if hidden
    if (hidden_markers_.count(name) > 0) {
      skipped_nodes++;
      continue;
    }
    
    createNodeMarker(node, false);  // false = full map
    created_nodes++;
  }
  
  int created_edges = 0;
  int skipped_edges = 0;
  
  // Create edge markers
  for (const auto& edge : cached_topomap_.edges.edges) {
    std::string type = edge.type;
    
    // Filter by floor selection - check if at least one endpoint node is on the selected floor
    if (!selected_floor_.empty() && selected_floor_ != "None") {
      bool edge_on_floor = false;
      // Find source and target nodes
      for (const auto& node : cached_topomap_.nodes.nodes) {
        if (node.id == edge.source || node.id == edge.target) {
          for (const auto& floor_name : node.floor) {
            if (floor_name == selected_floor_) {
              edge_on_floor = true;
              break;
            }
          }
          if (edge_on_floor) break;
        }
      }
      if (!edge_on_floor) {
        continue;
      }
    }
    
    // Check visibility settings
    if ((type == "floor" && !show_floor_edges_) ||
        (type == "room" && !show_room_edges_) ||
        (type == "transition" && !show_transition_edges_)) {
      skipped_edges++;
      continue;
    }
    
    std::string name = type + "/edge_" + edge.id;
    
    // Check if hidden
    if (hidden_markers_.count(name) > 0) {
      skipped_edges++;
      continue;
    }
    
    createEdgeMarker(edge, false);  // false = full map
    created_edges++;
  }
  
  server_->applyChanges();
}

void TopomapPanel::createPrunedInteractiveMarkers()
{
  // Clear existing pruned markers
  pruned_server_->clear();
  
  if (pruned_topomap_.nodes.nodes.empty()) {
    pruned_server_->applyChanges();
    return;  // No pruned map available yet
  }
  
  // Create node markers for pruned map
  for (const auto& node : pruned_topomap_.nodes.nodes) {
    std::string type = node.type == "floor" ? "floor" :
                       node.type == "room" ? "room" : "transition";
    
    // Filter by floor selection if a specific floor is selected
    if (!selected_floor_.empty() && selected_floor_ != "None") {
      bool node_on_floor = false;
      // For floor nodes, check the node name directly
      if (type == "floor") {
        node_on_floor = (node.name == selected_floor_);
      } else {
        // For other nodes, check the floor array
        for (const auto& floor_name : node.floor) {
          if (floor_name == selected_floor_) {
            node_on_floor = true;
            break;
          }
        }
      }
      if (!node_on_floor) {
        continue;
      }
    }
    
    // Check visibility settings
    if ((type == "floor" && !show_floor_nodes_) ||
        (type == "room" && !show_room_nodes_) ||
        (type == "transition" && !show_transition_nodes_)) {
      continue;
    }
    
    createNodeMarker(node, true);  // true = pruned map
  }
  
  // Create edge markers for pruned map
  for (const auto& edge : pruned_topomap_.edges.edges) {
    std::string type = edge.type;
    
    // Filter by floor selection
    if (!selected_floor_.empty() && selected_floor_ != "None") {
      bool edge_on_floor = false;
      for (const auto& node : pruned_topomap_.nodes.nodes) {
        if (node.id == edge.source || node.id == edge.target) {
          for (const auto& floor_name : node.floor) {
            if (floor_name == selected_floor_) {
              edge_on_floor = true;
              break;
            }
          }
          if (edge_on_floor) break;
        }
      }
      if (!edge_on_floor) {
        continue;
      }
    }
    
    // Check visibility settings
    if ((type == "floor" && !show_floor_edges_) ||
        (type == "room" && !show_room_edges_) ||
        (type == "transition" && !show_transition_edges_)) {
      continue;
    }
    
    createEdgeMarker(edge, true);  // true = pruned map
  }
  
  pruned_server_->applyChanges();
}

void TopomapPanel::createNodeMarker(const navbim_msgs::msg::TopomapNode& node, bool is_pruned)
{
  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = "ifc";
  int_marker.header.stamp = node_->now();
  
  std::string type = node.type == "floor" ? "floor" :
                     node.type == "room" ? "room" : "transition";
  int_marker.name = type + "/node_" + node.id;
  int_marker.description = node.name;
  
  // Set position
  int_marker.pose.position.x = node.position.x;
  int_marker.pose.position.y = node.position.y;
  int_marker.pose.position.z = node.position.z + elevate_markers_;  // Elevate by parameter value
  int_marker.pose.orientation.w = 1.0;
  
  // Create sphere marker
  visualization_msgs::msg::Marker sphere_marker;
  sphere_marker.type = visualization_msgs::msg::Marker::SPHERE;
  sphere_marker.scale.x = 0.5;
  sphere_marker.scale.y = 0.5;
  sphere_marker.scale.z = 0.5;
  
  // Set color based on type 
  sphere_marker.color.a = 1.0;
  if (type == "floor") {
    // Gray for floors
    sphere_marker.color.r = 0.5f;
    sphere_marker.color.g = 0.5f;
    sphere_marker.color.b = 0.5f;
  } else if (type == "room") {
    // Blue for rooms
    sphere_marker.color.r = 0.0f;
    sphere_marker.color.g = 0.0f;
    sphere_marker.color.b = 1.0f;
  } else if (type == "transition") {
    // Green for transitions (including start and goal)
    sphere_marker.color.r = 0.0f;
    sphere_marker.color.g = 1.0f;
    sphere_marker.color.b = 0.0f;
  } else {
    // Orange for unknown
    sphere_marker.color.r = 1.0f;
    sphere_marker.color.g = 0.5f;
    sphere_marker.color.b = 0.0f;
  }
  
  // Create text marker for node name (only if labels are enabled)
  visualization_msgs::msg::InteractiveMarkerControl marker_control;
  marker_control.always_visible = true;
  marker_control.markers.push_back(sphere_marker);
  
  if (show_labels_) {
    visualization_msgs::msg::Marker text_marker;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.text = node.name;
    text_marker.scale.z = 0.3;  // Text height
    text_marker.color.r = 0.0;
    text_marker.color.g = 0.0;
    text_marker.color.b = 0.0;
    text_marker.color.a = 1.0;  // Black text
    text_marker.pose.position.z = 0.5;  // Offset above sphere
    marker_control.markers.push_back(text_marker);
  }
  
  marker_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
  int_marker.controls.push_back(marker_control);
  
  // Add menu control
  visualization_msgs::msg::InteractiveMarkerControl menu_control;
  menu_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MENU;
  menu_control.name = "menu_control";
  int_marker.controls.push_back(menu_control);
  
  // Add marker to appropriate server
  auto& marker_server = is_pruned ? pruned_server_ : server_;
  auto& marker_menu_handler = is_pruned ? pruned_menu_handler_ : menu_handler_;
  
  marker_server->insert(int_marker);
  marker_server->setCallback(int_marker.name,
    std::bind(&TopomapPanel::processFeedback, this, std::placeholders::_1));
  
  // Apply menu to this marker
  marker_menu_handler.apply(*marker_server, int_marker.name);
}

void TopomapPanel::createEdgeMarker(const navbim_msgs::msg::TopomapEdge& edge, bool is_pruned)
{
  // Use appropriate topomap based on is_pruned flag
  const auto& topomap = is_pruned ? pruned_topomap_ : cached_topomap_;
  
  // Find the nodes connected by this edge
  const navbim_msgs::msg::TopomapNode* from_node = nullptr;
  const navbim_msgs::msg::TopomapNode* to_node = nullptr;
  
  for (const auto& node : topomap.nodes.nodes) {
    if (node.id == edge.source) {
      from_node = &node;
    }
    if (node.id == edge.target) {
      to_node = &node;
    }
  }
  
  if (!from_node || !to_node) {
    return;
  }
  
  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = "ifc";
  int_marker.header.stamp = node_->now();
  
  // Determine edge type based on endpoint nodes
  std::string edge_namespace;
  if (from_node->type == "transition" && to_node->type == "transition") {
    edge_namespace = "transition";
  } else if ((from_node->type == "transition" && to_node->type == "room") ||
             (from_node->type == "room" && to_node->type == "transition")) {
    edge_namespace = "room";
  } else if (from_node->type == "floor" || to_node->type == "floor") {
    edge_namespace = "floor";
  } else {
    edge_namespace = from_node->type;  // Fallback
  }
  
  int_marker.name = edge_namespace + "/edge_" + edge.id;
  int_marker.description = "Edge: " + from_node->name + " → " + to_node->name;
  
  // Set position at midpoint
  int_marker.pose.position.x = (from_node->position.x + to_node->position.x) / 2.0;
  int_marker.pose.position.y = (from_node->position.y + to_node->position.y) / 2.0;
  int_marker.pose.position.z = (from_node->position.z + to_node->position.z) / 2.0 + elevate_markers_;  // Elevate by parameter value
  int_marker.pose.orientation.w = 1.0;
  
  // Create LINE_STRIP marker 
  visualization_msgs::msg::Marker line_marker;
  line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line_marker.scale.x = 0.1;  // Line width

  // Set color based on edge type
  line_marker.color.a = 1.0; 
  if (edge.type == "transition") {
    // Red for transitions
    line_marker.color.r = 1.0f;
    line_marker.color.g = 0.0f;
    line_marker.color.b = 0.0f;
  } else if (edge.type == "floor") {
    // Gray for floors
    line_marker.color.r = 0.5f;
    line_marker.color.g = 0.5f;
    line_marker.color.b = 0.5f;
  } else if (edge.type == "room") {
    // Yellow for rooms
    line_marker.color.r = 1.0f;
    line_marker.color.g = 1.0f;
    line_marker.color.b = 0.0f;
  } else {
    // Orange for unknown
    line_marker.color.r = 1.0f;
    line_marker.color.g = 0.5f;
    line_marker.color.b = 0.0f;
  }
  
  // Add points for the line (relative to marker pose)
  geometry_msgs::msg::Point p1, p2;
  p1.x = from_node->position.x - int_marker.pose.position.x;
  p1.y = from_node->position.y - int_marker.pose.position.y;
  p1.z = from_node->position.z - int_marker.pose.position.z;
  p2.x = to_node->position.x - int_marker.pose.position.x;
  p2.y = to_node->position.y - int_marker.pose.position.y;
  p2.z = to_node->position.z - int_marker.pose.position.z;
  line_marker.points.push_back(p1);
  line_marker.points.push_back(p2);
  
  visualization_msgs::msg::InteractiveMarkerControl marker_control;
  marker_control.always_visible = true;
  marker_control.markers.push_back(line_marker);
  marker_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
  int_marker.controls.push_back(marker_control);
  
  // Add menu control
  visualization_msgs::msg::InteractiveMarkerControl menu_control;
  menu_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MENU;
  menu_control.name = "menu_control";
  int_marker.controls.push_back(menu_control);
  
  // Add marker to appropriate server
  auto& marker_server = is_pruned ? pruned_server_ : server_;
  auto& marker_menu_handler = is_pruned ? pruned_menu_handler_ : menu_handler_;
  
  marker_server->insert(int_marker);
  marker_server->setCallback(int_marker.name,
    std::bind(&TopomapPanel::processFeedback, this, std::placeholders::_1));
  
  // Apply menu to this marker
  marker_menu_handler.apply(*marker_server, int_marker.name);
}

void TopomapPanel::processFeedback(
  const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
{
  if (feedback->event_type != visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT) {
    return;
  }
  
  std::string marker_name = feedback->marker_name;
  
  // Determine if it's a node or edge
  bool is_node = marker_name.find("/node_") != std::string::npos;
  
  switch (feedback->menu_entry_id) {
    case 1:  // Display Info
      if (is_node) {
        // Try to find the node in cached_topomap first
        bool found = false;
        for (const auto& node : cached_topomap_.nodes.nodes) {
          std::string type = node.type;
          std::string name = type + "/node_" + node.id;
          if (name == marker_name) {
            displayNodeInfo(node);
            found = true;
            break;
          }
        }
        
        // If not found, try pruned_topomap
        if (!found) {
          for (const auto& node : pruned_topomap_.nodes.nodes) {
            std::string type = node.type;
            std::string name = type + "/node_" + node.id;
            if (name == marker_name) {
              displayNodeInfo(node);
              break;
            }
          }
        }
      } else {
        // Try to find the edge in cached_topomap first
        bool found = false;
        for (const auto& edge : cached_topomap_.edges.edges) {
          std::string type = edge.type;
          std::string name = type + "/edge_" + edge.id;
          if (name == marker_name) {
            displayEdgeInfo(edge);
            found = true;
            break;
          }
        }
        
        // If not found, try pruned_topomap
        if (!found) {
          for (const auto& edge : pruned_topomap_.edges.edges) {
            std::string type = edge.type;
            std::string name = type + "/edge_" + edge.id;
            if (name == marker_name) {
              displayEdgeInfo(edge);
              break;
            }
          }
        }
      }
      break;
      
    case 2:  // Hide Marker
      hidden_markers_.insert(marker_name);
      createInteractiveMarkers();  // Refresh
      createPrunedInteractiveMarkers();  // Also refresh pruned markers
      break;
  }
}

void TopomapPanel::displayNodeInfo(const navbim_msgs::msg::TopomapNode& node)
{
  std::stringstream ss;
  
  ss << "Node ID: " << node.id << "\n";
  ss << "Name: " << node.name << "\n";
  ss << "Type: " << node.type << "\n";
  
  ss << "Position: (" << std::fixed << std::setprecision(2)
     << node.position.x << ", " << node.position.y << ", " << node.position.z << ")\n";
  
  // Calculate polygon area for room nodes
  if (node.type == "room" && !node.polygon.outer.points.empty()) {
    double area = calculatePolygonArea(node.polygon.outer.points);
    ss << "Polygon Area: " << std::fixed << std::setprecision(2) << area << " m²\n";
  }
  
  // Display in QMessageBox - use QMetaObject::invokeMethod to ensure it runs on the Qt thread
  std::string info_text = ss.str();
  
  QMetaObject::invokeMethod(qApp, [info_text]() {
    QMessageBox* msgBox = new QMessageBox();
    msgBox->setWindowTitle("Node Information");
    msgBox->setText(QString::fromStdString(info_text));
    msgBox->setIcon(QMessageBox::Information);
    msgBox->setAttribute(Qt::WA_DeleteOnClose);
    msgBox->setModal(false);
    msgBox->show();
    msgBox->raise();
    msgBox->activateWindow();
  }, Qt::QueuedConnection);
}

void TopomapPanel::displayEdgeInfo(const navbim_msgs::msg::TopomapEdge& edge)
{
  std::stringstream ss;
  
  ss << "Edge ID: " << edge.id << "\n";
  ss << "From Node: " << edge.source << "\n";
  ss << "To Node: " << edge.target << "\n";
  ss << "Type: " << edge.type << "\n";
  
  if (edge.type == "transition") {
    ss << "Estimated Cost: " << std::fixed << std::setprecision(2) << edge.estimated_cost << "\n";
    ss << "Estimated Distance: " << std::fixed << std::setprecision(2) << edge.estimated_distance << " m\n";
    
    // Display path information
    if (edge.path.poses.empty()) {
      ss << "Path: Unplanned\n";
    } else {
      ss << "Path: Planned\n";
      ss << "Planned Distance: " << std::fixed << std::setprecision(2) << edge.planned_distance << " m\n";
      ss << "Planned Cost: " << std::fixed << std::setprecision(2) << edge.planned_cost << "\n";
    }
  }
  
  // Display in QMessageBox - use QMetaObject::invokeMethod to ensure it runs on the Qt thread
  std::string info_text = ss.str();
  
  QMetaObject::invokeMethod(qApp, [info_text]() {
    QMessageBox* msgBox = new QMessageBox();
    msgBox->setWindowTitle("Edge Information");
    msgBox->setText(QString::fromStdString(info_text));
    msgBox->setIcon(QMessageBox::Information);
    msgBox->setAttribute(Qt::WA_DeleteOnClose);
    msgBox->setModal(false);
    msgBox->show();
    msgBox->raise();
    msgBox->activateWindow();
  }, Qt::QueuedConnection);
}

double TopomapPanel::calculatePolygonArea(
  const std::vector<geometry_msgs::msg::Point32>& points)
{
  if (points.size() < 3) {
    return 0.0;
  }
  
  // Shoelace formula
  double area = 0.0;
  for (size_t i = 0; i < points.size(); i++) {
    size_t j = (i + 1) % points.size();
    area += points[i].x * points[j].y;
    area -= points[j].x * points[i].y;
  }
  
  return std::abs(area) / 2.0;
}

}  // namespace navbim_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(navbim_rviz_plugins::TopomapPanel, rviz_common::Panel)
