#ifndef NAVBIM_RVIZ_PLUGINS__TOPOMAP_PANEL_HPP_
#define NAVBIM_RVIZ_PLUGINS__TOPOMAP_PANEL_HPP_

#include <QCheckBox>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/string.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <navbim_msgs/srv/get_topological_map.hpp>
#include <navbim_msgs/msg/topomap.hpp>
#include <navbim_msgs/msg/topomap_node.hpp>
#include <navbim_msgs/msg/topomap_edge.hpp>

#include <memory>
#include <set>
#include <vector>

namespace navbim_rviz_plugins
{

/**
 * @brief Panel for controlling topological map visualization
 * 
 * Provides checkboxes to toggle visibility of different node and edge types:
 * - Floor nodes/edges
 * - Room nodes/edges  
 * - Transition nodes/edges
 */
class TopomapPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit TopomapPanel(QWidget* parent = nullptr);
  ~TopomapPanel() override;

  void onInitialize() override;

private Q_SLOTS:
  void onFloorNodesToggled(bool checked);
  void onRoomNodesToggled(bool checked);
  void onTransitionNodesToggled(bool checked);
  void onFloorEdgesToggled(bool checked);
  void onRoomEdgesToggled(bool checked);
  void onTransitionEdgesToggled(bool checked);
  void onLabelsToggled(bool checked);

private:
  void loadTopologicalMap();
  void createInteractiveMarkers();
  void createPrunedInteractiveMarkers();
  void createNodeMarker(const navbim_msgs::msg::TopomapNode& node, bool is_pruned);
  void createEdgeMarker(const navbim_msgs::msg::TopomapEdge& edge, bool is_pruned);
  void processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback);
  void displayNodeInfo(const navbim_msgs::msg::TopomapNode& node);
  void displayEdgeInfo(const navbim_msgs::msg::TopomapEdge& edge);
  double calculatePolygonArea(const std::vector<geometry_msgs::msg::Point32>& points);

  // ROS node
  rclcpp::Node::SharedPtr node_;
  
  // Interactive marker server
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> pruned_server_;
  
  // Menu handler for interactive markers
  interactive_markers::MenuHandler menu_handler_;
  interactive_markers::MenuHandler pruned_menu_handler_;
  
  // Service client for topological map
  rclcpp::Client<navbim_msgs::srv::GetTopologicalMap>::SharedPtr topomap_client_;
  
  // Cached topological maps
  navbim_msgs::msg::Topomap cached_topomap_;
  navbim_msgs::msg::Topomap pruned_topomap_;
  
  // Subscription for pruned topological map
  rclcpp::Subscription<navbim_msgs::msg::Topomap>::SharedPtr pruned_topomap_sub_;
  
  // Floor selection subscription
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr floor_selection_sub_;
  std::string selected_floor_;
  
  // Hidden markers
  std::set<std::string> hidden_markers_;
  
  // QTimer for spinning the ROS node
  QTimer* ros_spinner_timer_;
  
  // UI Elements
  QCheckBox* floor_nodes_checkbox_;
  QCheckBox* room_nodes_checkbox_;
  QCheckBox* transition_nodes_checkbox_;
  QCheckBox* floor_edges_checkbox_;
  QCheckBox* room_edges_checkbox_;
  QCheckBox* transition_edges_checkbox_;
  QCheckBox* labels_checkbox_;
  QLabel* status_label_;
  
  // Visibility state
  bool show_floor_nodes_;
  bool show_room_nodes_;
  bool show_transition_nodes_;
  bool show_floor_edges_;
  bool show_room_edges_;
  bool show_transition_edges_;
  bool show_labels_;
  
  // Marker elevation
  double elevate_markers_;
};

}  // namespace navbim_rviz_plugins

#endif  // NAVBIM_RVIZ_PLUGINS__TOPOMAP_PANEL_HPP_
