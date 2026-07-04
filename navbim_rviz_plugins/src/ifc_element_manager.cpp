#include "navbim_rviz_plugins/ifc_element_manager.hpp"
#include <rclcpp/logging.hpp>
#include <QMessageBox>
#include <QApplication>
#include <thread>
#include <chrono>
#include <cmath>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace navbim_rviz_plugins {

// Static instance initialization
IfcElementManager* IfcElementManager::instance_ = nullptr;

IfcElementManager::IfcElementManager(rclcpp::Node::SharedPtr node)
    : node_(node)
    , frame_id_("ifc")
    , next_marker_id_(0)
{
    // Initialize interactive marker server
    marker_server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
        "ifc_interactive_markers", node_);
    
    // Initialize service client for element info
    element_info_client_ = node_->create_client<navbim_msgs::srv::GetIfcElementInfo>(
        "bim_server/get_element_info");
    
    // Initialize service client for querying elements by type
    elements_by_type_client_ = node_->create_client<navbim_msgs::srv::GetElementsByType>(
        "bim_server/get_elements_by_type");
    
    // Initialize action client for path planning
    compute_path_client_ = rclcpp_action::create_client<navbim_msgs::action::NavbimComputePathToPose>(
        node_, "navbim_compute_path_to_pose");
    
    // Initialize TF2 buffer and listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Setup context menu
    setupMenuHandler();
    
    RCLCPP_INFO(node_->get_logger(), "IfcElementManager initialized");
}

IfcElementManager::~IfcElementManager()
{
    if (marker_server_) {
        marker_server_->clear();
        marker_server_->applyChanges();
    }
}

void IfcElementManager::setupMenuHandler()
{
    // Create context menu entries
    make_transparent_handle_ = menu_handler_.insert("Make Transparent (50%)",
        std::bind(&IfcElementManager::onElementMenuSelect, this, std::placeholders::_1));
    
    hide_element_handle_ = menu_handler_.insert("Hide Element",
        std::bind(&IfcElementManager::onElementMenuSelect, this, std::placeholders::_1));
    
    display_info_handle_ = menu_handler_.insert("Display IFC Info",
        std::bind(&IfcElementManager::onElementMenuSelect, this, std::placeholders::_1));
    
}

void IfcElementManager::displayElements(const std::vector<std::string>& mesh_files, 
                                       const std::string& frame_id)
{
    frame_id_ = frame_id;
    
    // Clear existing elements
    clearAllElements();
    
    RCLCPP_INFO(node_->get_logger(), "Displaying %zu IFC elements as interactive markers", mesh_files.size());
    
    // Create interactive markers for each mesh file
    for (const auto& mesh_file : mesh_files) {
        if (!std::filesystem::exists(mesh_file)) {
            RCLCPP_WARN(node_->get_logger(), "Mesh file does not exist: %s", mesh_file.c_str());
            continue;
        }
        
        try {
            auto interactive_marker = createInteractiveMarker(mesh_file);
            std::string guid = extractGuidFromFilename(mesh_file);
            
            // Store element state
            ElementState state;
            state.guid = guid;
            state.mesh_file = mesh_file;
            state.visible = true;
            state.alpha = 1.0f;
            state.marker_name = interactive_marker.name;
            state.original_marker = interactive_marker.controls[0].markers[0];
            
            element_states_[guid] = state;
            
            // Add to interactive marker server
            marker_server_->insert(interactive_marker);
            menu_handler_.apply(*marker_server_, interactive_marker.name);
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to create interactive marker for %s: %s", 
                        mesh_file.c_str(), e.what());
        }
    }
    
    marker_server_->applyChanges();
    RCLCPP_INFO(node_->get_logger(), "Successfully created %zu interactive markers", element_states_.size());
}

void IfcElementManager::clearAllElements()
{
    marker_server_->clear();
    element_states_.clear();
    next_marker_id_ = 0;
    marker_server_->applyChanges();
}

void IfcElementManager::hideElementsByGuid(const std::vector<std::string>& guids)
{
    for (const auto& guid : guids) {
        setElementVisibility(guid, false);
    }
}

bool IfcElementManager::isElementVisible(const std::string& guid) const
{
    auto it = element_states_.find(guid);
    return (it != element_states_.end()) ? it->second.visible : false;
}

float IfcElementManager::getElementTransparency(const std::string& guid) const
{
    auto it = element_states_.find(guid);
    return (it != element_states_.end()) ? it->second.alpha : 1.0f;
}

std::vector<std::string> IfcElementManager::getVisibleElements() const
{
    std::vector<std::string> visible_elements;
    for (const auto& [guid, state] : element_states_) {
        if (state.visible) {
            visible_elements.push_back(guid);
        }
    }
    return visible_elements;
}

void IfcElementManager::setElementTransparency(const std::string& guid, float alpha)
{
    auto it = element_states_.find(guid);
    if (it != element_states_.end()) {
        it->second.alpha = std::clamp(alpha, 0.0f, 1.0f);
        updateElementDisplay(guid);
    } else {
        RCLCPP_WARN(node_->get_logger(), "Cannot set transparency: Element %s not found in state map", guid.c_str());
    }
}

void IfcElementManager::setElementVisibility(const std::string& guid, bool visible)
{
    auto it = element_states_.find(guid);
    if (it != element_states_.end()) {
        it->second.visible = visible;
        updateElementDisplay(guid);
    } else {
        RCLCPP_WARN(node_->get_logger(), "Cannot set visibility: Element %s not found in state map", guid.c_str());
    }
}

void IfcElementManager::registerInstance()
{
    instance_ = this;
    RCLCPP_INFO(node_->get_logger(), "IFC Element Manager registered as global instance");
}

IfcElementManager* IfcElementManager::getInstance()
{
    return instance_;
}

void IfcElementManager::onElementMenuSelect(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
{
    std::string marker_name = feedback->marker_name;
    
    // Get the interactive marker to extract GUID from description
    visualization_msgs::msg::InteractiveMarker int_marker;
    if (!marker_server_->get(marker_name, int_marker)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to get interactive marker: %s", marker_name.c_str());
        return;
    }
    
    // Extract GUID directly from the description field
    std::string guid = int_marker.description;
    
    if (guid.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "Empty GUID in marker description for: %s", marker_name.c_str());
        return;
    }
    
    if (feedback->menu_entry_id == make_transparent_handle_) {
        makeTransparent(guid);
    } else if (feedback->menu_entry_id == hide_element_handle_) {
        hideElement(guid);
    } else if (feedback->menu_entry_id == display_info_handle_) {
        displayIfcInfo(guid);
    }
}

void IfcElementManager::makeTransparent(const std::string& element_guid)
{
    setElementTransparency(element_guid, 0.5f);
    RCLCPP_INFO(node_->get_logger(), "Set element %s to transparent (50%%)", element_guid.c_str());
}

void IfcElementManager::hideElement(const std::string& element_guid)
{
    setElementVisibility(element_guid, false);
    RCLCPP_INFO(node_->get_logger(), "Hidden element %s", element_guid.c_str());
}

void IfcElementManager::displayIfcInfo(const std::string& element_guid)
{
    RCLCPP_INFO(node_->get_logger(), "Requesting IFC info for element %s", element_guid.c_str());
    
    if (!element_info_client_) {
        RCLCPP_ERROR(node_->get_logger(), "Element info service client not initialized");
        QMetaObject::invokeMethod(qApp, []() {
            QMessageBox::warning(nullptr, "Service Error", "Element info service client not initialized.");
        }, Qt::QueuedConnection);
        return;
    }

    if (!element_info_client_->service_is_ready()) {
        RCLCPP_WARN(node_->get_logger(), "BIM server service not available");
        QMetaObject::invokeMethod(qApp, []() {
            QMessageBox::warning(nullptr, "Service Unavailable",
                "BIM server is not available.\n\nPlease ensure the BIM server is running.");
        }, Qt::QueuedConnection);
        return;
    }

    auto request = std::make_shared<navbim_msgs::srv::GetIfcElementInfo::Request>();
    request->guid = element_guid;

    // Send async request with callback
    auto response_callback = [this, element_guid](
        rclcpp::Client<navbim_msgs::srv::GetIfcElementInfo>::SharedFuture future) {
        try {
            auto response = future.get();
            
            if (response->success) {
                RCLCPP_INFO(node_->get_logger(),
                    "Element Info - GUID: %s, Type: %s, Name: %s, Origin: (%.2f, %.2f, %.2f)",
                    element_guid.c_str(),
                    response->element_type.c_str(),
                    response->element_name.c_str(),
                    response->pose.position.x,
                    response->pose.position.y,
                    response->pose.position.z);
                
                // Show info dialog on Qt main thread
                std::string guid = element_guid;
                std::string elem_type = response->element_type;
                std::string elem_name = response->element_name;
                double x = response->pose.position.x;
                double y = response->pose.position.y;
                double z = response->pose.position.z;
                
                QMetaObject::invokeMethod(qApp, [guid, elem_type, elem_name, x, y, z]() {
                    QString info_text = "GUID: " + QString::fromStdString(guid) + "\n"
                        + "Type: " + QString::fromStdString(elem_type) + "\n"
                        + "Name: " + QString::fromStdString(elem_name) + "\n"
                        + QString("Origin: (%1, %2, %3)")
                            .arg(x, 0, 'f', 2)
                            .arg(y, 0, 'f', 2)
                            .arg(z, 0, 'f', 2);
                    QMessageBox::information(nullptr, "IFC Element Information", info_text);
                }, Qt::QueuedConnection);
            } else {
                RCLCPP_WARN(node_->get_logger(), "Failed to get element info: %s", 
                    response->message.c_str());
                
                std::string msg = response->message;
                QMetaObject::invokeMethod(qApp, [msg]() {
                    QMessageBox::warning(nullptr, "Element Not Found", QString::fromStdString(msg));
                }, Qt::QueuedConnection);
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Exception in service callback: %s", e.what());
            
            std::string error = e.what();
            QMetaObject::invokeMethod(qApp, [error]() {
                QMessageBox::warning(nullptr, "Service Error",
                    QString("Failed to retrieve element information: %1").arg(QString::fromStdString(error)));
            }, Qt::QueuedConnection);
        }
    };

    element_info_client_->async_send_request(request, response_callback);
}

std::string IfcElementManager::extractGuidFromFilename(const std::string& filepath) const
{
    std::filesystem::path file_path(filepath);
    return file_path.stem().string();
}

visualization_msgs::msg::InteractiveMarker IfcElementManager::createInteractiveMarker(const std::string& mesh_file)
{
    std::string guid = extractGuidFromFilename(mesh_file);
    
    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = frame_id_;
    int_marker.header.stamp = node_->get_clock()->now();

    int_marker.name = "ifc_element_" + std::to_string(next_marker_id_);
    // Store the original GUID in the description field
    int_marker.description = guid;
    
    // Set pose to origin (individual IFC elements should have their own transforms in the mesh)
    int_marker.pose.position.x = 0.0;
    int_marker.pose.position.y = 0.0;
    int_marker.pose.position.z = 0.0;
    int_marker.pose.orientation.w = 1.0;
    
    // Set scale for the interactive marker
    int_marker.scale = 1.0;
    
    // Create mesh marker
    auto mesh_marker = createMeshMarker(mesh_file, next_marker_id_++);
    
    // Create interactive marker control with right-click menu interaction
    visualization_msgs::msg::InteractiveMarkerControl control;
    control.always_visible = true;
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
    control.markers.push_back(mesh_marker);
    
    int_marker.controls.push_back(control);
    
    return int_marker;
}

visualization_msgs::msg::Marker IfcElementManager::createMeshMarker(const std::string& mesh_file, int marker_id)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = node_->get_clock()->now();
    marker.id = marker_id;
    marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    // Set mesh resource
    marker.mesh_resource = "file://" + mesh_file;
    marker.mesh_use_embedded_materials = true;
    
    // Set scale
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    
    // Set pose
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.w = 1.0;
    
    // Set color (default: white, fully opaque)
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    
    return marker;
}

void IfcElementManager::updateElementDisplay(const std::string& element_guid)
{
    auto it = element_states_.find(element_guid);
    if (it == element_states_.end()) {
        RCLCPP_WARN(node_->get_logger(), "Attempted to update non-existent element: %s", element_guid.c_str());
        return;
    }
    
    const auto& state = it->second;
    
    visualization_msgs::msg::InteractiveMarker int_marker;
    if (!marker_server_->get(state.marker_name, int_marker)) {
        RCLCPP_WARN(node_->get_logger(), "Failed to get interactive marker: %s", state.marker_name.c_str());
        return;
    }
    
    if (state.visible) {
        // Update the marker with current transparency
        if (!int_marker.controls.empty() && !int_marker.controls[0].markers.empty()) {
            auto& marker = int_marker.controls[0].markers[0];
            marker = state.original_marker;
            marker.color.a = state.alpha;
            marker.action = visualization_msgs::msg::Marker::ADD;
        }
    } else {
        // Hide the marker by clearing markers or setting action to DELETE
        if (!int_marker.controls.empty()) {
            int_marker.controls[0].markers.clear();
        }
    }
    
    // Update the marker server
    marker_server_->insert(int_marker);
    marker_server_->applyChanges();
}

std::optional<geometry_msgs::msg::PoseStamped> IfcElementManager::getCurrentRobotPose(
    const std::string& frame_id,
    const std::string& robot_frame) const
{
    try {
        geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
            frame_id, robot_frame, tf2::TimePointZero);

        geometry_msgs::msg::PoseStamped robot_pose;
        robot_pose.header.frame_id = frame_id;
        robot_pose.header.stamp = node_->get_clock()->now();
        robot_pose.pose.position.x = transform.transform.translation.x;
        robot_pose.pose.position.y = transform.transform.translation.y;
        robot_pose.pose.position.z = transform.transform.translation.z;
        robot_pose.pose.orientation = transform.transform.rotation;

        RCLCPP_DEBUG(node_->get_logger(),
                    "Got robot pose: (%.2f, %.2f, %.2f) in frame '%s'",
                    robot_pose.pose.position.x,
                    robot_pose.pose.position.y,
                    robot_pose.pose.position.z,
                    frame_id.c_str());
        return robot_pose;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(node_->get_logger(),
                   "Could not get robot pose in frame '%s': %s",
                   frame_id.c_str(), ex.what());
        return std::nullopt;
    }
}

void IfcElementManager::sendPlanningGoal(
    const geometry_msgs::msg::PoseStamped& goal_pose,
    const std::string& planner_id)
{
    if (!compute_path_client_) {
        RCLCPP_ERROR(node_->get_logger(), "Path planning action client not initialized");
        return;
    }
    
    // Wait for action server to be available
    if (!compute_path_client_->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node_->get_logger(), "Path planning action server 'navbim_compute_path_to_pose' not available");
        QMetaObject::invokeMethod(qApp, []() {
            QMessageBox::warning(nullptr, "Planning Server Unavailable",
                "Path planning action server 'navbim_compute_path_to_pose' is not available.\n\n"
                "Please ensure the gpp_bim server is running.");
        }, Qt::QueuedConnection);
        return;
    }
    
    // Get current robot pose as start
    auto start_pose_opt = getCurrentRobotPose(goal_pose.header.frame_id);
    if (!start_pose_opt) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to get current robot pose for planning");
        QMetaObject::invokeMethod(qApp, []() {
            QMessageBox::warning(nullptr, "TF Error",
                "Failed to get current robot pose.\n\n"
                "Please ensure TF is publishing the robot's base_link frame.");
        }, Qt::QueuedConnection);
        return;
    }

    // Prepare action goal
    navbim_msgs::action::NavbimComputePathToPose::Goal goal;
    goal.goal = goal_pose;
    goal.start = *start_pose_opt;
    goal.planner_id = planner_id;
    goal.use_start = true;  // Provide explicit start pose
    
    RCLCPP_INFO(node_->get_logger(), 
                "Sending path planning goal to (%.2f, %.2f, %.2f) with yaw %.2f using planner '%s'",
                goal_pose.pose.position.x,
                goal_pose.pose.position.y,
                goal_pose.pose.position.z,
                2.0 * std::atan2(goal_pose.pose.orientation.z, goal_pose.pose.orientation.w),
                planner_id.c_str());
    
    // Send goal options
    auto send_goal_options = rclcpp_action::Client<navbim_msgs::action::NavbimComputePathToPose>::SendGoalOptions();
    
    // Goal response callback
    send_goal_options.goal_response_callback = 
        [this](std::shared_ptr<rclcpp_action::ClientGoalHandle<navbim_msgs::action::NavbimComputePathToPose>> goal_handle) {
            if (!goal_handle) {
                RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
                QMetaObject::invokeMethod(qApp, []() {
                    QMessageBox::warning(nullptr, "Goal Rejected",
                        "Path planning goal was rejected by the server.");
                }, Qt::QueuedConnection);
            } else {
                RCLCPP_INFO(node_->get_logger(), "Goal accepted by server, planning in progress...");
            }
        };
    
    // Result callback
    send_goal_options.result_callback = 
        [this](const rclcpp_action::ClientGoalHandle<navbim_msgs::action::NavbimComputePathToPose>::WrappedResult& result) {
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    if (result.result->error_code == navbim_msgs::action::NavbimComputePathToPose::Result::NONE) {
                        RCLCPP_INFO(node_->get_logger(), "Path planning succeeded! Path has %zu waypoints",
                                    result.result->path.poses.size());
                        QMetaObject::invokeMethod(qApp, [waypoints = result.result->path.poses.size()]() {
                            QMessageBox::information(nullptr, "Planning Complete",
                                QString("Inspection path planning completed successfully!\n\nPath contains %1 waypoints.")
                                .arg(waypoints));
                        }, Qt::QueuedConnection);
                    } else {
                        RCLCPP_ERROR(node_->get_logger(), "Path planning failed with error code %d: %s",
                                    result.result->error_code, result.result->error_msg.c_str());
                        QMetaObject::invokeMethod(qApp, [error_msg = result.result->error_msg]() {
                            QMessageBox::warning(nullptr, "Planning Failed",
                                QString("Path planning failed: %1").arg(QString::fromStdString(error_msg)));
                        }, Qt::QueuedConnection);
                    }
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(node_->get_logger(), "Path planning goal was aborted");
                    QMetaObject::invokeMethod(qApp, []() {
                        QMessageBox::warning(nullptr, "Planning Aborted",
                            "Inspection path planning was aborted.");
                    }, Qt::QueuedConnection);
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_WARN(node_->get_logger(), "Path planning goal was canceled");
                    break;
                default:
                    RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
                    break;
            }
        };
    
    // Feedback callback (optional)
    send_goal_options.feedback_callback = 
        [this](rclcpp_action::ClientGoalHandle<navbim_msgs::action::NavbimComputePathToPose>::SharedPtr,
               const std::shared_ptr<const navbim_msgs::action::NavbimComputePathToPose::Feedback> feedback) {
            // NavbimComputePathToPose doesn't have feedback, but we keep this for consistency
            (void)feedback;  // Suppress unused parameter warning
        };
    
    // Send the goal asynchronously
    compute_path_client_->async_send_goal(goal, send_goal_options);
}

} // namespace navbim_rviz_plugins