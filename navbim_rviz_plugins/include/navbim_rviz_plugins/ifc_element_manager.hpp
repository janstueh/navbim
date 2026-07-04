#ifndef IFC_ELEMENT_MANAGER_HPP
#define IFC_ELEMENT_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <navbim_msgs/srv/get_ifc_element_info.hpp>
#include <navbim_msgs/srv/get_elements_by_type.hpp>
#include <navbim_msgs/action/navbim_compute_path_to_pose.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <optional>
#include <unordered_map>
#include <memory>
#include <string>
#include <vector>
#include <filesystem>

namespace navbim_rviz_plugins
{

/**
 * @brief Manages interactive markers for IFC elements with right-click menu functionality
 * 
 * This class handles the display and interaction of individual IFC elements as interactive
 * markers in RViz. It provides context menu options for controlling visibility and
 * transparency of elements.
 */
class IfcElementManager
{
public:
    /**
     * @brief Constructor
     * @param node Shared pointer to the ROS2 node
     */
    explicit IfcElementManager(rclcpp::Node::SharedPtr node);
    
    /**
     * @brief Destructor
     */
    ~IfcElementManager();

    /**
     * @brief Display IFC elements as interactive markers
     * @param mesh_files Vector of absolute paths to mesh files
     * @param frame_id Frame ID for the markers (default: "ifc")
     */
    void displayElements(const std::vector<std::string>& mesh_files, 
                        const std::string& frame_id = "ifc");

    /**
     * @brief Clear all interactive markers
     */
    void clearAllElements();

    /**
     * @brief Hide specific elements by their GUID
     * @param guids Vector of element GUIDs to hide
     */
    void hideElementsByGuid(const std::vector<std::string>& guids);

    /**
     * @brief Check if an element is currently visible
     * @param guid Element GUID
     * @return True if element is visible, false otherwise
     */
    bool isElementVisible(const std::string& guid) const;

    /**
     * @brief Get transparency level of an element
     * @param guid Element GUID
     * @return Alpha value (0.0 = transparent, 1.0 = opaque)
     */
    float getElementTransparency(const std::string& guid) const;

    /**
     * @brief Get list of currently visible element GUIDs
     * @return Vector of visible element GUIDs
     */
    std::vector<std::string> getVisibleElements() const;

    /**
     * @brief Set transparency for a specific element
     * @param guid Element GUID
     * @param alpha Alpha value (0.0 = transparent, 1.0 = opaque)
     */
    void setElementTransparency(const std::string& guid, float alpha);

    /**
     * @brief Set visibility for a specific element
     * @param guid Element GUID
     * @param visible True to show, false to hide
     */
    void setElementVisibility(const std::string& guid, bool visible);

    /**
     * @brief Register this instance as the global IfcElementManager (static registry)
     */
    void registerInstance();

    /**
     * @brief Get the global IfcElementManager instance
     * @return Pointer to the registered instance, or nullptr if not registered
     */
    static IfcElementManager* getInstance();

private:
    static IfcElementManager* instance_;  ///< Static instance for registry
    /**
     * @brief Structure to hold element state information
     */
    struct ElementState {
        std::string guid;                                    ///< Element GUID
        std::string mesh_file;                              ///< Path to mesh file
        bool visible = true;                                ///< Visibility state
        float alpha = 1.0f;                                ///< Transparency level
        visualization_msgs::msg::Marker original_marker;   ///< Original marker configuration
        std::string marker_name;                           ///< Interactive marker name
    };

    // ROS2 components
    rclcpp::Node::SharedPtr node_;                          ///< ROS2 node
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> marker_server_;  ///< Interactive marker server
    interactive_markers::MenuHandler menu_handler_;        ///< Context menu handler
    rclcpp::Client<navbim_msgs::srv::GetIfcElementInfo>::SharedPtr element_info_client_;  ///< Service client for element info
    rclcpp::Client<navbim_msgs::srv::GetElementsByType>::SharedPtr elements_by_type_client_;  ///< Service client for querying elements by type
    rclcpp_action::Client<navbim_msgs::action::NavbimComputePathToPose>::SharedPtr compute_path_client_;  ///< Action client for path planning
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;            ///< TF2 buffer
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;  ///< TF2 listener

    // State management
    std::unordered_map<std::string, ElementState> element_states_;  ///< Element state storage
    std::string frame_id_;                                  ///< Current frame ID
    int next_marker_id_;                                   ///< Next available marker ID
    // Menu item handles
    interactive_markers::MenuHandler::EntryHandle make_transparent_handle_;
    interactive_markers::MenuHandler::EntryHandle hide_element_handle_;
    interactive_markers::MenuHandler::EntryHandle display_info_handle_;

    /**
     * @brief Initialize the context menu
     */
    void setupMenuHandler();

    /**
     * @brief Callback for interactive marker menu selections
     * @param feedback Feedback message containing menu selection
     */
    void onElementMenuSelect(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback);

    /**
     * @brief Set element to transparent (alpha = 0.5)
     * @param element_guid Element GUID
     */
    void makeTransparent(const std::string& element_guid);

    /**
     * @brief Hide an element completely
     * @param element_guid Element GUID
     */
    void hideElement(const std::string& element_guid);

    /**
     * @brief Display IFC info dialog
     * @param element_guid Element GUID
     */
    void displayIfcInfo(const std::string& element_guid);


    /**
     * @brief Get current robot pose in the specified frame
     * @param frame_id Target frame ID
     * @param robot_frame Robot base frame (default: "base_link")
     * @return Current robot pose, or nullopt if the TF lookup fails
     */
    std::optional<geometry_msgs::msg::PoseStamped> getCurrentRobotPose(
        const std::string& frame_id,
        const std::string& robot_frame = "base_link") const;

    /**
     * @brief Send path planning goal to action server
     * @param goal_pose Goal pose for path planning
     * @param planner_id Planner ID to use (optional, defaults to "GridBased")
     */
    void sendPlanningGoal(
        const geometry_msgs::msg::PoseStamped& goal_pose,
        const std::string& planner_id = "GridBased");

    /**
     * @brief Extract GUID from filename
     * @param filepath Full path to mesh file
     * @return Extracted GUID (filename stem)
     */
    std::string extractGuidFromFilename(const std::string& filepath) const;

    /**
     * @brief Create an interactive marker for a mesh file
     * @param mesh_file Path to mesh file
     * @return Created interactive marker
     */
    visualization_msgs::msg::InteractiveMarker createInteractiveMarker(const std::string& mesh_file);

    /**
     * @brief Create a mesh marker from file path
     * @param mesh_file Path to mesh file
     * @param marker_id Unique marker ID
     * @return Configured mesh marker
     */
    visualization_msgs::msg::Marker createMeshMarker(const std::string& mesh_file, int marker_id);

    /**
     * @brief Update the display of an element after state change
     * @param element_guid Element GUID
     */
    void updateElementDisplay(const std::string& element_guid);

};

} // namespace navbim_rviz_plugins

#endif // IFC_ELEMENT_MANAGER_HPP