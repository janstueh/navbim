#ifndef BIM_PANEL_H
#define BIM_PANEL_H

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rviz_common/panel.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include "navbim_msgs/srv/get_floor_nodes.hpp"
#include "navbim_rviz_plugins/ifc_element_manager.hpp"
#include <lifecycle_msgs/srv/get_state.hpp>

#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QInputDialog>
#include <QMessageBox>
#include <QTimer>

#include <map>
#include <tuple>
#include <string>
#include <vector>
#include <filesystem>

namespace navbim_rviz_plugins
{
    class BimPanel : public rviz_common::Panel
    {
    Q_OBJECT
    public:
        BimPanel(QWidget* parent = nullptr);

    public Q_SLOTS:
        void onBimModelsClicked();
        void onFloorsClicked();
        void spinRosNode(); // Slot for spinning the ROS2 node
        void tryLoadFloors(); // Slot for attempting to load floors asynchronously
    
    private:
        // functions
        void searchBimModels(const std::string& bim_path);
        void searchFloors();
        bool isLifecycleNodeActive(const std::string& node_name);
        void displayIfcElements(const std::string& navigation_model_path, const std::string& model, const std::string& floor);
        void startPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
        void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        // variables
        std::string navigation_model_filepath;
        QStringList bim_models;
        QStringList floors;
        std::map<std::string, std::pair<double, double>> floor_heights; // floor_name -> (min_z, max_z)
        std::string selected_model;
        std::string selected_floor;
        float elevate_start_goal = 0.1;
        // UI components
        QVBoxLayout* layout;
        QPushButton* bim_models_button;
        QLabel* bim_model_status_label;
        QPushButton* floors_button;
        QLabel* floor_status_label;
        // ROS2
        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<IfcElementManager> ifc_element_manager_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr selected_floor_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr start_pose_pub;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub;
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr start_pose_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub;
        rclcpp::Client<navbim_msgs::srv::GetFloorNodes>::SharedPtr floor_nodes_client_;
        rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr topomap_state_client_;
        QTimer* ros_spinner_timer_;
        QTimer* floor_loader_timer_;
        bool floors_loaded_{false};
    };
} // namespace navbim_rviz_plugins

#endif // BIM_PANEL_H
