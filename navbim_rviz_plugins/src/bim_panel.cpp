#include "navbim_rviz_plugins/bim_panel.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <algorithm>
#include <cctype>
#include <sstream>
#include <filesystem>
#include <thread>
#include <chrono>
#include <QPointer>

namespace navbim_rviz_plugins
{
    BimPanel::BimPanel(QWidget* parent) : rviz_common::Panel(parent)
    {
        // Initialize ROS2 node
        node_ = rclcpp::Node::make_shared("navbim_rviz_plugins");
        
        // Initialize IFC element manager
        ifc_element_manager_ = std::make_shared<IfcElementManager>(node_);
        ifc_element_manager_->registerInstance();
        
        // Create layout
        layout = new QVBoxLayout;
        
        // Create UI components
        bim_models_button = new QPushButton("Select BIM model", this);
        bim_model_status_label = new QLabel("No BIM model selected", this);
        floors_button = new QPushButton("Select floor", this);
        floor_status_label = new QLabel("No floor selected", this);

        // Connect signals
        connect(bim_models_button, &QPushButton::clicked, this, &BimPanel::onBimModelsClicked);
        connect(floors_button, &QPushButton::clicked, this, &BimPanel::onFloorsClicked);

        // Add widgets to layout
        layout->addWidget(bim_models_button);
        layout->addWidget(bim_model_status_label);
        layout->addWidget(floors_button);
        layout->addWidget(floor_status_label);
        
        // Set layout
        setLayout(layout);

        // Get parameters
        node_->declare_parameter("nav_model", "");
        node_->declare_parameter("elevate_start_goal", 0.0);

        navigation_model_filepath = node_->get_parameter("nav_model").as_string();
        elevate_start_goal = node_->get_parameter("elevate_start_goal").as_double();

        start_pose_pub = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/navbim/initialpose", 1);
        goal_pose_pub = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/navbim/goal", 1);
        
        // Create publisher for selected floor
        selected_floor_pub_ = node_->create_publisher<std_msgs::msg::String>("/bim_panel/selected_floor", 10);

        // Initialize the service client for floor nodes
        floor_nodes_client_ = node_->create_client<navbim_msgs::srv::GetFloorNodes>("topomap_server/get_floor_nodes");
        
        // Initialize the service client for TopomapServer lifecycle state
        topomap_state_client_ = node_->create_client<lifecycle_msgs::srv::GetState>("topomap_server/get_state");

        // Search for BIM models
        std::string bim_path = ament_index_cpp::get_package_share_directory("navbim_bringup") + "/bim";
        searchBimModels(bim_path);

        // Strip the model from the nav_model path
        if (!navigation_model_filepath.empty()) {
            selected_model = navigation_model_filepath.substr(navigation_model_filepath.find_last_of("/\\") + 1);
        }
        if (!selected_model.empty()) {
            QString qBimModel = QString::fromStdString(selected_model);
            if (bim_models.contains(qBimModel)) {
                // Model exists in the list
                std::string navigation_model_path = ament_index_cpp::get_package_share_directory("navbim_bringup") + "/nav_model/" + selected_model;
                bim_model_status_label->setText("Selected model: " + qBimModel);
                // Display mesh of whole building
                displayIfcElements(navigation_model_path, selected_model, selected_model);
            } else {
                // Model doesn't exist in the list
                QMessageBox::warning(this, "Unknown BIM Model", 
                    "The specified BIM model '" + qBimModel + "' was not found.");
            }
        }

        // Subscribe to RViz pose topics
        start_pose_sub = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 1, 
            std::bind(&BimPanel::startPoseCallback, this, std::placeholders::_1));
        goal_pose_sub = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 1, 
            std::bind(&BimPanel::goalPoseCallback, this, std::placeholders::_1));
        
        // Setup timer to spin the ROS2 node for interactive marker server communication
        ros_spinner_timer_ = new QTimer(this);
        connect(ros_spinner_timer_, &QTimer::timeout, this, &BimPanel::spinRosNode);
        ros_spinner_timer_->start(10); // Spin every 10ms for responsive interactive markers
        
        // Setup timer to asynchronously load floors when topomap_server is ready
        if (!selected_model.empty()) {
            floor_status_label->setText("Loading floors...");
            floor_loader_timer_ = new QTimer(this);
            connect(floor_loader_timer_, &QTimer::timeout, this, &BimPanel::tryLoadFloors);
            floor_loader_timer_->start(500); // Check every 500ms
        }
    }

    void BimPanel::onBimModelsClicked()
    {
        if (bim_models.empty()) {
            QMessageBox::warning(this, "No BIM Models", "No BIM models found in the package directory.");
            return;
        }

        bool ok;        
        QString selected = QInputDialog::getItem(this, 
                                            "Select BIM Model", 
                                            "Available models:", 
                                            bim_models, 
                                            0,      // Default selection index
                                            false,  // Non-editable
                                            &ok);
        
        if (ok && !selected.isEmpty()) {
            selected_model = selected.toStdString();
            std::string navigation_model_path = ament_index_cpp::get_package_share_directory("navbim_bringup") + "/nav_model/" + selected_model;
            if (selected_model == "None") {
                // Reset the parameters if "None" is selected
                selected_model = ""; 
                selected_floor = ""; 
                bim_model_status_label->setText("No BIM model selected");
                floor_status_label->setText("No floor selected");
            // Check if the folder exists and if it's empty
            } else if (!std::filesystem::exists(navigation_model_path) || std::filesystem::is_empty(navigation_model_path)) {
                QMessageBox::warning(this, "Navigation model not generated", 
                    "The navigation model has not been generated yet for this BIM model. Please run "
                    "'ros2 launch navbim_bringup nav_model_gen_launch.py ifc:=" + QString::fromStdString(selected_model) + "'"
                    " to generate it.");
                selected_model = ""; 
                selected_floor = ""; 
                bim_model_status_label->setText("No BIM model selected");
                floor_status_label->setText("No floor selected");
            } else {
                // ToDo: Trigger loading new navigation model
                // Update the status label
                bim_model_status_label->setText("Selected model: " + selected);
                floor_status_label->setText("No floor selected");
                // Search for floors in the selected model
                searchFloors();
            }
        }
    }

    void BimPanel::onFloorsClicked()
    {
        if (floors.empty()) {
            QMessageBox::warning(this, "No floors", "No floors found for the BIM model.");
            return;
        }

        bool ok;        
        QString selected = QInputDialog::getItem(this, 
                                            "Select floor", 
                                            "Available floors:", 
                                            floors, 
                                            0,      // Default selection index
                                            false,  // Non-editable
                                            &ok);

        if (ok && !selected.isEmpty()) {
            selected_floor = selected.toStdString();
            std::string navigation_model_path = ament_index_cpp::get_package_share_directory("navbim_bringup") + "/nav_model/" + selected_model;
            visualization_msgs::msg::MarkerArray empty_markers;
            if (selected == "None") {
                // Reset the floor parameter if "None" is selected
                floor_status_label->setText("No floor selected");
                selected_floor = ""; 
                
                // Publish "None" to indicate no floor filtering
                auto msg = std::make_shared<std_msgs::msg::String>();
                msg->data = "None";
                selected_floor_pub_->publish(*msg);
                
                if (!selected_model.empty()) {
                    // Display mesh for whole building
                    displayIfcElements(navigation_model_path, selected_model, selected_model);
                } else {
                    QMessageBox::warning(this, "No BIM Model Selected", "Please select a BIM model first.");
                }
            }
            else {
                // Update the status label
                floor_status_label->setText("Selected floor: " + selected);
                
                // Publish selected floor name
                auto msg = std::make_shared<std_msgs::msg::String>();
                msg->data = selected_floor;
                selected_floor_pub_->publish(*msg);
                
                if (!selected_model.empty()) {
                    // Display mesh for selected floor
                    displayIfcElements(navigation_model_path, selected_model, selected_floor);
                } else {
                    QMessageBox::warning(this, "No BIM Model Selected", "Please select a BIM model first.");
                }
            }
        }
    }

    void BimPanel::searchBimModels(const std::string& bim_path)
    {
        try {
            for (const auto & entry : std::filesystem::directory_iterator(bim_path)) {
                if (entry.is_regular_file() && entry.path().extension() == ".ifc") {
                    // Add the filename without extension to the list of BIM models
                    std::string filename = entry.path().stem().string();
                    bim_models.push_back(QString::fromStdString(filename));
                }
            }
            bim_models.push_back(QString::fromStdString("None")); // Add "None" option for no model
        }
        catch(const std::exception& e) {
            QMessageBox::warning(this, "Failed to find BIM models", e.what());
        }
    }

    bool BimPanel::isLifecycleNodeActive(const std::string& /* node_name */)
    {
        if (!topomap_state_client_->service_is_ready()) {
            return false;
        }

        auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
        auto future = topomap_state_client_->async_send_request(request);

        // Wait for the service response with a short timeout
        if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(2)) == 
            rclcpp::FutureReturnCode::SUCCESS) {
            auto response = future.get();
            // lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE = 3
            return response->current_state.id == 3;
        }
        
        return false;
    }

    void BimPanel::tryLoadFloors()
    {
        // Already loaded, stop timer
        if (floors_loaded_) {
            if (floor_loader_timer_ && floor_loader_timer_->isActive()) {
                floor_loader_timer_->stop();
            }
            return;
        }
        
        // Check if topomap_server is active and service is ready
        if (!isLifecycleNodeActive("topomap_server")) {
            return;  // Not ready yet, try again next tick
        }
        
        if (!floor_nodes_client_->service_is_ready()) {
            return;  // Service not ready yet, try again next tick
        }
        
        // Service is ready! Stop the timer and make async request
        if (floor_loader_timer_ && floor_loader_timer_->isActive()) {
            floor_loader_timer_->stop();
        }
        
        RCLCPP_INFO(node_->get_logger(), "TopomapServer is ready, loading floors asynchronously...");
        
        // Use QPointer for safe callback after potential panel destruction
        QPointer<BimPanel> safe_this(this);
        auto request = std::make_shared<navbim_msgs::srv::GetFloorNodes::Request>();
        auto node = node_;  // Capture shared_ptr to keep node alive
        
        floor_nodes_client_->async_send_request(request,
            [safe_this, node](rclcpp::Client<navbim_msgs::srv::GetFloorNodes>::SharedFuture future) {
                // Check if panel still exists
                if (!safe_this) {
                    RCLCPP_DEBUG(node->get_logger(), "BimPanel destroyed, ignoring floor response");
                    return;
                }
                
                try {
                    auto response = future.get();
                    
                    // Use Qt's thread-safe mechanism to update UI
                    // Extract floor data before Qt callback to avoid null pointer warnings
                    std::vector<std::tuple<std::string, double, double>> floor_data;
                    bool success = response->success;
                    std::string message = response->message;
                    
                    if (success) {
                        for (const auto& floor_node : response->nodes.nodes) {
                            if (floor_node.type == "floor") {
                                floor_data.emplace_back(floor_node.name, floor_node.min_z, floor_node.max_z);
                            }
                        }
                        
                        // Sort floors by min_z (height)
                        if (!floor_data.empty()) {
                            std::sort(floor_data.begin(), floor_data.end(), 
                                     [](const auto& a, const auto& b) { return std::get<1>(a) < std::get<1>(b); });
                        }
                    }
                    
                    // Use Qt's thread-safe mechanism to update UI
                    // Suppress false-positive null-dereference warnings from GCC's static analysis
                    #pragma GCC diagnostic push
                    #pragma GCC diagnostic ignored "-Wnull-dereference"
                    QMetaObject::invokeMethod(safe_this, [safe_this, floor_data, success, message, node]() {
                        // Double-check panel still exists in UI thread
                        if (!safe_this) {
                            return;
                        }
                        
                        if (success) {
                            if (!floor_data.empty()) {
                                // Update UI with captured data
                                safe_this->floors.clear();
                                safe_this->floor_heights.clear();
                                
                                for (const auto& floor : floor_data) {
                                    std::string floor_name = std::get<0>(floor);
                                    double min_z = std::get<1>(floor);
                                    double max_z = std::get<2>(floor);
                                    safe_this->floors.push_back(QString::fromStdString(floor_name));
                                    safe_this->floor_heights[floor_name] = std::make_pair(min_z, max_z);
                                }
                                
                                safe_this->floors.push_back(QString::fromStdString("None"));
                                safe_this->floors_loaded_ = true;
                                safe_this->floor_status_label->setText(
                                    QString("Loaded %1 floors").arg(static_cast<int>(floor_data.size())));
                                
                                RCLCPP_INFO(node->get_logger(), 
                                    "Successfully loaded %zu floor nodes asynchronously", floor_data.size());
                            } else {
                                safe_this->floor_status_label->setText("No floors found");
                                RCLCPP_WARN(node->get_logger(), "No floor nodes found in topological map");
                            }
                        } else {
                            safe_this->floor_status_label->setText("Failed to load floors");
                            RCLCPP_WARN(node->get_logger(), 
                                "Floor nodes service call failed: %s", message.c_str());
                        }
                    }, Qt::QueuedConnection);
                    #pragma GCC diagnostic pop
                    
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(node->get_logger(), "Error in floor loading callback: %s", e.what());
                    
                    QMetaObject::invokeMethod(safe_this, [safe_this]() {
                        if (safe_this) {
                            safe_this->floor_status_label->setText("Error loading floors");
                        }
                    }, Qt::QueuedConnection);
                }
            });
    }

    void BimPanel::searchFloors()
    {
        // If floors already loaded, nothing to do
        if (floors_loaded_ && !floors.empty()) {
            RCLCPP_DEBUG(node_->get_logger(), "Floors already loaded");
            return;
        }
        
        // If timer is still running, floors are being loaded
        if (floor_loader_timer_ && floor_loader_timer_->isActive()) {
            RCLCPP_INFO(node_->get_logger(), "Floor loading already in progress...");
            return;
        }
        
        // Trigger a load attempt
        tryLoadFloors();
    }

    // Use interactive markers
    void BimPanel::displayIfcElements(const std::string& navigation_model_path, const std::string& model, const std::string& floor)
    {
        try {
            // Remove "_Stair_" part and everything that follows from floor name
            std::string floor_for_mesh = floor;
            size_t stair_pos = floor_for_mesh.find("_Stair_");
            if (stair_pos != std::string::npos) {
                floor_for_mesh = floor_for_mesh.substr(0, stair_pos);
            }
            
            // Path to the meshes directory for this model
            std::string meshes_dir = navigation_model_path + "/meshes/";
            
            // Check if meshes directory exists
            if (!std::filesystem::exists(meshes_dir)) {
                QMessageBox::warning(this, "Meshes not found", 
                    "No meshes directory found for model: " + QString::fromStdString(model));
                return;
            }
            
            std::vector<std::string> mesh_files;
            
            // Read YAML file for floor-specific meshes
            std::string yaml_file = navigation_model_path + "/" + floor_for_mesh + "/meshes.yaml";
            if (std::filesystem::exists(yaml_file)) {
                try {
                    YAML::Node yaml_data = YAML::LoadFile(yaml_file);
                    if (yaml_data["meshes"]) {
                        for (const auto& mesh_entry : yaml_data["meshes"]) {
                            std::string mesh_file = mesh_entry.as<std::string>();
                            if (std::filesystem::exists(mesh_file)) {
                                mesh_files.push_back(mesh_file);
                            }
                        }
                    }
                } catch (const YAML::Exception& e) {
                    // Fall back to loading all meshes if YAML parsing fails
                    RCLCPP_WARN(node_->get_logger(), "Failed to parse YAML file %s: %s", yaml_file.c_str(), e.what());
                }
            }
            
            // If no meshes were loaded from YAML or YAML doesn't exist, load all mesh files
            if (mesh_files.empty()) {
                for (const auto& entry : std::filesystem::directory_iterator(meshes_dir)) {
                    if (entry.is_regular_file() && 
                       (entry.path().extension() == ".gltf" || 
                       entry.path().extension() == ".glb" || 
                       entry.path().extension() == ".stl" || 
                       entry.path().extension() == ".obj" || 
                       entry.path().extension() == ".dae")) {
                        mesh_files.push_back(entry.path().string());
                    }
                }
            }
            
            // Display elements using IfcElementManager
            if (!mesh_files.empty()) {
                ifc_element_manager_->displayElements(mesh_files, "ifc");
                RCLCPP_INFO(node_->get_logger(), "Displayed %zu IFC elements as interactive markers", mesh_files.size());
            } else {
                QMessageBox::warning(this, "No meshes found", 
                    "No mesh files found for model: " + QString::fromStdString(model) + 
                    ", floor: " + QString::fromStdString(floor_for_mesh));
            }
        }
        catch(const std::exception& e) {
            QMessageBox::warning(this, "Mesh loading error", e.what());
        }
    }

    void BimPanel::startPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        geometry_msgs::msg::PoseStamped start_pose;
        start_pose.header = msg->header;
        start_pose.pose = msg->pose.pose;
        // Adjust the height based on current floor
        if (!selected_floor.empty()) {
            auto floor_it = floor_heights.find(selected_floor);
            if (floor_it != floor_heights.end()) {
                float floor_elevation = static_cast<float>(floor_it->second.first);
                start_pose.pose.position.z = floor_elevation + elevate_start_goal;
            } else {
                RCLCPP_WARN(node_->get_logger(), "Floor '%s' not found in height data", selected_floor.c_str());
            }
        } else {
            RCLCPP_INFO(node_->get_logger(), "No floor selected, keeping original z=%.2f", start_pose.pose.position.z);
        }
        // Publish the adjusted pose
        start_pose_pub->publish(start_pose);
    }

    void BimPanel::goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        geometry_msgs::msg::PoseStamped goal_pose = *msg;
        // Adjust the height based on current floor
        if (!selected_floor.empty()) {
            auto floor_it = floor_heights.find(selected_floor);
            if (floor_it != floor_heights.end()) {
                float floor_elevation = static_cast<float>(floor_it->second.first);
                goal_pose.pose.position.z = floor_elevation + elevate_start_goal;
            } else {
                RCLCPP_WARN(node_->get_logger(), "Floor '%s' not found in height data", selected_floor.c_str());
            }
        } else {
            RCLCPP_INFO(node_->get_logger(), "No floor selected, keeping original z=%.2f", goal_pose.pose.position.z);
        }
        // Publish the adjusted pose
        goal_pose_pub->publish(goal_pose);
    }

    void BimPanel::spinRosNode()
    {
        // Spin the ROS2 node to handle callbacks and interactive marker server communication
        if (node_) {
            rclcpp::spin_some(node_);
        }
    }

} // namespace navbim_rviz_plugins

PLUGINLIB_EXPORT_CLASS(navbim_rviz_plugins::BimPanel, rviz_common::Panel)