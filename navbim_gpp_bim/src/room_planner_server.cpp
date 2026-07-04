#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <iterator>
#include <memory>
#include <shared_mutex>
#include <string>
#include <thread>
#include <vector>
#include <utility>
#include <functional>

#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_util/costmap.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/line_iterator.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_thread.hpp"
#include "navbim_multi_costmap_2d/multi_costmap_2d_ros.hpp"
#include "navbim_smoother/simple_smoother.hpp"
#include "navbim_smoother/savitzky_golay_smoother.hpp"

#include "navbim_gpp_bim/room_planner_server.hpp"

using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace navbim_gpp_bim
{

RoomPlannerServer::RoomPlannerServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("room_planner_server", "", options),
  gp_loader_("nav2_core", "nav2_core::GlobalPlanner"),
  default_ids_{"GridBased"},
  default_types_{"nav2_navfn_planner::NavfnPlanner"},
  costmap_update_timeout_(1s),
  costmap_(nullptr),
  node_options_(options)  // Store options for costmap creation
{
  // Declare this node's parameters
  declare_parameter("planner_plugins", default_ids_);
  declare_parameter("expected_planner_frequency", 1.0);
  declare_parameter("costmap_update_timeout", 1.0);
  
  // Room-level path smoothing parameters
  declare_parameter("enable_room_smoothing", rclcpp::ParameterValue(true));
  declare_parameter("room_smoother_type", rclcpp::ParameterValue("simple"));  // "simple" or "savitzky_golay"
  declare_parameter("room_smoother_max_time", rclcpp::ParameterValue(0.5));  // seconds

  get_parameter("planner_plugins", planner_ids_);
  if (planner_ids_ == default_ids_) {
    for (size_t i = 0; i < default_ids_.size(); ++i) {
      declare_parameter(default_ids_[i] + ".plugin", default_types_[i]);
    }
  }
}

RoomPlannerServer::~RoomPlannerServer()
{
  /*
   * Backstop ensuring this state is destroyed, even if deactivate/cleanup are
   * never called.
   */
  planners_.clear();
}

void RoomPlannerServer::setCostmap(
  std::shared_ptr<navbim_multi_costmap_2d::MultiCostmap2DROS> costmap_ros)
{
  costmap_ros_ = costmap_ros;
  RCLCPP_INFO(get_logger(), "Costmap connected to RoomPlannerServer");
}

nav2_util::CallbackReturn
RoomPlannerServer::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  // Create the costmap as a sub-node (following Nav2 pattern exactly)
  // The costmap node is used in the implementation of the room planner
  costmap_ros_ = std::make_shared<navbim_multi_costmap_2d::MultiCostmap2DROS>(
    "global_costmap",
    std::string{get_namespace()},
    get_parameter("use_sim_time").as_bool(),
    node_options_);

  // Configure the costmap (it will create its own executor thread internally)
  costmap_ros_->configure();
  
  // Note: In multi-room setup, we don't have a single global costmap
  // Each room has its own costmap accessed via getCostmapForRoom()
  // costmap_ = costmap_ros_->getCostmap();

  // Collision checker will be initialized per-room when needed
  // if (!costmap_ros_->getUseRadius()) {
  //   collision_checker_ =
  //     std::make_unique<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(
  //     costmap_);
  // }

  tf_ = costmap_ros_->getTfBuffer();

  planner_types_.resize(planner_ids_.size());

  auto node = shared_from_this();

  for (size_t i = 0; i != planner_ids_.size(); i++) {
    try {
      planner_types_[i] = nav2_util::get_plugin_type_param(
        node, planner_ids_[i]);
      
      nav2_core::GlobalPlanner::Ptr planner;
      
      // If the planner type is NavfnPlanner, create our wrapper instead
      if (planner_types_[i] == "nav2_navfn_planner::NavfnPlanner") {
        planner = std::make_shared<NavfnPlannerWrapper>();
        RCLCPP_INFO(
          get_logger(), "Created NavfnPlannerWrapper for plugin %s",
          planner_ids_[i].c_str());
      } else if (planner_types_[i] == "nav2_theta_star_planner::ThetaStarPlanner") {
        planner = std::make_shared<ThetaStarPlannerWrapper>();
        RCLCPP_INFO(
          get_logger(), "Created ThetaStarPlannerWrapper for plugin %s",
          planner_ids_[i].c_str());
      } else if (planner_types_[i] == "navbim_gpp_bim::OMPLPlannerWrapper") {
        planner = std::make_shared<OMPLPlannerWrapper>();
        RCLCPP_INFO(
          get_logger(), "Created OMPLPlannerWrapper for plugin %s",
          planner_ids_[i].c_str());
      } else {
        planner = gp_loader_.createUniqueInstance(planner_types_[i]);
        RCLCPP_INFO(
          get_logger(), "Created global planner plugin %s of type %s",
          planner_ids_[i].c_str(), planner_types_[i].c_str());
      }
      
      planner->configure(node, planner_ids_[i], tf_, costmap_ros_);
      planners_.insert({planner_ids_[i], planner});
    } catch (const std::exception & ex) {
      RCLCPP_FATAL(
        get_logger(), "Failed to create global planner. Exception: %s",
        ex.what());
      on_cleanup(state);
      return nav2_util::CallbackReturn::FAILURE;
    }
  }

  for (size_t i = 0; i != planner_ids_.size(); i++) {
    planner_ids_concat_ += planner_ids_[i] + std::string(" ");
  }

  RCLCPP_INFO(
    get_logger(),
    "Planner Server has %s planners available.", planner_ids_concat_.c_str());

  double expected_planner_frequency;
  get_parameter("expected_planner_frequency", expected_planner_frequency);
  if (expected_planner_frequency > 0) {
    max_planner_duration_ = 1 / expected_planner_frequency;
  } else {
    RCLCPP_WARN(
      get_logger(),
      "The expected planner frequency parameter is %.4f Hz. The value should to be greater"
      " than 0.0 to turn on duration overrun warning messages", expected_planner_frequency);
    max_planner_duration_ = 0.0;
  }

  // Initialize room-level path smoother
  get_parameter("enable_room_smoothing", enable_room_smoothing_);
  get_parameter("room_smoother_type", room_smoother_type_);
  get_parameter("room_smoother_max_time", room_smoother_max_time_);
  
  {
    try {
      // Create the appropriate smoother based on the type parameter
      if (room_smoother_type_ == "simple") {
        smoother_ = std::make_unique<navbim_smoother::SimpleSmoother>();
        RCLCPP_INFO(get_logger(), "Using SimpleSmoother for path smoothing");
      } else if (room_smoother_type_ == "savitzky_golay") {
        smoother_ = std::make_unique<navbim_smoother::SavitzkyGolaySmoother>();
        RCLCPP_INFO(get_logger(), "Using SavitzkyGolaySmoother for path smoothing");
      } else {
        RCLCPP_ERROR(get_logger(), "Unknown smoother type: %s. Using SimpleSmoother as default.", 
                     room_smoother_type_.c_str());
        smoother_ = std::make_unique<navbim_smoother::SimpleSmoother>();
        room_smoother_type_ = "simple";
      }
      
      // Configure smoother with this node. Pass nullptr for costmap and footprint subscribers
      // since we'll provide costmap pointers directly when calling smooth()
      smoother_->configure(node, "room_planner_smoother", tf_, nullptr, nullptr);
      
      if (enable_room_smoothing_) {
        RCLCPP_INFO(get_logger(), "Room-level path smoothing enabled (type: %s) with max time: %.2f seconds", 
                    room_smoother_type_.c_str(), room_smoother_max_time_);
      } else {
        RCLCPP_INFO(get_logger(), "Room-level path smoothing disabled (smoother initialized for floor-level use)");
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Failed to configure path smoother: %s", ex.what());
      enable_room_smoothing_ = false;
      smoother_.reset();  // Clear smoother on failure
    }
  }

  // Initialize pubs & subs
  plan_publisher_ = create_publisher<nav_msgs::msg::Path>("plan", 1);

  double costmap_update_timeout_dbl;
  get_parameter("costmap_update_timeout", costmap_update_timeout_dbl);
  costmap_update_timeout_ = rclcpp::Duration::from_seconds(costmap_update_timeout_dbl);

  // Create the action server for path planning to a pose in a specific room
  action_server_room_ = std::make_unique<ActionServerToPoseInRoom>(
    shared_from_this(),
    "compute_path_to_pose_in_room",
    std::bind(&RoomPlannerServer::computePlanInRoom, this),
    nullptr,
    std::chrono::milliseconds(500),
    true);
  
  // Create service server for floor smoothing
  // Use MutuallyExclusive callback group to prevent concurrent costmap access
  // which can cause heap corruption when multiple threads access the costmap simultaneously
  floor_smoothing_callback_group_ = create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  floor_smoothing_service_ = create_service<navbim_msgs::srv::SmoothPathWithFloorCostmaps>(
    "/smooth_path_with_floor_costmaps",
    std::bind(&RoomPlannerServer::smoothPathWithFloorCostmaps, this,
              std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
    rclcpp::SystemDefaultsQoS(),
    floor_smoothing_callback_group_);

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
RoomPlannerServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  try {
    plan_publisher_->on_activate();
    
    // Activate the costmap FIRST (this waits for room costmaps to load)
    // Must complete before accepting planning requests via action server
    const auto costmap_ros_state = costmap_ros_->activate();
    if (costmap_ros_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      RCLCPP_ERROR(get_logger(), "Costmap failed to activate");
      return nav2_util::CallbackReturn::FAILURE;
    }
    
    // NOW activate action server - costmaps are ready
    action_server_room_->activate();

    PlannerMap::iterator it;
    for (it = planners_.begin(); it != planners_.end(); ++it) {
      it->second->activate();
    }

    // Add callback for dynamic parameters
    dyn_params_handler_ = add_on_set_parameters_callback(
      std::bind(&RoomPlannerServer::dynamicParametersCallback, this, _1));

    // create bond connection
    createBond();
    
    RCLCPP_INFO(get_logger(), "Activated");
    return nav2_util::CallbackReturn::SUCCESS;
    
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception during activation: %s", e.what());
    return nav2_util::CallbackReturn::FAILURE;
  }
}

nav2_util::CallbackReturn
RoomPlannerServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  action_server_room_->deactivate();
  plan_publisher_->on_deactivate();

  /*
   * The costmap is also a lifecycle node, so it may have already fired on_deactivate
   * via rcl preshutdown cb. Despite the rclcpp docs saying on_shutdown callbacks fire
   * in the order added, the preshutdown callbacks clearly don't per se, due to using an
   * unordered_set iteration. Once this issue is resolved, we can maybe make a stronger
   * ordering assumption: https://github.com/ros2/rclcpp/issues/2096
   */
  costmap_ros_->deactivate();

  PlannerMap::iterator it;
  for (it = planners_.begin(); it != planners_.end(); ++it) {
    it->second->deactivate();
  }

  dyn_params_handler_.reset();

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
RoomPlannerServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  // CRITICAL CLEANUP SEQUENCE (following Nav2 planner_server pattern):
  // The costmap was already deactivated in on_deactivate()
  
  // 1. Reset our own services/publishers first (these are on the main executor)
  action_server_room_.reset();
  floor_smoothing_service_.reset();
  plan_publisher_.reset();
  tf_.reset();
  
  // Note: Don't reset callback groups here - they will be cleaned up automatically
  // when the node is destroyed. Resetting them while services might still be
  // detaching from waitsets can cause FastDDS mutex assertions.
  
  // Give FastDDS time to finish processing any in-flight messages for the entities we just reset
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
  // 2. Cleanup costmap (it manages its own executor_thread_ internally like Nav2's Costmap2DROS)
  //    MultiCostmap2DROS::on_cleanup() stops executor thread and performs lifecycle cleanup
  //    NOTE: Do NOT reset costmap_ros_ here - let it be destroyed naturally when this node
  //    is destroyed. Manually resetting while DDS is still cleaning up causes segfaults.
  if (costmap_ros_) {
    costmap_ros_->cleanup();
    RCLCPP_INFO(get_logger(), "Costmap cleanup complete");
  }

  // Cleanup the smoother
  if (smoother_) {
    smoother_->cleanup();
    smoother_.reset();
  }

  // 5. Cleanup planner plugins
  PlannerMap::iterator it;
  for (it = planners_.begin(); it != planners_.end(); ++it) {
    it->second->cleanup();
  }
  planners_.clear();

  costmap_ = nullptr;
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
RoomPlannerServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

template<typename T>
bool RoomPlannerServer::isServerInactive(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server)
{
  if (action_server == nullptr || !action_server->is_server_active()) {
    RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
    return true;
  }

  return false;
}

template<typename T>
bool RoomPlannerServer::isCancelRequested(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server)
{
  if (action_server->is_cancel_requested()) {
    RCLCPP_INFO(get_logger(), "Goal was canceled. Canceling planning action.");
    action_server->terminate_all();
    return true;
  }

  return false;
}

template<typename T>
void RoomPlannerServer::getPreemptedGoalIfRequested(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
  typename std::shared_ptr<const typename T::Goal> goal)
{
  if (action_server->is_preempt_requested()) {
    goal = action_server->accept_pending_goal();
  }
}

nav_msgs::msg::Path
RoomPlannerServer::getPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const std::string & planner_id,
  std::function<bool()> cancel_checker)
{
  RCLCPP_DEBUG(
    get_logger(), "Attempting to a find path from (%.2f, %.2f) to "
    "(%.2f, %.2f).", start.pose.position.x, start.pose.position.y,
    goal.pose.position.x, goal.pose.position.y);

  if (planners_.find(planner_id) != planners_.end()) {
    return planners_[planner_id]->createPlan(start, goal, cancel_checker);
  } else {
    if (planners_.size() == 1 && planner_id.empty()) {
      RCLCPP_WARN_ONCE(
        get_logger(), "No planners specified in action call. "
        "Server will use only plugin %s in server."
        " This warning will appear once.", planner_ids_concat_.c_str());
      return planners_[planners_.begin()->first]->createPlan(
        start, goal, cancel_checker);
    } else {
      RCLCPP_ERROR(
        get_logger(), "planner %s is not a valid planner. "
        "Planner names are: %s", planner_id.c_str(),
        planner_ids_concat_.c_str());
      throw nav2_core::InvalidPlanner("Planner id " + planner_id + " is invalid");
    }
  }

  return nav_msgs::msg::Path();
}

void
RoomPlannerServer::publishPlan(const nav_msgs::msg::Path & path)
{
  auto msg = std::make_unique<nav_msgs::msg::Path>(path);
  if (plan_publisher_->is_activated() && plan_publisher_->get_subscription_count() > 0) {
    plan_publisher_->publish(std::move(msg));
  }
}

rcl_interfaces::msg::SetParametersResult
RoomPlannerServer::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);
  rcl_interfaces::msg::SetParametersResult result;
  for (auto parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();
    if (param_name.find('.') != std::string::npos) {
      continue;
    }

    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (param_name == "expected_planner_frequency") {
        if (parameter.as_double() > 0) {
          max_planner_duration_ = 1 / parameter.as_double();
        } else {
          RCLCPP_WARN(
            get_logger(),
            "The expected planner frequency parameter is %.4f Hz. The value should to be greater"
            " than 0.0 to turn on duration overrun warning messages", parameter.as_double());
          max_planner_duration_ = 0.0;
        }
      }
    }
  }

  result.successful = true;
  return result;
}

void RoomPlannerServer::computePlanInRoom()
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);

  auto start_time = this->now();
  auto goal = action_server_room_->get_current_goal();
  auto result = std::make_shared<ActionToPoseInRoomResult>();

  try {
    if (isServerInactive<ActionToPoseInRoom>(action_server_room_) ||
      isCancelRequested<ActionToPoseInRoom>(action_server_room_))
    {
      RCLCPP_DEBUG(get_logger(), "Action server inactive or cancel requested. Stopping.");
      return;
    }

    getPreemptedGoalIfRequested<ActionToPoseInRoom>(action_server_room_, goal);

    // Check if robot is properly localized before attempting to plan
    // This prevents transform timeout errors when robot isn't localized yet
    // Skip this check if use_start is true (explicit start pose provided)
    if (!goal->use_start) {
      try {
        geometry_msgs::msg::PoseStamped robot_pose;
        if (!costmap_ros_->getRobotPose(robot_pose)) {
          RCLCPP_WARN(get_logger(), "Robot not localized yet, cannot plan path in room");
          result->error_code = ActionToPoseInRoomResult::TF_ERROR;
          result->error_msg = "Robot not localized - please set initial pose";
          action_server_room_->terminate_current(result);
          return;
        }
      } catch (const std::exception & ex) {
        RCLCPP_WARN(get_logger(), "Failed to get robot pose: %s", ex.what());
        result->error_code = ActionToPoseInRoomResult::TF_ERROR;
        result->error_msg = "Failed to get robot pose - robot may not be localized";
        action_server_room_->terminate_current(result);
        return;
      }
    }

    // Get room-specific costmap using MultiCostmap2DROS
    auto multi_costmap = std::dynamic_pointer_cast<navbim_multi_costmap_2d::MultiCostmap2DROS>(costmap_ros_);
    if (!multi_costmap) {
      RCLCPP_ERROR(get_logger(), "Costmap is not a MultiCostmap2DROS instance");
      result->error_code = ActionToPoseInRoomResult::UNKNOWN;
      result->error_msg = "Internal error: costmap type mismatch";
      action_server_room_->terminate_current(result);
      return;
    }
    
    // Check if requesting aggregated floor costmap (one-level planning)
    bool using_floor_costmap = (goal->room_name == goal->floor_name);
    
    if (using_floor_costmap) {
      RCLCPP_DEBUG(get_logger(), "Using aggregated floor costmap for floor '%s' (one-level planning)",
                  goal->floor_name.c_str());
    }
    
    // CRITICAL: Hold shared locks during entire planning operation to prevent race conditions
    // - costmap_mutex_: Protects room costmap data from updates
    // - floor_cache_mutex_: Protects floor costmap data from updates (only if using floor costmaps)
    // Multiple planners can read concurrently (shared locks), but updates are blocked
    // NOTE: Floor costmaps are created by timer thread, NOT on-demand, to avoid thread-safety issues
    std::shared_lock<std::shared_mutex> costmap_lock(multi_costmap->getCostmapMutex());
    std::shared_lock<std::shared_mutex> floor_lock(multi_costmap->getFloorCacheMutex());
    
    // Get the appropriate costmap (keep shared_ptr alive to prevent use-after-free)
    std::shared_ptr<nav2_costmap_2d::Costmap2D> floor_costmap_shared;  // Keep floor costmap alive
    std::shared_ptr<nav2_costmap_2d::LayeredCostmap> room_layered_costmap;  // Keep room costmap alive
    nav2_costmap_2d::Costmap2D * room_costmap = nullptr;
    if (using_floor_costmap) {
      // Floor lock protects pointer validity
      floor_costmap_shared = multi_costmap->getCostmapForFloor(goal->floor_name);
      room_costmap = floor_costmap_shared.get();
    } else {
      // Can release floor lock for room-only planning
      floor_lock.unlock();
      room_layered_costmap = multi_costmap->getCostmapForRoom(goal->floor_name, goal->room_name);
      if (room_layered_costmap) {
        room_costmap = room_layered_costmap->getCostmap();
      }
    }
    
    if (!room_costmap) {
      if (using_floor_costmap) {
        RCLCPP_ERROR(get_logger(), "Failed to get aggregated costmap for floor %s", 
                     goal->floor_name.c_str());
        result->error_msg = "Failed to get aggregated costmap for specified floor";
      } else {
        RCLCPP_ERROR(get_logger(), "Failed to get costmap for room %s/%s", 
                     goal->floor_name.c_str(), goal->room_name.c_str());
        result->error_msg = "Failed to get costmap for specified room";
      }
      result->error_code = ActionToPoseInRoomResult::ROOM_NOT_FOUND;
      action_server_room_->terminate_current(result);
      return;
    }

    // Update the planner plugin's costmap to use the room-specific costmap
    // We need to update the planner's internal costmap pointer, not just the server's
    std::string planner_id = goal->planner_id.empty() ? default_ids_[0] : goal->planner_id;
    auto planner_it = planners_.find(planner_id);
    if (planner_it == planners_.end()) {
      RCLCPP_ERROR(get_logger(), "Planner plugin '%s' not found", planner_id.c_str());
      result->error_code = ActionToPoseInRoomResult::INVALID_PLANNER;
      result->error_msg = "Planner plugin not found: " + planner_id;
      action_server_room_->terminate_current(result);
      return;
    }

    // Try to cast the planner to our wrapper to update its costmap
    auto navfn_wrapper_planner = std::dynamic_pointer_cast<NavfnPlannerWrapper>(planner_it->second);
    auto theta_wrapper_planner = std::dynamic_pointer_cast<ThetaStarPlannerWrapper>(planner_it->second);
    auto ompl_wrapper_planner = std::dynamic_pointer_cast<OMPLPlannerWrapper>(planner_it->second);
    nav2_costmap_2d::Costmap2D * original_costmap = nullptr;
    
    if (navfn_wrapper_planner) {
      // Save original costmap and update to room costmap
      original_costmap = navfn_wrapper_planner->getCostmap();
      navfn_wrapper_planner->setCostmap(room_costmap);
      RCLCPP_DEBUG(get_logger(), "Updated NavfnPlanner costmap to room %s/%s", 
                   goal->floor_name.c_str(), goal->room_name.c_str());
    } else if (theta_wrapper_planner) {
      // Save original costmap and update to room costmap
      original_costmap = theta_wrapper_planner->getCostmap();
      theta_wrapper_planner->setCostmap(room_costmap);
      RCLCPP_DEBUG(get_logger(), "Updated ThetaStarPlanner costmap to room %s/%s", 
                   goal->floor_name.c_str(), goal->room_name.c_str());
    } else if (ompl_wrapper_planner) {
      // Save original costmap and update to room costmap
      original_costmap = ompl_wrapper_planner->getCostmap();
      ompl_wrapper_planner->setCostmap(room_costmap);
      RCLCPP_DEBUG(get_logger(), "Updated OMPL Planner costmap to room %s/%s", 
                   goal->floor_name.c_str(), goal->room_name.c_str());
    } else {
      RCLCPP_WARN(get_logger(), 
                  "Planner '%s' is not a wrapped planner, costmap switching may not work correctly",
                  planner_id.c_str());
    }

    // CRITICAL: Lock Costmap2D's internal mutex during planning to prevent
    // concurrent modifications from updateFloorCostmapRegion()
    auto * costmap_mutex = room_costmap->getMutex();
    std::unique_ptr<std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t>> costmap_data_lock;
    if (costmap_mutex) {
      costmap_data_lock = std::make_unique<std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t>>(*costmap_mutex);
    } else {
      RCLCPP_WARN(get_logger(), "Costmap mutex is null, planning without data lock protection");
    }

    // Plan the path using the room-specific costmap
    result->path = getPlan(
      goal->start, goal->goal, goal->planner_id,
      [this]() {
        return action_server_room_->is_cancel_requested();
      });

    // Restore original costmap in the planner
    if (navfn_wrapper_planner && original_costmap) {
      navfn_wrapper_planner->setCostmap(original_costmap);
    } else if (theta_wrapper_planner && original_costmap) {
      theta_wrapper_planner->setCostmap(original_costmap);
    } else if (ompl_wrapper_planner && original_costmap) {
      ompl_wrapper_planner->setCostmap(original_costmap);
    }

    // Apply path smoothing if enabled and path is valid
    if (enable_room_smoothing_ && smoother_ && result->path.poses.size() > 2) {
      try {
        RCLCPP_DEBUG(get_logger(), "Smoothing path with %zu poses", result->path.poses.size());
        bool smoothing_success = false;
        
        // Call the appropriate smooth() method based on smoother type
        if (room_smoother_type_ == "simple") {
          auto simple_smoother = dynamic_cast<navbim_smoother::SimpleSmoother*>(smoother_.get());
          if (simple_smoother) {
            smoothing_success = simple_smoother->smooth(
              result->path, room_costmap, rclcpp::Duration::from_seconds(room_smoother_max_time_));
          }
        } else if (room_smoother_type_ == "savitzky_golay") {
          auto sg_smoother = dynamic_cast<navbim_smoother::SavitzkyGolaySmoother*>(smoother_.get());
          if (sg_smoother) {
            smoothing_success = sg_smoother->smooth(
              result->path, room_costmap, rclcpp::Duration::from_seconds(room_smoother_max_time_));
          }
        }
        
        if (smoothing_success) {
          RCLCPP_DEBUG(get_logger(), "Path smoothed successfully, new size: %zu poses", 
                      result->path.poses.size());
        } else {
          RCLCPP_WARN(get_logger(), "Path smoothing did not complete successfully");
          if (!result->warnings.empty()) result->warnings += "; ";
          result->warnings += "Room smoothing did not complete successfully";
        }
      } catch (const std::exception & ex) {
        RCLCPP_WARN(get_logger(), "Path smoothing failed: %s. Using original path.", ex.what());
        if (!result->warnings.empty()) result->warnings += "; ";
        result->warnings += std::string("Room smoothing failed: ") + ex.what();
        // Continue with original path if smoothing fails
      }
    }

    if (result->path.poses.size() == 0) {
      result->error_code = ActionToPoseInRoomResult::NO_VALID_PATH;
      result->error_msg = "No valid path found in room";
      result->path_distance = -1.0;
      result->path_cost = -1.0;
    } else {
      result->error_code = ActionToPoseInRoomResult::NONE;
      result->error_msg = "Path found successfully";
      
      // Calculate path metrics
      result->path_distance = calculatePathDistance(result->path);
      // Use cost = distance for now
      result->path_cost = result->path_distance;
      //result->path_cost = calculatePathCost(result->path, room_costmap);
      
      RCLCPP_DEBUG(
        get_logger(),
        "Path found: distance=%.2f m, average_cost=%.2f",
        result->path_distance, result->path_cost);
      
      publishPlan(result->path);
    }

    auto end_time = this->now();
    result->planning_time = end_time - start_time;
    
    // Final check to ensure action server is still in valid state before completing
    if (!action_server_room_ || isServerInactive<ActionToPoseInRoom>(action_server_room_)) {
      RCLCPP_WARN(get_logger(), "Action server became inactive before completing, cannot send result");
      return;
    }
    
    // Check if goal was cancelled during planning
    if (isCancelRequested<ActionToPoseInRoom>(action_server_room_)) {
      RCLCPP_INFO(get_logger(), "Goal was cancelled during planning");
      return;
    }
    
    // Attempt to complete the action, catching any internal action server errors
    try {
      action_server_room_->succeeded_current(result);
    } catch (const std::exception & result_ex) {
      RCLCPP_ERROR(get_logger(), "Failed to send action result: %s", result_ex.what());
      // Action may have been cancelled or server state changed
      return;
    }

  } catch (const std::exception & ex) {
    std::string error_msg;
    exceptionWarning(goal->start, goal->goal, goal->planner_id, 
                     goal->floor_name, goal->room_name, ex, error_msg);
    result->error_code = ActionToPoseInRoomResult::UNKNOWN;
    result->error_msg = error_msg;
    action_server_room_->terminate_current(result);
  }
}

double RoomPlannerServer::calculatePathDistance(const nav_msgs::msg::Path & path)
{
  if (path.poses.size() < 2) {
    return 0.0;
  }

  double total_distance = 0.0;
  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & prev_pose = path.poses[i - 1].pose.position;
    const auto & curr_pose = path.poses[i].pose.position;
    
    double dx = curr_pose.x - prev_pose.x;
    double dy = curr_pose.y - prev_pose.y;
    double dz = curr_pose.z - prev_pose.z;
    
    total_distance += std::sqrt(dx * dx + dy * dy + dz * dz);
  }
  
  return total_distance;
}

double RoomPlannerServer::calculatePathCost(
  const nav_msgs::msg::Path & path,
  nav2_costmap_2d::Costmap2D * costmap)
{
  if (path.poses.size() < 2 || !costmap) {
    return -1.0;  // Invalid path or costmap
  }

  double total_cost = 0.0;
  int total_cells = 0;
  double resolution = costmap->getResolution();

  // Iterate through each segment of the path
  for (size_t i = 1; i < path.poses.size(); ++i) {
    unsigned int x0, y0, x1, y1;
    
    // Convert world coordinates to map coordinates
    if (!costmap->worldToMap(
          path.poses[i - 1].pose.position.x,
          path.poses[i - 1].pose.position.y, x0, y0))
    {
      continue;  // Skip if start point is outside costmap
    }
    
    if (!costmap->worldToMap(
          path.poses[i].pose.position.x,
          path.poses[i].pose.position.y, x1, y1))
    {
      continue;  // Skip if end point is outside costmap
    }
    
    // Use Bresenham line iterator to sample ALL cells along the segment
    for (nav2_util::LineIterator line(x0, y0, x1, y1); line.isValid(); line.advance()) {
      unsigned char cost = costmap->getCost(line.getX(), line.getY());
      total_cost += static_cast<double>(cost);
      total_cells++;
    }
  }

  // Convert accumulated cost to "effective distance" in meters
  // The idea: each cell contributes (1 + penalty_factor) to the effective distance
  // 
  // Nav2 inflation formula: cost = 252 * exp(-k * distance)
  // We invert this to get a traversability factor:
  // - Free space (cost=0): factor = 1.0 (no penalty)
  // - Inflated areas (cost>0): factor = 1.0 + (cost/252)
  // 
  // This way:
  // - Path through free space: effective_distance ≈ actual_distance
  // - Path through obstacles: effective_distance > actual_distance
  
  if (total_cells > 0) {
    double avg_cost = total_cost / total_cells;
    double cell_distance = total_cells * resolution;  // actual distance in meters
    
    // Traversability factor based on average cost
    // Free space (avg_cost=0): factor = 1.0
    // Inscribed obstacle (avg_cost=252): factor = 2.0
    double traversability_factor = 1.0 + (avg_cost / 252.0);
    
    // Effective distance = actual_distance * traversability_factor
    return cell_distance * traversability_factor;
  }

  return 0.0;
}

void RoomPlannerServer::exceptionWarning(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const std::string & planner_id,
  const std::string & floor_name,
  const std::string & room_name,
  const std::exception & ex,
  std::string & error_msg)
{
  std::stringstream ss;
  ss << std::fixed << std::setprecision(2)
     << planner_id << " plugin failed to plan in room '" 
     << room_name << "' on floor '" << floor_name << "' from ("
     << start.pose.position.x << ", " << start.pose.position.y
     << ") to ("
     << goal.pose.position.x << ", " << goal.pose.position.y << ")"
     << ": \"" << ex.what() << "\"";

  error_msg = ss.str();
  RCLCPP_WARN(get_logger(), error_msg.c_str());
}

nav_msgs::msg::Path RoomPlannerServer::smoothFloorSegments(
  const nav_msgs::msg::Path & path,
  const std::vector<std::tuple<std::string, size_t, size_t>> & floor_segments)
{
  // Note: Floor-based smoothing uses the same smoother as room-based smoothing
  // but it can work independently (e.g., when room smoothing is disabled but floor smoothing is enabled)
  if (!smoother_) {
    RCLCPP_WARN(get_logger(), "Smoother not initialized, cannot perform floor smoothing");
    return path;
  }

  nav_msgs::msg::Path smoothed = path;
  
  // Get multi_costmap pointer once
  auto multi_costmap = std::dynamic_pointer_cast<navbim_multi_costmap_2d::MultiCostmap2DROS>(costmap_ros_);
  if (!multi_costmap) {
    RCLCPP_WARN(get_logger(), "Costmap is not a MultiCostmap2DROS instance");
    return path;
  }
  
  // Hold shared locks during smoothing to prevent costmap modifications during access
  // NOTE: Floor costmaps are created by timer thread, NOT on-demand
  // If a floor costmap doesn't exist, smoothing for that segment will be skipped
  std::shared_lock<std::shared_mutex> costmap_lock(multi_costmap->getCostmapMutex());
  std::shared_lock<std::shared_mutex> floor_lock(multi_costmap->getFloorCacheMutex());

  for (const auto & [floor_name, start_idx, end_idx] : floor_segments) {
    // end_idx is inclusive, so valid range is start_idx <= end_idx < size
    if (start_idx > end_idx || end_idx >= smoothed.poses.size()) {
      continue;
    }

    // Get the costmap for this floor (floor_lock protects pointer)
    auto costmap = multi_costmap->getCostmapForFloor(floor_name);
    if (!costmap) {
      RCLCPP_WARN(get_logger(), "No costmap for floor '%s', skipping segment", floor_name.c_str());
      continue;
    }

    // CRITICAL: Lock Costmap2D's internal mutex during smoothing to prevent
    // concurrent modifications from updateFloorCostmapRegion()
    auto * costmap_mutex = costmap->getMutex();
    if (!costmap_mutex) {
      RCLCPP_ERROR(get_logger(), "Costmap mutex is null for floor '%s', skipping", floor_name.c_str());
      continue;
    }
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> costmap_data_lock(*costmap_mutex);

    // Extract segment (end_idx is inclusive, so use end_idx+1 for exclusive end iterator)
    nav_msgs::msg::Path segment;
    segment.header = smoothed.header;
    segment.poses.assign(
      smoothed.poses.begin() + start_idx,
      smoothed.poses.begin() + end_idx + 1);

    // Smooth this segment
    nav_msgs::msg::Path smoothed_segment;
    try {
      bool smoothing_success = false;
      if (room_smoother_type_ == "simple") {
        auto simple_smoother = dynamic_cast<navbim_smoother::SimpleSmoother*>(smoother_.get());
        if (simple_smoother) {
          smoothing_success = simple_smoother->smooth(
            segment, costmap.get(), rclcpp::Duration::from_seconds(room_smoother_max_time_));
        }
      } else if (room_smoother_type_ == "savitzky_golay") {
        auto sg_smoother = dynamic_cast<navbim_smoother::SavitzkyGolaySmoother*>(smoother_.get());
        if (sg_smoother) {
          smoothing_success = sg_smoother->smooth(
            segment, costmap.get(), rclcpp::Duration::from_seconds(room_smoother_max_time_));
        }
      }
      
      if (smoothing_success) {
        smoothed_segment = segment;
        // Replace original segment with smoothed one (end_idx is inclusive, use +1 for exclusive end)
        smoothed.poses.erase(
          smoothed.poses.begin() + start_idx,
          smoothed.poses.begin() + end_idx + 1);
        smoothed.poses.insert(
          smoothed.poses.begin() + start_idx,
          smoothed_segment.poses.begin(),
          smoothed_segment.poses.end());
      } else {
        RCLCPP_WARN(get_logger(), 
          "Floor smoothing not applied for floor '%s' segment [%zu, %zu] (collision detected or no improvement)",
          floor_name.c_str(), start_idx, end_idx);
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "Smoothing failed for floor '%s': %s", floor_name.c_str(), e.what());
    }
  }

  return smoothed;
}

void RoomPlannerServer::smoothPathWithFloorCostmaps(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<navbim_msgs::srv::SmoothPathWithFloorCostmaps::Request> request,
  std::shared_ptr<navbim_msgs::srv::SmoothPathWithFloorCostmaps::Response> response)
{
  RCLCPP_DEBUG(get_logger(), "Floor smoothing service called with %zu floors", 
               request->floor_names.size());
  
  // Build floor segments from service request using provided indices
  std::vector<std::tuple<std::string, size_t, size_t>> floor_segments;
  
  if (request->floor_names.empty()) {
    response->success = false;
    response->message = "No floor names provided";
    response->smoothed_path = request->path;
    return;
  }
  
  if (request->path.poses.empty()) {
    response->success = false;
    response->message = "Empty path provided";
    response->smoothed_path = request->path;
    return;
  }
  
  // Validate that we have matching arrays
  if (request->floor_names.size() != request->segment_start_indices.size() ||
      request->floor_names.size() != request->segment_end_indices.size()) {
    response->success = false;
    response->message = "Mismatched floor_names, segment_start_indices, and segment_end_indices sizes";
    response->smoothed_path = request->path;
    return;
  }
  
  // Build segments from the provided indices
  for (size_t i = 0; i < request->floor_names.size(); ++i) {
    size_t start_idx = request->segment_start_indices[i];
    size_t end_idx = request->segment_end_indices[i];
    
    // Validate indices (inclusive end index)
    if (start_idx >= request->path.poses.size() || end_idx >= request->path.poses.size()) {
      RCLCPP_WARN(get_logger(), "Invalid segment indices for floor '%s': [%zu, %zu] with path size %zu",
                  request->floor_names[i].c_str(), start_idx, end_idx, request->path.poses.size());
      continue;
    }
    
    if (end_idx < start_idx) {
      RCLCPP_DEBUG(get_logger(), "Skipping floor '%s' segment [%zu, %zu] - invalid range (end < start)",
                   request->floor_names[i].c_str(), start_idx, end_idx);
      continue;
    }
    
    // Need at least 3 poses for meaningful smoothing
    if (end_idx <= start_idx + 1) {
      RCLCPP_DEBUG(get_logger(), "Skipping floor '%s' segment [%zu, %zu] - insufficient poses for smoothing",
                   request->floor_names[i].c_str(), start_idx, end_idx);
      continue;
    }
    
    floor_segments.emplace_back(request->floor_names[i], start_idx, end_idx);
    RCLCPP_DEBUG(get_logger(), "Added segment: floor='%s', poses [%zu, %zu] (inclusive)",
                 request->floor_names[i].c_str(), start_idx, end_idx);
  }
  
  // Call the smoothing implementation
  try {
    response->smoothed_path = smoothFloorSegments(request->path, floor_segments);
    response->success = true;
    response->message = "Floor smoothing completed successfully";
  } catch (const std::exception & e) {
    response->success = false;
    response->message = std::string("Floor smoothing failed: ") + e.what();
    response->smoothed_path = request->path;
  }
  RCLCPP_DEBUG(get_logger(), "%s", response->message.c_str());
}

}  // namespace navbim_gpp_bim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(navbim_gpp_bim::RoomPlannerServer)