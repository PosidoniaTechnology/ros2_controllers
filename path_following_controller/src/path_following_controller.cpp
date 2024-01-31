// Copyright (c) 2024, Posidonia Technology d.o.o.
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include "path_following_controller/path_following_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp_action/create_server.hpp"
#include "rclcpp_action/server_goal_handle.hpp"
#include "controller_interface/helpers.hpp"

#include "lifecycle_msgs/msg/state.hpp"

namespace path_following_controller
{
PathFollowingController::PathFollowingController() 
: controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn PathFollowingController::on_init()
{
  try
  {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();

  /*   TODO
    // Set interpolation method from string parameter
    interpolation_method_ = interpolation_methods::from_string(params_.interpolation_method);
  */
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  /*   REVIEW NECESSITY
  // TODO(christophfroehlich): remove deprecation warning
  if (params_.allow_nonzero_velocity_at_trajectory_end)
  {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "[Deprecated]: \"allow_nonzero_velocity_at_trajectory_end\" is set to "
      "true. The default behavior will change to false.");
  }
  */

  return CallbackReturn::SUCCESS;
}

controller_interface::return_type PathFollowingController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    return controller_interface::return_type::OK;
  }


  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn PathFollowingController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const auto LOGGER = get_node()->get_logger();

  // get degrees of freedom
  dof_ = params_.joints.size();

  command_joint_names_ = params_.command_joints;

/*     REVIEW NECESSITY
  if (command_joint_names_.empty())
  {
    command_joint_names_ = params_.joints;
    RCLCPP_INFO(
      LOGGER, "No specific joint names are used for command interfaces. Using 'joints' parameter.");
  }
  else if (command_joint_names_.size() != params_.joints.size())
  {
    RCLCPP_ERROR(
      LOGGER, "'command_joints' parameter has to have the same size as 'joints' parameter.");
    return CallbackReturn::FAILURE;
  }
*/
  if (params_.command_interfaces.empty())
  {
    RCLCPP_ERROR(LOGGER, "'command_interfaces' parameter is empty.");
    return CallbackReturn::FAILURE;
  }

  // Configure joint position error normalization from ROS parameters (angle_wraparound)
  joints_angle_wraparound_.resize(dof_);
  for (size_t i = 0; i < dof_; ++i){
      const auto & gains = params_.gains.joints_map.at(params_.joints[i]);
      joints_angle_wraparound_[i] = gains.angle_wraparound;
  }

  // Check if only allowed interface types are used and initialize storage to avoid memory
  // allocation during activation
  joint_command_interface_.resize(allowed_interface_types_.size());

  has_position_command_interface_ =
    contains_interface_type(params_.command_interfaces, hardware_interface::HW_IF_POSITION);
  has_velocity_command_interface_ =
    contains_interface_type(params_.command_interfaces, hardware_interface::HW_IF_VELOCITY);
  has_acceleration_command_interface_ =
    contains_interface_type(params_.command_interfaces, hardware_interface::HW_IF_ACCELERATION);
  has_effort_command_interface_ =
    contains_interface_type(params_.command_interfaces, hardware_interface::HW_IF_EFFORT);
  
  if (params_.state_interfaces.empty())
  {
    RCLCPP_ERROR(LOGGER, "'state_interfaces' parameter is empty.");
    return CallbackReturn::FAILURE;
  }

  // Check if only allowed interface types are used and initialize storage to avoid memory
  // allocation during activation
  // Note: 'effort' storage is also here, but never used.
  joint_state_interface_.resize(allowed_interface_types_.size());

  has_position_state_interface_ =
    contains_interface_type(params_.state_interfaces, hardware_interface::HW_IF_POSITION);
  has_velocity_state_interface_ =
    contains_interface_type(params_.state_interfaces, hardware_interface::HW_IF_VELOCITY);
  has_acceleration_state_interface_ =
    contains_interface_type(params_.state_interfaces, hardware_interface::HW_IF_ACCELERATION);

  // REVIEW NECESSITY, MOVE TO UTIL
  // Validation of combinations of state and velocity together have to be done
  // here because the parameter validators only deal with each parameter
  // separately.
  if (
    has_velocity_command_interface_ && params_.command_interfaces.size() == 1 &&
    (!has_velocity_state_interface_ || !has_position_state_interface_))
  {
    RCLCPP_ERROR(
      LOGGER,
      "'velocity' command interface can only be used alone if 'velocity' and "
      "'position' state interfaces are present");
    return CallbackReturn::FAILURE;
  }

  // effort is always used alone so no need for size check
  if (
    has_effort_command_interface_ &&
    (!has_velocity_state_interface_ || !has_position_state_interface_))
  {
    RCLCPP_ERROR(
      LOGGER,
      "'effort' command interface can only be used alone if 'velocity' and "
      "'position' state interfaces are present");
    return CallbackReturn::FAILURE;
  }

  // REVIEW AND MOVE TO UTILS
  auto get_interface_list = [](const std::vector<std::string> & interface_types)
  {
    std::stringstream ss_interfaces;
    for (size_t index = 0; index < interface_types.size(); ++index)
    {
      if (index != 0)
      {
        ss_interfaces << " ";
      }
      ss_interfaces << interface_types[index];
    }
    return ss_interfaces.str();
  };
  // Print output so users can be sure the interface setup is correct
  RCLCPP_INFO(
    LOGGER, "Command interfaces are [%s] and state interfaces are [%s].",
    get_interface_list(params_.command_interfaces).c_str(),
    get_interface_list(params_.state_interfaces).c_str());

  /*   TODO
  // parse remaining parameters
  const std::string interpolation_string =
    get_node()->get_parameter("interpolation_method").as_string();
  interpolation_method_ = interpolation_methods::from_string(interpolation_string);
  RCLCPP_INFO(
    LOGGER, "Using '%s' interpolation method.",
    interpolation_methods::InterpolationMethodMap.at(interpolation_method_).c_str());
  */

  /*
  // if there is only velocity or if there is effort command interface
  // then use also PID adapter
  use_closed_loop_pid_adapter_ =
    (has_velocity_command_interface_ && params_.command_interfaces.size() == 1 &&
     !params_.open_loop_control) ||
    has_effort_command_interface_;

  if (use_closed_loop_pid_adapter_)
  {
    pids_.resize(dof_);
    ff_velocity_scale_.resize(dof_);
    tmp_command_.resize(dof_, 0.0);

    update_pids();
  }
  */
  
  // prepare hold_position_msg
  //init_hold_position_msg();

  resize_joint_trajectory_point(state_current_, dof_);
  resize_joint_trajectory_point_command(command_current_, dof_);
  resize_joint_trajectory_point(state_desired_, dof_);
  resize_joint_trajectory_point(state_error_, dof_);
  resize_joint_trajectory_point(last_commanded_state_, dof_);

  // SUBSCRIBERS
  joint_command_subscriber_ =
    get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "~/joint_trajectory", rclcpp::SystemDefaultsQoS(),
      std::bind(&PathFollowingController::subscriber_callback, this, std::placeholders::_1));
  
  // PUBLISHERS
  RCLCPP_INFO(LOGGER, "Controller state will be published at %.2f Hz.", params_.state_publish_rate);
  if (params_.state_publish_rate > 0.0)
    state_publisher_period_ = rclcpp::Duration::from_seconds(1.0 / params_.state_publish_rate);
  else
    state_publisher_period_ = rclcpp::Duration::from_seconds(0.0);

  controller_state_publisher_ = 
    get_node()->create_publisher<ControllerStateMsg>(
      "~/controller_state", rclcpp::SystemDefaultsQoS());
  rt_publisher_ = std::make_unique<RealtimePublisher>(controller_state_publisher_);

  // REVIEW AND SEPARATE THIS
  rt_publisher_->lock();
  rt_publisher_->msg_.joint_names = params_.joints;
  rt_publisher_->msg_.reference.positions.resize(dof_);
  rt_publisher_->msg_.reference.velocities.resize(dof_);
  rt_publisher_->msg_.reference.accelerations.resize(dof_);
  rt_publisher_->msg_.feedback.positions.resize(dof_);
  rt_publisher_->msg_.error.positions.resize(dof_);
  if (has_velocity_state_interface_)
  {
    rt_publisher_->msg_.feedback.velocities.resize(dof_);
    rt_publisher_->msg_.error.velocities.resize(dof_);
  }
  if (has_acceleration_state_interface_)
  {
    rt_publisher_->msg_.feedback.accelerations.resize(dof_);
    rt_publisher_->msg_.error.accelerations.resize(dof_);
  }
  if (has_position_command_interface_)
  {
    rt_publisher_->msg_.output.positions.resize(dof_);
  }
  if (has_velocity_command_interface_)
  {
    rt_publisher_->msg_.output.velocities.resize(dof_);
  }
  if (has_acceleration_command_interface_)
  {
    rt_publisher_->msg_.output.accelerations.resize(dof_);
  }
  if (has_effort_command_interface_)
  {
    rt_publisher_->msg_.output.effort.resize(dof_);
  }
  rt_publisher_->unlock();
 
  last_state_publish_time_ = get_node()->now();

  using namespace std::placeholders;
  // SERVICES
  query_state_service_ = get_node()->create_service<QueryStateType>(
    std::string(get_node()->get_name()) + "/query_state",
    std::bind(&PathFollowingController::query_state_service, this, _1, _2));

  // ACTIONS
  if (params_.allow_partial_joints_goal)
    RCLCPP_INFO(LOGGER, "Goals with partial set of joints are allowed");
  
  RCLCPP_INFO(
    LOGGER, "Action status changes will be monitored at %.2f Hz.", params_.action_monitor_rate);
  action_monitor_period_ = rclcpp::Duration::from_seconds(1.0 / params_.action_monitor_rate);  
  action_server_ = rclcpp_action::create_server<ActionType>(
    get_node()->get_node_base_interface(), get_node()->get_node_clock_interface(),
    get_node()->get_node_logging_interface(), get_node()->get_node_waitables_interface(),
    std::string(get_node()->get_name()) + "/follow_joint_trajectory",
    std::bind(&PathFollowingController::goal_received_callback, this, _1, _2),
    std::bind(&PathFollowingController::goal_cancelled_callback, this, _1),
    std::bind(&PathFollowingController::goal_accepted_callback, this, _1));

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration PathFollowingController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  if (dof_ == 0)
  {
    fprintf(
      stderr,
      "Degrees of freedom invalid;"
      " expected a positive value. Actual value: %zu\n",
      dof_);
    std::exit(EXIT_FAILURE);
  }

  command_interfaces_config.names.reserve(dof_ * params_.command_interfaces.size());
  for (const auto & joint_name : command_joint_names_)
  {
    for (const auto & interface_type : params_.command_interfaces)
    {
      command_interfaces_config.names.push_back(joint_name + "/" + interface_type);
    }
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration PathFollowingController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names.reserve(dof_ * params_.state_interfaces.size());
  for (const auto & joint_name : params_.joints)
  {
    for (const auto & interface_type : params_.state_interfaces)
    {
      state_interfaces_config.names.push_back(joint_name + "/" + interface_type);
    }
  }
  return state_interfaces_config;
}


controller_interface::CallbackReturn PathFollowingController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // update the dynamic map parameters
  param_listener_->refresh_dynamic_parameters();
  params_ = param_listener_->get_params();

  // parse remaining parameters
  default_tolerances_ = get_segment_tolerances(params_);


  // REVIEW is this check necessary. Can't we just use _has_x_interface flags for consistency?
  // order all joints in the storage
  for (const auto & interface : params_.command_interfaces)
  {
    auto it =
      std::find(allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
    auto index = std::distance(allowed_interface_types_.begin(), it);
    if (!controller_interface::get_ordered_interfaces(
          command_interfaces_, command_joint_names_, interface, joint_command_interface_[index]))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Expected %zu '%s' command interfaces, got %zu.", dof_,
        interface.c_str(), joint_command_interface_[index].size());
      return CallbackReturn::ERROR;
    }
  }
  for (const auto & interface : params_.state_interfaces)
  {
    auto it =
      std::find(allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
    auto index = std::distance(allowed_interface_types_.begin(), it);
    if (!controller_interface::get_ordered_interfaces(
          state_interfaces_, params_.joints, interface, joint_state_interface_[index]))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Expected %zu '%s' state interfaces, got %zu.", dof_,
        interface.c_str(), joint_state_interface_[index].size());
      return CallbackReturn::ERROR;
    }
  }
  
  /*
  traj_external_point_ptr_ = std::make_shared<Trajectory>();
  traj_msg_external_point_ptr_.writeFromNonRT(
    std::shared_ptr<trajectory_msgs::msg::JointTrajectory>());
  */

  subscriber_is_active_ = true;
  last_state_publish_time_ = get_node()->now();

  // REVIEW: What is meant by restart? Move to utils
  // Handle restart of controller by reading from commands if those are not NaN (a controller was
  // running already)

  /*
  trajectory_msgs::msg::JointTrajectoryPoint state;
  resize_joint_trajectory_point(state, dof_);
  if (read_state_from_command_interfaces(state))
  {
    state_current_ = state;
    last_commanded_state_ = state;
  }
  else
  {
    // Initialize current state storage from hardware
    read_state_from_state_interfaces(state_current_);
    read_state_from_state_interfaces(last_commanded_state_);
  }
  */

  /* TRAJECTORY
  // The controller should start by holding position at the beginning of active state
  add_new_trajectory_msg(set_hold_position());
  rt_is_holding_.writeFromNonRT(true);
  */

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PathFollowingController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const auto active_goal = *rt_active_goal_.readFromNonRT();
  
  if (active_goal)
  {
    rt_has_pending_goal_.writeFromNonRT(false);
    auto action_res = std::make_shared<ActionType::Result>();
    action_res->set__error_code(ActionType::Result::INVALID_GOAL);
    action_res->set__error_string("Current goal cancelled during controller deactivation.");
    active_goal->setCanceled(action_res);
    rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
  }

  // REVIEW, separate to utils and combine with hardware_write in update.
  for (size_t index = 0; index < dof_; ++index)
  {
    if (has_position_command_interface_)
      joint_command_interface_[0][index].get().set_value(
        joint_command_interface_[0][index].get().get_value());

    if (has_velocity_command_interface_)
      joint_command_interface_[1][index].get().set_value(0.0);

    if (has_acceleration_command_interface_)
      joint_command_interface_[2][index].get().set_value(0.0);

    // TODO(anyone): How to halt when using effort commands?
    if (has_effort_command_interface_)
      joint_command_interface_[3][index].get().set_value(0.0);
  }
  
  for (size_t index = 0; index < allowed_interface_types_.size(); ++index)
  {
    joint_command_interface_[index].clear();
    joint_state_interface_[index].clear();
  }
  release_interfaces();

  subscriber_is_active_ = false;

  /* TRAJECTORY
  traj_external_point_ptr_.reset();
  */

  return controller_interface::CallbackReturn::SUCCESS;
}

// UTILS ----------------------------------------

bool PathFollowingController::validate_trajectory_msg(
  const trajectory_msgs::msg::JointTrajectory & trajectory) const
{
  // If partial joints goals are not allowed, goal should specify all controller joints
  if (!params_.allow_partial_joints_goal)
  {
    if (trajectory.joint_names.size() != dof_)
    {
      RCLCPP_ERROR(get_node()->get_logger(),
        "Joints on incoming trajectory don't match the controller joints.");
      return false;
    }
  }

  if (trajectory.joint_names.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Empty joint names on incoming trajectory.");
    return false;
  }

  const auto trajectory_start_time = static_cast<rclcpp::Time>(trajectory.header.stamp);
  // If the starting time is set to 0.0, it means the controller should start now.
  // Otherwise we check if the trajectory ends before the current time,
  // in which case it can be ignored.
  if (trajectory_start_time.seconds() != 0.0)
  {
    auto trajectory_end_time = trajectory_start_time;
    for (const auto & p : trajectory.points)
    {
      trajectory_end_time += p.time_from_start;
    }
    if (trajectory_end_time < get_node()->now())
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Received trajectory with non-zero start time (%f) that ends in the past (%f)",
        trajectory_start_time.seconds(), trajectory_end_time.seconds());
      return false;
    }
  }

  for (size_t i = 0; i < trajectory.joint_names.size(); ++i)
  {
    const std::string & incoming_joint_name = trajectory.joint_names[i];

    auto it = std::find(params_.joints.begin(), params_.joints.end(), incoming_joint_name);
    if (it == params_.joints.end())
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Incoming joint %s doesn't match the controller's joints.",
        incoming_joint_name.c_str());
      return false;
    }
  }

  if (!params_.allow_nonzero_velocity_at_trajectory_end)
  {
    for (size_t i = 0; i < trajectory.points.back().velocities.size(); ++i)
    {
      if (fabs(trajectory.points.back().velocities.at(i)) > std::numeric_limits<float>::epsilon())
      {
        RCLCPP_ERROR(
          get_node()->get_logger(),
          "Velocity of last trajectory point of joint %s is not zero: %.15f",
          trajectory.joint_names.at(i).c_str(), trajectory.points.back().velocities.at(i));
        return false;
      }
    }
  }

  rclcpp::Duration previous_traj_time(0ms);
  for (size_t i = 0; i < trajectory.points.size(); ++i)
  {
    if ((i > 0) && (rclcpp::Duration(trajectory.points[i].time_from_start) <= previous_traj_time))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Time between points %zu and %zu is not strictly increasing, it is %f and %f respectively",
        i - 1, i, previous_traj_time.seconds(),
        rclcpp::Duration(trajectory.points[i].time_from_start).seconds());
      return false;
    }
    previous_traj_time = trajectory.points[i].time_from_start;

    const size_t joint_count = trajectory.joint_names.size();
    const auto & points = trajectory.points;
    // This currently supports only position, velocity and acceleration inputs
    if (
      !validate_trajectory_point_field(joint_count, points[i].positions, "positions", i, false) ||
      !validate_trajectory_point_field(joint_count, points[i].velocities, "velocities", i, true) ||
      !validate_trajectory_point_field(joint_count, points[i].accelerations, "accelerations", i, true))
    {
      return false;
    }
    // reject effort entries
    if (!points[i].effort.empty())
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Trajectories with effort fields are currently not supported.");
      return false;
    }
  }
  return true;
}

bool PathFollowingController::validate_trajectory_point_field(
  size_t joint_names_size, const std::vector<double> & vector_field,
  const std::string & string_for_vector_field, size_t i, bool allow_empty) const
{
  if (allow_empty && vector_field.empty())
  {
    return true;
  }
  if (joint_names_size != vector_field.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Mismatch between joint_names size (%zu) and %s (%zu) at point #%zu.", joint_names_size,
      string_for_vector_field.c_str(), vector_field.size(), i);
    return false;
  }
  return true;
}

/*
void JointTrajectoryController::add_new_trajectory_msg(
  const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> & traj_msg)
{
  traj_msg_external_point_ptr_.writeFromNonRT(traj_msg);
}

void JointTrajectoryController::preempt_active_goal()
{
  const auto active_goal = *rt_active_goal_.readFromNonRT();
  if (active_goal)
  {
    auto action_res = std::make_shared<FollowJTrajAction::Result>();
    action_res->set__error_code(FollowJTrajAction::Result::INVALID_GOAL);
    action_res->set__error_string("Current goal cancelled due to new incoming action.");
    active_goal->setCanceled(action_res);
    rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
  }
}
*/
bool PathFollowingController::contains_interface_type(
  const std::vector<std::string> & interface_type_list, const std::string & interface_type)
{
  return std::find(interface_type_list.begin(), interface_type_list.end(), interface_type) !=
         interface_type_list.end();
}

void PathFollowingController::resize_joint_trajectory_point(
  trajectory_msgs::msg::JointTrajectoryPoint & point, size_t size)
{
  point.positions.resize(size, 0.0);
  if (has_velocity_state_interface_)
  {
    point.velocities.resize(size, 0.0);
  }
  if (has_acceleration_state_interface_)
  {
    point.accelerations.resize(size, 0.0);
  }
}

void PathFollowingController::resize_joint_trajectory_point_command(
  trajectory_msgs::msg::JointTrajectoryPoint & point, size_t size)
{
  if (has_position_command_interface_)
  {
    point.positions.resize(size, 0.0);
  }
  if (has_velocity_command_interface_)
  {
    point.velocities.resize(size, 0.0);
  }
  if (has_acceleration_command_interface_)
  {
    point.accelerations.resize(size, 0.0);
  }
  if (has_effort_command_interface_)
  {
    point.effort.resize(size, 0.0);
  }
}

bool PathFollowingController::read_state_from_command_interfaces(JointTrajectoryPoint & state)
{
  bool has_values = true;

  auto assign_point_from_interface =
    [&](std::vector<double> & trajectory_point_interface, const auto & joint_interface)
  {
    for (size_t index = 0; index < dof_; ++index)
      trajectory_point_interface[index] = joint_interface[index].get().get_value();
  };

  auto interface_has_values = [](const auto & joint_interface)
  {
    return std::find_if(
             joint_interface.begin(), joint_interface.end(),
             [](const auto & interface)
             { return std::isnan(interface.get().get_value()); }) == joint_interface.end();
  };

  // Assign values from the command interfaces as state. Therefore needs check for both.
  // Position state interface has to exist always
  if (has_position_command_interface_ && interface_has_values(joint_command_interface_[0]))
  {
    assign_point_from_interface(state.positions, joint_command_interface_[0]);
  }
  else
  {
    state.positions.clear();
    has_values = false;
  }
  // velocity and acceleration states are optional
  if (has_velocity_state_interface_)
  {
    if (has_velocity_command_interface_ && interface_has_values(joint_command_interface_[1]))
    {
      assign_point_from_interface(state.velocities, joint_command_interface_[1]);
    }
    else
    {
      state.velocities.clear();
      has_values = false;
    }
  }
  else
  {
    state.velocities.clear();
  }
  // Acceleration is used only in combination with velocity
  if (has_acceleration_state_interface_)
  {
    if (has_acceleration_command_interface_ && interface_has_values(joint_command_interface_[2]))
    {
      assign_point_from_interface(state.accelerations, joint_command_interface_[2]);
    }
    else
    {
      state.accelerations.clear();
      has_values = false;
    }
  }
  else
  {
    state.accelerations.clear();
  }

  return has_values;
}

void PathFollowingController::read_state_from_state_interfaces(JointTrajectoryPoint & state)
{
  auto assign_point_from_interface =
    [&](std::vector<double> & trajectory_point_interface, const auto & joint_interface)
  {
    for (size_t index = 0; index < dof_; ++index)
    {
      trajectory_point_interface[index] = joint_interface[index].get().get_value();
    }
  };

  // Assign values from the hardware
  // Position states always exist
  assign_point_from_interface(state.positions, joint_state_interface_[0]);
  // velocity and acceleration states are optional
  if (has_velocity_state_interface_)
  {
    assign_point_from_interface(state.velocities, joint_state_interface_[1]);
    // Acceleration is used only in combination with velocity
    if (has_acceleration_state_interface_)
    {
      assign_point_from_interface(state.accelerations, joint_state_interface_[2]);
    }
    else
    {
      // Make empty so the property is ignored during interpolation
      state.accelerations.clear();
    }
  }
  else
  {
    // Make empty so the property is ignored during interpolation
    state.velocities.clear();
    state.accelerations.clear();
  }
}

// CALLBACKS -----------------------------------------------------

void PathFollowingController::subscriber_callback(
  const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> msg)
{
  /*
  if (!validate_trajectory_msg(*msg))
  {
    return;
  }
  // http://wiki.ros.org/joint_trajectory_controller/UnderstandingTrajectoryReplacement
  // always replace old msg with new one for now
  if (subscriber_is_active_)
  {
    add_new_trajectory_msg(msg);
    rt_is_holding_.writeFromNonRT(false);
  }
  */
};

rclcpp_action::GoalResponse PathFollowingController::goal_received_callback(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const ActionType::Goal> goal)
{
  RCLCPP_INFO(get_node()->get_logger(), "Received new action goal");

  /*
  // Precondition: Running controller
  if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Can't accept new action goals. Controller is not running.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (!validate_trajectory_msg(goal->trajectory))
  {
    return rclcpp_action::GoalResponse::REJECT;
  }
  */
  RCLCPP_INFO(get_node()->get_logger(), "Accepted new action goal");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PathFollowingController::goal_cancelled_callback(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionType>> goal_handle)
{
  RCLCPP_INFO(get_node()->get_logger(), "Got request to cancel goal");
  /*
  // Check that cancel request refers to currently active goal (if any)
  const auto active_goal = *rt_active_goal_.readFromNonRT();
  if (active_goal && active_goal->gh_ == goal_handle)
  {
    RCLCPP_INFO(
      get_node()->get_logger(), "Canceling active action goal because cancel callback received.");

    // Mark the current goal as canceled
    rt_has_pending_goal_.writeFromNonRT(false);
    auto action_res = std::make_shared<FollowJTrajAction::Result>();
    active_goal->setCanceled(action_res);
    rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());

    // Enter hold current position mode
    add_new_trajectory_msg(set_hold_position());
  }
  */
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PathFollowingController::goal_accepted_callback(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionType>> goal_handle)
{
  /*
  // mark a pending goal
  rt_has_pending_goal_.writeFromNonRT(true);

  // Update new trajectory
  {
    preempt_active_goal();
    auto traj_msg =
      std::make_shared<trajectory_msgs::msg::JointTrajectory>(goal_handle->get_goal()->trajectory);

    add_new_trajectory_msg(traj_msg);
    rt_is_holding_.writeFromNonRT(false);
  }

  // Update the active goal
  RealtimeGoalHandlePtr rt_goal = std::make_shared<RealtimeGoalHandle>(goal_handle);
  rt_goal->preallocated_feedback_->joint_names = params_.joints;
  rt_goal->execute();
  rt_active_goal_.writeFromNonRT(rt_goal);

  // Set smartpointer to expire for create_wall_timer to delete previous entry from timer list
  goal_handle_timer_.reset();

  // Setup goal status checking timer
  goal_handle_timer_ = get_node()->create_wall_timer(
    action_monitor_period_.to_chrono<std::chrono::nanoseconds>(),
    std::bind(&RealtimeGoalHandle::runNonRealtime, rt_goal));
  */
}

void PathFollowingController::query_state_service(
  const std::shared_ptr<QueryStateType::Request> request,
  std::shared_ptr<QueryStateType::Response> response)
{
  /*
  const auto logger = get_node()->get_logger();
  // Preconditions
  if (get_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    RCLCPP_ERROR(logger, "Can't sample trajectory. Controller is not active.");
    response->success = false;
    return;
  }
  const auto active_goal = *rt_active_goal_.readFromRT();
  response->name = params_.joints;
  trajectory_msgs::msg::JointTrajectoryPoint state_requested = state_current_;
  if (has_active_trajectory())
  {
    TrajectoryPointConstIter start_segment_itr, end_segment_itr;
    response->success = traj_external_point_ptr_->sample(
      static_cast<rclcpp::Time>(request->time), interpolation_method_, state_requested,
      start_segment_itr, end_segment_itr);
    // If the requested sample time precedes the trajectory finish time respond as failure
    if (response->success)
    {
      if (end_segment_itr == traj_external_point_ptr_->end())
      {
        RCLCPP_ERROR(logger, "Requested sample time precedes the current trajectory end time.");
        response->success = false;
      }
    }
    else
    {
      RCLCPP_ERROR(
        logger, "Requested sample time is earlier than the current trajectory start time.");
    }
  }
  else
  {
    RCLCPP_ERROR(logger, "Currently there is no valid trajectory instance.");
    response->success = false;
  }
  response->position = state_requested.positions;
  response->velocity = state_requested.velocities;
  response->acceleration = state_requested.accelerations;
  */
}

}  // namespace path_following_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  path_following_controller::PathFollowingController, controller_interface::ControllerInterface)
