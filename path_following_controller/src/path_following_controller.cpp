// Copyright (c) 2024, Posidonia Technology d.o.o.
// All rights reserved.
//
// 
// Apache License, Version 2.0
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include "path_following_controller/path_following_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "angles/angles.h"
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
    return controller_interface::return_type::OK;
  
  // update dynamic parameters
  if (param_listener_->is_old(params_))
  {
    params_ = param_listener_->get_params();
    default_tolerances_ = get_segment_tolerances(params_);
  }

  ///////////////// GET OR REPLACE CURRENT GOAL
  // get active goal from the action server
  const auto active_goal = *rt_active_goal_.readFromRT();

  // REVIEW: rewrite this logic in a nicer way, get rid of dereferencing
  // check if there is a new goal message
  auto current_external_msg = traj_external_point_ptr_->get_trajectory_msg();
  auto new_external_msg = traj_msg_external_point_ptr_.readFromRT();    

  // Discard action goal if it is pending, but not active (still somewhere in goal_handle_timer_)
  // or if there was a new goal received, but from topic
  if(
    current_external_msg != *new_external_msg 
    && (*(rt_action_goal_pending_.readFromRT()) && !active_goal) == false)
  {
    fill_partial_goal(*new_external_msg);
    sort_to_local_joint_order(*new_external_msg);
    traj_external_point_ptr_->update(*new_external_msg);   // resets trajectory index to 0
  }


  ///////////////// UPDATE CURRENT STATE
  read_state_from_state_interfaces(state_current_);


  if(has_active_trajectory())
  {    
    bool is_tolerance_violated = false;

    //////// SAMPLE NEXT TRAJECTORY
    traj_external_point_ptr_->sample(state_desired_);

    ///////////////// WRITE TO HARDWARE
    auto assign_interface_from_point =
    [&](const auto & command_interface, const std::vector<double> & trajectory_point)
    {
      for (size_t index = 0; index < dof_; ++index)
        command_interface[index].get().set_value(trajectory_point[index]);
    };
    if (has_position_command_interface_)
      assign_interface_from_point(joint_command_interface_[0], state_desired_.positions);
    if (has_velocity_command_interface_)
      assign_interface_from_point(joint_command_interface_[1], state_desired_.velocities);
    if (has_acceleration_command_interface_)
      assign_interface_from_point(joint_command_interface_[2], state_desired_.accelerations);

    ///////////////// COMPUTE ERROR
    for (size_t index = 0; index < dof_; ++index)
      compute_error_for_joint(state_error_, index, state_current_, state_desired_);
    
    /////////// CHECK TOLERANCES
    if (*(rt_is_holding_.readFromRT()) == false)
      is_tolerance_violated = !check_waypoint_tolerance(dof_, state_error_, default_tolerances_.waypoint);

    ///// INCREMENT?
    if(!is_tolerance_violated)
      traj_external_point_ptr_->increment();

    ///////////////// FEEDBACK
    if (active_goal)
    {
      auto feedback = std::make_shared<ActionType::Feedback>();
      feedback->header.stamp = time;
      feedback->joint_names = params_.joints;

      feedback->actual = state_current_;
      feedback->desired = state_desired_;
      feedback->error = state_error_;
      active_goal->setFeedback(feedback);

      if (traj_external_point_ptr_->is_completed())
      {
        auto action_res = std::make_shared<ActionType::Result>();
        action_res->set__error_code(ActionType::Result::SUCCESSFUL);
        active_goal->setSucceeded(action_res);
        // TODO(matthew-reynolds): Need a lock-free write here
        // See https://github.com/ros-controls/ros2_controllers/issues/168
        rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
        rt_action_goal_pending_.writeFromNonRT(false);

        RCLCPP_INFO(get_node()->get_logger(), "Action goal reached successfully.");

        traj_msg_external_point_ptr_.reset();
        traj_msg_external_point_ptr_.initRT(set_success_trajectory_point());
      }
    }
    else if(topic_goal_received_)
    {
      if (traj_external_point_ptr_->is_completed())
      {
        RCLCPP_INFO(get_node()->get_logger(), "Topic goal reached successfully.");
        topic_goal_received_ = false;
        traj_msg_external_point_ptr_.reset();
        traj_msg_external_point_ptr_.initRT(set_success_trajectory_point());
      }
    }
  }

  publish_state(state_desired_, state_current_, state_error_);
  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn PathFollowingController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const auto LOGGER = get_node()->get_logger();

  // get degrees of freedom
  dof_ = params_.joints.size();

  command_joint_names_ = params_.command_joints;
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

  // Check if only allowed interface types are used and initialize storage to avoid memory
  // allocation during activation
  if (params_.command_interfaces.empty())
  {
    RCLCPP_ERROR(LOGGER, "'command_interfaces' parameter is empty.");
    return CallbackReturn::FAILURE;
  }

  joint_command_interface_.resize(allowed_interface_types_.size());

  has_position_command_interface_ =
    contains_interface_type(params_.command_interfaces, hardware_interface::HW_IF_POSITION);
  has_velocity_command_interface_ =
    contains_interface_type(params_.command_interfaces, hardware_interface::HW_IF_VELOCITY);
  has_acceleration_command_interface_ =
    contains_interface_type(params_.command_interfaces, hardware_interface::HW_IF_ACCELERATION);
  has_effort_command_interface_ =
    contains_interface_type(params_.command_interfaces, hardware_interface::HW_IF_EFFORT);
  
  // Check if only allowed interface types are used and initialize storage to avoid memory
  // allocation during activation
  if (params_.state_interfaces.empty())
  {
    RCLCPP_ERROR(LOGGER, "'state_interfaces' parameter is empty.");
    return CallbackReturn::FAILURE;
  }

  joint_state_interface_.resize(allowed_interface_types_.size());

  has_position_state_interface_ =
    contains_interface_type(params_.state_interfaces, hardware_interface::HW_IF_POSITION);
  has_velocity_state_interface_ =
    contains_interface_type(params_.state_interfaces, hardware_interface::HW_IF_VELOCITY);
  has_acceleration_state_interface_ =
    contains_interface_type(params_.state_interfaces, hardware_interface::HW_IF_ACCELERATION);

  
  // REVIEW AND MOVE TO UTILS
  auto get_interface_list = [](const std::vector<std::string> & interface_types)
  {
    std::stringstream ss_interfaces;
    for (size_t index = 0; index < interface_types.size(); ++index)
    {
      if (index != 0) ss_interfaces << " ";
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

  // prepare hold_position_msg
  init_hold_position_msg();

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
  
  init_rt_publisher_msg();  

  // ACTIONS
  using namespace std::placeholders;
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

  /*
  joints_angle_wraparound_.resize(dof_);
  for (size_t i = 0; i < dof_; ++i){
      const auto & gains = params_.gains.joints_map.at(params_.joints[i]);
      std::cout<<params_.gains.joints_map.at(params_.joints[i]).angle_wraparound<<std::endl;
      joints_angle_wraparound_[i] = gains.angle_wraparound;
  }
  */

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
  traj_external_point_ptr_ = std::make_shared<Trajectory>();
  traj_msg_external_point_ptr_.writeFromNonRT(
    std::shared_ptr<trajectory_msgs::msg::JointTrajectory>());

  subscriber_is_active_ = true;
  last_state_publish_time_ = get_node()->now();

  // REVIEW: Move to utils
  // Handle restart of controller by reading from commands if those are not NaN (a controller was
  // running already)
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


  // The controller should start by holding position at the beginning of active state
  add_new_trajectory_msg(set_hold_position());
  rt_is_holding_.writeFromNonRT(true);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PathFollowingController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const auto active_goal = *rt_active_goal_.readFromNonRT();
  
  if (active_goal)
  {
    rt_action_goal_pending_.writeFromNonRT(false);
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

  rt_is_holding_.writeFromNonRT(true);
  subscriber_is_active_ = false;
  traj_external_point_ptr_.reset();

  return controller_interface::CallbackReturn::SUCCESS;
}

// UTILS ----------------------------------------

void PathFollowingController::init_hold_position_msg()
{
  hold_position_msg_ptr_ = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  hold_position_msg_ptr_->header.stamp =
    rclcpp::Time(0.0, 0.0, get_node()->get_clock()->get_clock_type());  // start immediately
  hold_position_msg_ptr_->joint_names = params_.joints;
  hold_position_msg_ptr_->points.resize(1);  // a trivial msg only
  hold_position_msg_ptr_->points[0].velocities.clear();
  hold_position_msg_ptr_->points[0].accelerations.clear();
  hold_position_msg_ptr_->points[0].effort.clear();
  if (has_velocity_command_interface_ || has_acceleration_command_interface_)
  {
    // add velocity, so that trajectory sampling returns velocity points in any case
    hold_position_msg_ptr_->points[0].velocities.resize(dof_, 0.0);
  }
  if (has_acceleration_command_interface_)
  {
    // add velocity, so that trajectory sampling returns acceleration points in any case
    hold_position_msg_ptr_->points[0].accelerations.resize(dof_, 0.0);
  }
}

void PathFollowingController::init_rt_publisher_msg(){
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
}

//REVIEW: should trajectory_msg_validation be done in Trajectory class, instead of validating
// first the trajectory_msg and then the Trajectory object itself? 
// That way the message is only validated once, in the update function, when assigning new msg.
// (functions like has_nontrivial_msg(), has_trajectory_msg()...)
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

void PathFollowingController::fill_partial_goal(
  std::shared_ptr<trajectory_msgs::msg::JointTrajectory> trajectory_msg) const
{
  // REVIEW: move this to trajectory validation and fill a partial goal there.
  // joint names in the goal are a subset of existing joints, as checked in goal_callback
  // so if the size matches, no filling is needed.
  if (dof_ == trajectory_msg->joint_names.size()) 
    return;


  trajectory_msg->joint_names.reserve(dof_);

  // REVIEW: rework this logic
  for (size_t index = 0; index < dof_; ++index)
  {
    {
      if (
        std::find(
          trajectory_msg->joint_names.begin(), trajectory_msg->joint_names.end(),
          params_.joints[index]) != trajectory_msg->joint_names.end())
      {
        // joint found on msg
        continue;
      }
      trajectory_msg->joint_names.push_back(params_.joints[index]);

      for (auto & it : trajectory_msg->points)
      {
        // Assume hold position with 0 velocity and acceleration for missing joints
        if (!it.positions.empty())
        {
          if (
            has_position_command_interface_ &&
            !std::isnan(joint_command_interface_[0][index].get().get_value()))
          {
            // copy last command if cmd interface exists
            it.positions.push_back(joint_command_interface_[0][index].get().get_value());
          }
          else if (has_position_state_interface_)
          {
            // copy current state if state interface exists
            it.positions.push_back(joint_state_interface_[0][index].get().get_value());
          }
        }
        if (!it.velocities.empty())
        {
          it.velocities.push_back(0.0);
        }
        if (!it.accelerations.empty())
        {
          it.accelerations.push_back(0.0);
        }
        if (!it.effort.empty())
        {
          it.effort.push_back(0.0);
        }
      }
    }
  }
}

// REVIEW: MOVE TO UTILS. Maybe move "sort_to_local_joint_order()" along with trajectory validation to trajectory.hpp
/**
 * \return The map between \p t1 indices (implicitly encoded in return vector indices) to \p t2
 * indices. If \p t1 is <tt>"{C, B}"</tt> and \p t2 is <tt>"{A, B, C, D}"</tt>, the associated
 * mapping vector is <tt>"{2, 1}"</tt>.
 */
template <class T>
inline std::vector<size_t> mapping(const T & t1, const T & t2)
{
  // t1 must be a subset of t2
  if (t1.size() > t2.size())
  {
    return std::vector<size_t>();
  }

  std::vector<size_t> mapping_vector(t1.size());  // Return value
  for (auto t1_it = t1.begin(); t1_it != t1.end(); ++t1_it)
  {
    auto t2_it = std::find(t2.begin(), t2.end(), *t1_it);
    if (t2.end() == t2_it)
    {
      return std::vector<size_t>();
    }
    else
    {
      const size_t t1_dist = std::distance(t1.begin(), t1_it);
      const size_t t2_dist = std::distance(t2.begin(), t2_it);
      mapping_vector[t1_dist] = t2_dist;
    }
  }
  return mapping_vector;
}

void PathFollowingController::sort_to_local_joint_order(
  std::shared_ptr<trajectory_msgs::msg::JointTrajectory> trajectory_msg)
{
  // rearrange all points in the trajectory message based on mapping
  std::vector<size_t> mapping_vector = mapping(trajectory_msg->joint_names, params_.joints);
  auto remap = [this](
                 const std::vector<double> & to_remap,
                 const std::vector<size_t> & mapping) -> std::vector<double>
  {
    if (to_remap.empty())
    {
      return to_remap;
    }
    if (to_remap.size() != mapping.size())
    {
      RCLCPP_WARN(
        get_node()->get_logger(), "Invalid input size (%zu) for sorting", to_remap.size());
      return to_remap;
    }
    static std::vector<double> output(dof_, 0.0);
    // Only resize if necessary since it's an expensive operation
    if (output.size() != mapping.size())
    {
      output.resize(mapping.size(), 0.0);
    }
    for (size_t index = 0; index < mapping.size(); ++index)
    {
      auto map_index = mapping[index];
      output[map_index] = to_remap[index];
    }
    return output;
  };

  for (size_t index = 0; index < trajectory_msg->points.size(); ++index)
  {
    trajectory_msg->points[index].positions =
      remap(trajectory_msg->points[index].positions, mapping_vector);

    trajectory_msg->points[index].velocities =
      remap(trajectory_msg->points[index].velocities, mapping_vector);

    trajectory_msg->points[index].accelerations =
      remap(trajectory_msg->points[index].accelerations, mapping_vector);

    trajectory_msg->points[index].effort =
      remap(trajectory_msg->points[index].effort, mapping_vector);
  }
}

// error defined as the difference between current and desired
void PathFollowingController::compute_error_for_joint(JointTrajectoryPoint & error, int index,
                                   const JointTrajectoryPoint & current,
                                   const JointTrajectoryPoint & desired)
{
  auto wrap_around = params_.gains.joints_map.at(
                      params_.joints[index]).angle_wraparound;
  if (wrap_around)
  {
    // if desired, the shortest_angular_distance is calculated, i.e., the error is
    //  normalized between -pi<error<pi
    error.positions[index] =
      angles::shortest_angular_distance(current.positions[index], desired.positions[index]);
  }
  else
  {
    error.positions[index] = desired.positions[index] - current.positions[index];
  }
  if (
    has_velocity_state_interface_ &&
    (has_velocity_command_interface_ || has_effort_command_interface_))
  {
    error.velocities[index] = desired.velocities[index] - current.velocities[index];
  }
  if (has_acceleration_state_interface_ && has_acceleration_command_interface_)
  {
    error.accelerations[index] = desired.accelerations[index] - current.accelerations[index];
  }
};

void PathFollowingController::add_new_trajectory_msg(
  const std::shared_ptr<JointTrajectory> & traj_msg)
{
  traj_msg_external_point_ptr_.writeFromNonRT(traj_msg);
}

std::shared_ptr<JointTrajectory> PathFollowingController::set_hold_position()
{
  // Command to stay at current position
  hold_position_msg_ptr_->points[0].positions = state_current_.positions;

  // set flag, otherwise tolerances will be checked with holding position too
  rt_is_holding_.writeFromNonRT(true);

  return hold_position_msg_ptr_;
}

std::shared_ptr<JointTrajectory> PathFollowingController::set_success_trajectory_point()
{
  // set last command to be repeated at success, no matter if it has nonzero velocity or
  // acceleration
  hold_position_msg_ptr_->points[0] = traj_external_point_ptr_->get_trajectory_msg()->points.back();
  hold_position_msg_ptr_->points[0].time_from_start = rclcpp::Duration(0, 0);

  // set flag, otherwise tolerances will be checked with success_trajectory_point too
  rt_is_holding_.writeFromNonRT(true);

  return hold_position_msg_ptr_;
}

bool PathFollowingController::has_active_trajectory() const
{
  return traj_external_point_ptr_ != nullptr && 
         traj_external_point_ptr_->get_trajectory_msg() != nullptr;
}

void PathFollowingController::preempt_active_goal()
{
  const auto active_goal = *rt_active_goal_.readFromNonRT();
  if (active_goal)
  {
    auto action_res = std::make_shared<ActionType::Result>();
    action_res->set__error_code(ActionType::Result::INVALID_GOAL);
    action_res->set__error_string("Current goal cancelled due to new incoming action.");
    active_goal->setCanceled(action_res);
    rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
  }
}

void PathFollowingController::publish_state(
  const JointTrajectoryPoint & desired_state, const JointTrajectoryPoint & current_state,
  const JointTrajectoryPoint & state_error)
{
  if (state_publisher_period_.seconds() <= 0.0)
  {
    return;
  }

  if (controller_state_publisher_ && rt_publisher_->trylock())
  {
    last_state_publish_time_ = get_node()->now();
    rt_publisher_->msg_.header.stamp = last_state_publish_time_;
    rt_publisher_->msg_.reference.positions = desired_state.positions;
    rt_publisher_->msg_.reference.velocities = desired_state.velocities;
    rt_publisher_->msg_.reference.accelerations = desired_state.accelerations;
    rt_publisher_->msg_.feedback.positions = current_state.positions;
    rt_publisher_->msg_.error.positions = state_error.positions;
    if (has_velocity_state_interface_)
    {
      rt_publisher_->msg_.feedback.velocities = current_state.velocities;
      rt_publisher_->msg_.error.velocities = state_error.velocities;
    }
    if (has_acceleration_state_interface_)
    {
      rt_publisher_->msg_.feedback.accelerations = current_state.accelerations;
      rt_publisher_->msg_.error.accelerations = state_error.accelerations;
    }
    rt_publisher_->unlockAndPublish();
  }
}

// CALLBACKS -----------------------------------------------------

void PathFollowingController::subscriber_callback(
  const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> msg)
{
  if (!validate_trajectory_msg(*msg))
    return;

  // http://wiki.ros.org/joint_trajectory_controller/UnderstandingTrajectoryReplacement
  // always replace old msg with new one for now
  if (subscriber_is_active_)
  {
    topic_goal_received_ = true;
    RCLCPP_INFO(get_node()->get_logger(), "Received new message on /joint_trajectory topic");
    add_new_trajectory_msg(msg);
    rt_is_holding_.writeFromNonRT(false);
  }
};

rclcpp_action::GoalResponse PathFollowingController::goal_received_callback(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const ActionType::Goal> goal)
{
  RCLCPP_INFO(get_node()->get_logger(), "Received new action goal");

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

  RCLCPP_INFO(get_node()->get_logger(), "Accepted new action goal");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PathFollowingController::goal_cancelled_callback(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionType>> goal_handle)
{
  RCLCPP_INFO(get_node()->get_logger(), "Got request to cancel goal");

  // Check that cancel request refers to currently active goal (if any)
  const auto active_goal = *rt_active_goal_.readFromNonRT();
  if (active_goal && active_goal->gh_ == goal_handle)
  {
    RCLCPP_INFO(
      get_node()->get_logger(), "Canceling active action goal because cancel callback received.");

    //REVIEW: gather this into some remove_goal() fucntion
    // Mark the current goal as canceled
    rt_action_goal_pending_.writeFromNonRT(false);
    auto action_res = std::make_shared<ActionType::Result>();
    active_goal->setCanceled(action_res);
    rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());

    // Enter hold current position mode
    add_new_trajectory_msg(set_hold_position());
  }

  return rclcpp_action::CancelResponse::ACCEPT;
}

void PathFollowingController::goal_accepted_callback(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionType>> goal_handle)
{
  // mark a pending goal
  rt_action_goal_pending_.writeFromNonRT(true);

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

  // Setup periodical goal status schecking
  goal_handle_timer_.reset();
  goal_handle_timer_ = get_node()->create_wall_timer(
    action_monitor_period_.to_chrono<std::chrono::nanoseconds>(),
    std::bind(&RealtimeGoalHandle::runNonRealtime, rt_goal));
  
}


}  // namespace path_following_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  path_following_controller::PathFollowingController, controller_interface::ControllerInterface)
