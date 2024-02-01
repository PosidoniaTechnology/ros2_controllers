// Copyright (c) 2024, Posidonia Technology d.o.o.
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#ifndef PATH_FOLLOWING_CONTROLLER__PATH_FOLLOWING_CONTROLLER_HPP_
#define PATH_FOLLOWING_CONTROLLER__PATH_FOLLOWING_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "realtime_tools/realtime_server_goal_handle.h"
#include "rclcpp_action/server.hpp"
#include "path_following_controller_parameters.hpp"
#include "path_following_controller/visibility_control.h"
#include "path_following_controller/tolerances.hpp"
#include "path_following_controller/trajectory.hpp"

#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"

#include "std_srvs/srv/set_bool.hpp"
#include "control_msgs/srv/query_trajectory_state.hpp"

#include "control_msgs/action/follow_joint_trajectory.hpp"

using namespace std::chrono_literals;  // NOLINT

namespace path_following_controller
{

using JointTrajectoryPoint = trajectory_msgs::msg::JointTrajectoryPoint;
using JointTrajectory = trajectory_msgs::msg::JointTrajectory;

class PathFollowingController : public controller_interface::ControllerInterface
{
public:
  PATH_FOLLOWING_CONTROLLER__VISIBILITY_PUBLIC
  PathFollowingController();

  PATH_FOLLOWING_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  PATH_FOLLOWING_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  PATH_FOLLOWING_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  PATH_FOLLOWING_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  PATH_FOLLOWING_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  PATH_FOLLOWING_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  PATH_FOLLOWING_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  // Degrees of freedom
  size_t dof_;
  std::vector<std::string> command_joint_names_;

  std::shared_ptr<path_following_controller::ParamListener> param_listener_;
  path_following_controller::Params params_;

  std::vector<std::string> state_joints_;

  std::shared_ptr<Trajectory> traj_external_point_ptr_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<JointTrajectory>>
    traj_msg_external_point_ptr_;

  std::shared_ptr<JointTrajectory> hold_position_msg_ptr_ = nullptr;

  // Preallocate variables used in the realtime update() function
  trajectory_msgs::msg::JointTrajectoryPoint state_current_;
  trajectory_msgs::msg::JointTrajectoryPoint command_current_;
  trajectory_msgs::msg::JointTrajectoryPoint state_desired_;
  trajectory_msgs::msg::JointTrajectoryPoint state_error_;
  trajectory_msgs::msg::JointTrajectoryPoint last_commanded_state_;

  // Should position errors get wrapped around for i-th joint?
  std::vector<bool> joints_angle_wraparound_;

  // Tolerances for a given trajectory segment
  SegmentTolerances default_tolerances_;

  // HARDWARE INTERFACES

  // To reduce number of variables and to make the code shorter the interfaces are ordered in types
  // as the following constants
  const std::vector<std::string> allowed_interface_types_ = {
    hardware_interface::HW_IF_POSITION,
    hardware_interface::HW_IF_VELOCITY,
    hardware_interface::HW_IF_ACCELERATION,
    hardware_interface::HW_IF_EFFORT,
  };
  // The interfaces are defined as the types in 'allowed_interface_types_' member.
  // For convenience, for each type the interfaces are ordered so that i-th position
  // matches i-th index in joint_names_
  template <typename T>
  using InterfaceReferences = std::vector<std::vector<std::reference_wrapper<T>>>;
  InterfaceReferences<hardware_interface::LoanedCommandInterface> joint_command_interface_;
  InterfaceReferences<hardware_interface::LoanedStateInterface> joint_state_interface_;

  bool has_position_state_interface_ = false;
  bool has_velocity_state_interface_ = false;
  bool has_acceleration_state_interface_ = false;
  //   no effort state interface
  bool has_position_command_interface_ = false;
  bool has_velocity_command_interface_ = false;
  bool has_acceleration_command_interface_ = false;
  bool has_effort_command_interface_ = false;

  void read_state_from_state_interfaces(JointTrajectoryPoint & state);
  /** Assign values from the command interfaces as state.
   * This is only possible if command AND state interfaces exist for the same type,
   *  therefore needs check for both.
   * @param[out] state to be filled with values from command interfaces.
   * @return true if all interfaces exists and contain non-NaN values, false otherwise.
   */
  bool read_state_from_command_interfaces(JointTrajectoryPoint & state);


  // SUBSCRIBERS
  bool subscriber_is_active_ = false;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_command_subscriber_ =
    nullptr;
  // callback for topic interface
  void subscriber_callback(const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> msg);
  
  // PUBLISHERS
  using ControllerStateMsg = control_msgs::msg::JointTrajectoryControllerState;
  rclcpp::Publisher<ControllerStateMsg>::SharedPtr controller_state_publisher_;
  rclcpp::Duration state_publisher_period_ = rclcpp::Duration(20ms);
  rclcpp::Time last_state_publish_time_;
  using RealtimePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;
  std::unique_ptr<RealtimePublisher> rt_publisher_;

  // SERVICES
  using QueryStateType = control_msgs::srv::QueryTrajectoryState;
  rclcpp::Service<QueryStateType>::SharedPtr query_state_service_;
  // service callback
  void query_state_service(
    const std::shared_ptr<control_msgs::srv::QueryTrajectoryState::Request> request,
    std::shared_ptr<control_msgs::srv::QueryTrajectoryState::Response> response);

  // ACTIONS
  using ActionType = control_msgs::action::FollowJointTrajectory;
  using RealtimeGoalHandle = realtime_tools::RealtimeServerGoalHandle<ActionType>;
  using RealtimeGoalHandlePtr = std::shared_ptr<RealtimeGoalHandle>;
  using RealtimeGoalHandleBuffer = realtime_tools::RealtimeBuffer<RealtimeGoalHandlePtr>;

  rclcpp_action::Server<ActionType>::SharedPtr action_server_;
  RealtimeGoalHandleBuffer rt_active_goal_;  ///< Currently active action goal, if any.
  realtime_tools::RealtimeBuffer<bool> rt_goal_pending_;  ///< Is there a pending action goal?
  rclcpp::TimerBase::SharedPtr goal_handle_timer_;
  rclcpp::Duration action_monitor_period_ = rclcpp::Duration(50ms);

  // action server callbacks
  rclcpp_action::GoalResponse goal_received_callback(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ActionType::Goal> goal);
  rclcpp_action::CancelResponse goal_cancelled_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionType>> goal_handle);
  void goal_accepted_callback(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionType>> goal_handle);

    

private:
  // UTILS
  bool has_active_trajectory() const;
  bool validate_trajectory_msg(const JointTrajectory & trajectory) const;
  bool validate_trajectory_point_field(
    size_t joint_names_size, const std::vector<double> & vector_field,
    const std::string & string_for_vector_field, size_t i, bool allow_empty) const;

  // fill trajectory_msg so it matches joints controlled by this controller
  // positions set to current position, velocities, accelerations and efforts to 0.0
  void fill_partial_goal(
    std::shared_ptr<JointTrajectory> trajectory_msg) const;
  // incoming message might have a different joint order, sort it to local one
  void sort_to_local_joint_order(
    std::shared_ptr<JointTrajectory> trajectory_msg);
  void preempt_active_goal();
  std::shared_ptr<JointTrajectory> set_success_trajectory_point();

  void init_hold_position_msg();
  /** @brief set the current position with zero velocity and acceleration as new command
   */
  std::shared_ptr<JointTrajectory> set_hold_position();
  // True if holding position or repeating last trajectory point in case of success
  realtime_tools::RealtimeBuffer<bool> rt_is_holding_;


  void add_new_trajectory_msg(
    const std::shared_ptr<JointTrajectory> & traj_msg);
  bool contains_interface_type(
  const std::vector<std::string> & interface_type_list, const std::string & interface_type);
  void resize_joint_trajectory_point(
  JointTrajectoryPoint & point, size_t size);
  void resize_joint_trajectory_point_command(
  JointTrajectoryPoint & point, size_t size);
};

}  // namespace path_following_controller

#endif  // PATH_FOLLOWING_CONTROLLER__PATH_FOLLOWING_CONTROLLER_HPP_
