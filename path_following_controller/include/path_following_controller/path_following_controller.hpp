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


namespace path_following_controller
{

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
  using JointTrajectoryPoint = trajectory_msgs::msg::JointTrajectoryPoint;
  using JointTrajectory = trajectory_msgs::msg::JointTrajectory;

  // Degrees of freedom
  size_t dof_;
  std::vector<std::string> command_joint_names_;

  std::shared_ptr<path_following_controller::ParamListener> param_listener_;
  path_following_controller::Params params_;

  std::shared_ptr<Trajectory> current_trajectory_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<JointTrajectory>>
    rt_new_trajectory_msg_;

  std::shared_ptr<JointTrajectory> hold_position_msg_ptr_ = nullptr;
  realtime_tools::RealtimeBuffer<bool> rt_is_holding_;

  // Preallocate variables used in the realtime update() function
  JointTrajectoryPoint state_current_;
  JointTrajectoryPoint state_desired_;
  JointTrajectoryPoint state_error_;

  // Tolerances for a given trajectory segment
  SegmentTolerances default_tolerances_;

  // HARDWARE INTERFACES
  // For convenience, interface types are ordered so that i-th position
  // matches i-th index in joint_names_
  const std::vector<std::string> allowed_command_interface_types_ = {
    hardware_interface::HW_IF_POSITION
  };
  const std::vector<std::string> allowed_state_interface_types_ = {
    hardware_interface::HW_IF_POSITION,
    hardware_interface::HW_IF_VELOCITY,
    hardware_interface::HW_IF_ACCELERATION,
  };
  template <typename T>
  using InterfaceReferences = std::vector<std::vector<std::reference_wrapper<T>>>;
  InterfaceReferences<hardware_interface::LoanedCommandInterface> joint_command_interface_;
  InterfaceReferences<hardware_interface::LoanedStateInterface> joint_state_interface_;

  bool has_position_state_interface_ = false;
  bool has_velocity_state_interface_ = false;
  bool has_acceleration_state_interface_ = false;
  //   no effort state interface
  bool has_position_command_interface_ = false;

  // SUBSCRIBERS
  bool topic_goal_received_ = false;
  // Subscriber for goals recieved through topics
  rclcpp::Subscription<JointTrajectory>::SharedPtr joint_command_subscriber_ = nullptr;
  // callback for topic interface
  void subscriber_callback(const std::shared_ptr<JointTrajectory> msg);
  
  // PUBLISHERS
  using ControllerStateMsg = control_msgs::msg::JointTrajectoryControllerState;
  rclcpp::Publisher<ControllerStateMsg>::SharedPtr controller_state_publisher_;
  rclcpp::Duration state_publisher_period_ = rclcpp::Duration::from_seconds(0.02);
  rclcpp::Time last_state_publish_time_;
  using RealtimePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;
  std::unique_ptr<RealtimePublisher> rt_publisher_;

  // ACTIONS
  using ActionType = control_msgs::action::FollowJointTrajectory;
  using RealtimeGoalHandle = realtime_tools::RealtimeServerGoalHandle<ActionType>;
  using RealtimeGoalHandlePtr = std::shared_ptr<RealtimeGoalHandle>;
  using RealtimeGoalHandleBuffer = realtime_tools::RealtimeBuffer<RealtimeGoalHandlePtr>;

  rclcpp_action::Server<ActionType>::SharedPtr action_server_;
  RealtimeGoalHandleBuffer rt_active_goal_;  ///< Currently active action goal, if any.
  realtime_tools::RealtimeBuffer<bool> rt_action_goal_pending_;  ///< Is there a pending action goal?
  rclcpp::TimerBase::SharedPtr goal_handle_timer_;
  rclcpp::Duration action_monitor_period_ = rclcpp::Duration::from_seconds(0.05);

  // action server callbacks
  rclcpp_action::GoalResponse goal_received_callback(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ActionType::Goal> goal);
  rclcpp_action::CancelResponse goal_cancelled_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionType>> goal_handle);
  void goal_accepted_callback(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionType>> goal_handle);

  // UTILS
  void compute_error_for_joint(JointTrajectoryPoint & error, int index,
                                   const JointTrajectoryPoint & current,
                                   const JointTrajectoryPoint & desired);
  bool has_active_trajectory() const;
  bool validate_trajectory_msg(const JointTrajectory & trajectory) const;

private:
  // UTILS
  void read_state_from_state_interfaces(JointTrajectoryPoint & state);
  bool validate_trajectory_point_field(
    size_t joint_names_size, const std::vector<double> & vector_field,
    const std::string & string_for_vector_field, size_t i, bool allow_empty) const;

  void publish_state(const JointTrajectoryPoint & desired_state, 
                     const JointTrajectoryPoint & current_state,
                     const JointTrajectoryPoint & state_error);

  // fill trajectory_msg so it matches joints controlled by this controller
  // positions set to current position, velocities, accelerations and efforts to 0.0
  void fill_partial_goal( 
    std::shared_ptr<JointTrajectory> trajectory_msg) const;
  // incoming message might have a different joint order, sort it to local one
  void sort_to_local_joint_order(
    std::shared_ptr<JointTrajectory> trajectory_msg);
  void preempt_active_goal();

  void init_rt_publisher_msg();
  void init_hold_msg();
  /** @brief set the current position with zero velocity and acceleration as new command
   */
  std::shared_ptr<JointTrajectory> hold_msg();
  std::shared_ptr<JointTrajectory> success_msg();

  // Writes message to realtime buffer
  inline void set_new_trajectory_msg(
    const std::shared_ptr<JointTrajectory> & traj_msg){ rt_new_trajectory_msg_.writeFromNonRT(traj_msg); }

  // Reads message from realtime buffer
  inline std::shared_ptr<JointTrajectory> get_new_trajectory_msg(){ return *rt_new_trajectory_msg_.readFromRT(); }

  bool contains_interface_type(
  const std::vector<std::string> & interface_type_list, const std::string & interface_type);
  void resize_joint_trajectory_point(
  JointTrajectoryPoint & point, size_t size);
};

}  // namespace path_following_controller

#endif  // PATH_FOLLOWING_CONTROLLER__PATH_FOLLOWING_CONTROLLER_HPP_
