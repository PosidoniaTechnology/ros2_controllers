// Copyright (c) 2024, Nikola Banovic
// Copyright (c) 2024, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "sequential_joint_trajectory_controller/sequential_joint_trajectory_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"

namespace sequential_joint_trajectory_controller
{
controller_interface::CallbackReturn SequentialJointTrajectoryController::on_init()
{
  // Create the parameter listener and get the parameters
  sequential_param_listener_ = std::make_shared<sequential_joint_trajectory_controller::ParamListener>(get_node());
  sequential_params_ = sequential_param_listener_->get_params();
  return JointTrajectoryController::on_init();
}

controller_interface::InterfaceConfiguration SequentialJointTrajectoryController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf = JointTrajectoryController::state_interface_configuration();
  //conf.names.push_back(sequential_params_.MY_PARAM); add state interfaces like this if necessary
  return conf;
}

controller_interface::CallbackReturn SequentialJointTrajectoryController::on_activate(const rclcpp_lifecycle::State& state)
{
  TimeData time_data;
  time_data.time = get_node()->now();
  time_data.period = rclcpp::Duration::from_nanoseconds(0);
  time_data.uptime = get_node()->now();
  time_data_.initRT(time_data);
  return JointTrajectoryController::on_activate(state);
}

controller_interface::return_type SequentialJointTrajectoryController::update(const rclcpp::Time& time,
                                                                          const rclcpp::Duration& period)
{
  if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    return controller_interface::return_type::OK;
  }
  // update dynamic parameters
  if (param_listener_->is_old(params_))
  {
    params_ = param_listener_->get_params();
    default_tolerances_ = get_segment_tolerances(params_);
  }

  auto compute_error_for_joint = [&](
                                   JointTrajectoryPoint & error, int index,
                                   const JointTrajectoryPoint & current,
                                   const JointTrajectoryPoint & desired)
  {
    // error defined as the difference between current and desired
    if (joints_angle_wraparound_[index])
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

  // don't update goal after we sampled the trajectory to avoid any racecondition
  const auto active_goal = *rt_active_goal_.readFromRT();

  // Check if a new external message has been received from nonRT threads
  auto current_external_msg = traj_external_point_ptr_->get_trajectory_msg();
  auto new_external_msg = traj_msg_external_point_ptr_.readFromRT();
  // Discard, if a goal is pending but still not active (somewhere stuck in goal_handle_timer_)
  if (current_external_msg != *new_external_msg && (*(rt_has_pending_goal_.readFromRT()) && !active_goal) == false) {
    fill_partial_goal(*new_external_msg);
    sort_to_local_joint_order(*new_external_msg);
    // TODO(denis): Add here integration of position and velocity
    traj_external_point_ptr_->update(*new_external_msg);
  }

  // TODO(anyone): can I here also use const on joint_interface since the reference_wrapper is not
  // changed, but its value only?
  auto assign_interface_from_point = [&](auto& joint_interface, const std::vector<double>& trajectory_point_interface) {
    for (size_t index = 0; index < dof_; ++index) {
      joint_interface[index].get().set_value(trajectory_point_interface[index]);
    }
  };

  // current state update
  state_current_.time_from_start.set__sec(0);
  read_state_from_state_interfaces(state_current_);


  // currently carrying out a trajectory
  if (has_active_trajectory()) 
  {
    TimeData time_data;
    time_data.time = time;
    rcl_duration_value_t nsec_period = period.nanoseconds();

    // if tolerance was violated in the previous step, scale by 0.0,
    // effectively "pausing" the internal clock of the controller for this cycle
    double scaling_factor;
    if(tolerance_violated_while_moving_){
      scaling_factor = 0.0;
      tolerance_violated_while_moving_ = false;
    }else{
      scaling_factor = 1.0;
    }
    time_data.period = rclcpp::Duration::from_nanoseconds(scaling_factor * nsec_period);
    time_data.uptime = time_data_.readFromRT()->uptime + time_data.period;
    rclcpp::Time traj_time = time_data.uptime;
    time_data_.writeFromNonRT(time_data);

    bool first_sample = false;
    // if sampling the first time, set the point before you sample
    if (!traj_external_point_ptr_->is_sampled_already()) {
      first_sample = true;
      if (params_.open_loop_control) {
        traj_external_point_ptr_->set_point_before_trajectory_msg(traj_time, last_commanded_state_);
      } else {
        traj_external_point_ptr_->set_point_before_trajectory_msg(traj_time, state_current_);
      }
    }

    // find segment for current timestamp
    joint_trajectory_controller::TrajectoryPointConstIter start_segment_itr, end_segment_itr;
    const bool valid_point = traj_external_point_ptr_->sample(traj_time, interpolation_method_, state_desired_,
                                                              start_segment_itr, end_segment_itr);

    if (valid_point) {
      const rclcpp::Time traj_start = traj_external_point_ptr_->time_from_start();
      // this is the time instance
      // - started with the first segment: when the first point will be reached (in the future)
      // - later: when the point of the current segment was reached
      const rclcpp::Time segment_time_from_start = traj_start + start_segment_itr->time_from_start;
      // time_difference is
      // - negative until first point is reached
      // - counting from zero to time_from_start of next point
      double time_difference = time.seconds() - segment_time_from_start.seconds();

      bool outside_goal_tolerance = false;
      bool within_goal_time = true;
      const bool before_last_point = end_segment_itr != traj_external_point_ptr_->end();

      // have we reached the end, are not holding position, and is a timeout configured?
      // Check independently of other tolerances
      if (!before_last_point && *(rt_is_holding_.readFromRT()) == false && cmd_timeout_ > 0.0 &&
          time_difference > cmd_timeout_) {
        RCLCPP_WARN(get_node()->get_logger(), "Aborted due to command timeout");

        traj_msg_external_point_ptr_.reset();
        traj_msg_external_point_ptr_.initRT(set_hold_position());
      }

      // Check state/goal tolerance
      for (size_t index = 0; index < dof_; ++index) {
        compute_error_for_joint(state_error_, index, state_current_, state_desired_);

        // Always check the state tolerance on the first sample in case the first sample
        // is the last point       
        if ((before_last_point || first_sample) &&
            !check_state_tolerance_per_joint(state_error_, index, default_tolerances_.state_tolerance[index], false) &&
            *(rt_is_holding_.readFromRT()) == false) {
          tolerance_violated_while_moving_ = true;
          //RCLCPP_WARN(get_node()->get_logger(), "%s state tolerance violated.", params_.joints[index].c_str());
          //RCLCPP_WARN(get_node()->get_logger(), "error: %f", state_error_.positions[index]);
        }

        // past the final point, check that we end up inside goal tolerance
        if (!before_last_point &&
            !check_state_tolerance_per_joint(state_error_, index, default_tolerances_.goal_state_tolerance[index],
                                             false) &&
            *(rt_is_holding_.readFromRT()) == false) {
          outside_goal_tolerance = true;

          if (default_tolerances_.goal_time_tolerance != 0.0) {
            if (time_difference > default_tolerances_.goal_time_tolerance) {
              //RCLCPP_WARN(get_node()->get_logger(), "goal time tolerance violated for this segment");
              //RCLCPP_WARN(get_node()->get_logger(), "Time: %f", time.seconds());
              //RCLCPP_WARN(get_node()->get_logger(), "Uptime: %f", time_data_.readFromRT()->uptime.seconds());
              //RCLCPP_WARN(get_node()->get_logger(), "time - segment_time_from_start: %f", time_data_.readFromRT()->uptime.seconds());
              within_goal_time = false;
            }
          }
        }
      }

      // set values for next hardware write() if tolerance is met
      if (within_goal_time) {
        if (use_closed_loop_pid_adapter_) {
          // Update PIDs
          for (auto i = 0ul; i < dof_; ++i) {
            tmp_command_[i] = (state_desired_.velocities[i] * ff_velocity_scale_[i]) +
                              pids_[i]->computeCommand(state_error_.positions[i], state_error_.velocities[i],
                                                       (uint64_t)period.nanoseconds());
          }
        }

        // set values for next hardware write()
        if (has_position_command_interface_)
          assign_interface_from_point(joint_command_interface_[0], state_desired_.positions);

        if (has_velocity_command_interface_) {
          if (use_closed_loop_pid_adapter_)
            assign_interface_from_point(joint_command_interface_[1], tmp_command_);
          else 
            assign_interface_from_point(joint_command_interface_[1], state_desired_.velocities);
        }
        
        if (has_acceleration_command_interface_)
          assign_interface_from_point(joint_command_interface_[2], state_desired_.accelerations);

        if (has_effort_command_interface_)
          assign_interface_from_point(joint_command_interface_[3], tmp_command_);

        // store the previous command. Used in open-loop control mode
        last_commanded_state_ = state_desired_;
      }

      if (active_goal) {
        // send feedback
        auto feedback = std::make_shared<FollowJTrajAction::Feedback>();
        feedback->header.stamp = time;
        feedback->joint_names = params_.joints;

        feedback->actual = state_current_;
        feedback->desired = state_desired_;
        feedback->error = state_error_;
        active_goal->setFeedback(feedback);

        // check abort
        if (tolerance_violated_while_moving_) {
          /*
          auto result = std::make_shared<FollowJTrajAction::Result>();
          result->set__error_code(FollowJTrajAction::Result::PATH_TOLERANCE_VIOLATED);
          active_goal->setAborted(result);
          // TODO(matthew-reynolds): Need a lock-free write here
          // See https://github.com/ros-controls/ros2_controllers/issues/168
          rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
          rt_has_pending_goal_.writeFromNonRT(false);

          RCLCPP_WARN(get_node()->get_logger(), "Aborted due to state tolerance violation");

          traj_msg_external_point_ptr_.reset();
          traj_msg_external_point_ptr_.initRT(set_hold_position());

          int i = 0;
          for (auto &&position : state_error_.positions){
            RCLCPP_WARN(get_node()->get_logger(), "Joint%d error: %f", i, position);
            i++;
          }
          */ 


        } else if (!before_last_point) {
          if (!outside_goal_tolerance) {
            auto res = std::make_shared<FollowJTrajAction::Result>();
            res->set__error_code(FollowJTrajAction::Result::SUCCESSFUL);
            active_goal->setSucceeded(res);
            // TODO(matthew-reynolds): Need a lock-free write here
            // See https://github.com/ros-controls/ros2_controllers/issues/168
            rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
            rt_has_pending_goal_.writeFromNonRT(false);

            RCLCPP_INFO(get_node()->get_logger(), "Goal reached, success!");
            RCLCPP_WARN(get_node()->get_logger(), "Execution delay: %.3fs", time.seconds() - time_data_.readFromRT()->uptime.seconds());

            traj_msg_external_point_ptr_.reset();
            traj_msg_external_point_ptr_.initRT(set_success_trajectory_point());
          } else if (!within_goal_time) {
            auto result = std::make_shared<FollowJTrajAction::Result>();
            result->set__error_code(FollowJTrajAction::Result::GOAL_TOLERANCE_VIOLATED);
            active_goal->setAborted(result);
            // TODO(matthew-reynolds): Need a lock-free write here
            // See https://github.com/ros-controls/ros2_controllers/issues/168
            rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
            rt_has_pending_goal_.writeFromNonRT(false);

            RCLCPP_WARN(get_node()->get_logger(), "Aborted due goal_time_tolerance exceeding by %f seconds",
                        time_difference);

            traj_msg_external_point_ptr_.reset();
            traj_msg_external_point_ptr_.initRT(set_hold_position());
          }
        }
      } else if (tolerance_violated_while_moving_ && *(rt_has_pending_goal_.readFromRT()) == false) {
        // we need to ensure that there is no pending goal -> we get a race condition otherwise
        RCLCPP_ERROR(get_node()->get_logger(), "Holding position due to state tolerance violation");

        traj_msg_external_point_ptr_.reset();
        traj_msg_external_point_ptr_.initRT(set_hold_position());
      } else if (!before_last_point && !within_goal_time && *(rt_has_pending_goal_.readFromRT()) == false) {
        RCLCPP_ERROR(get_node()->get_logger(), "Exceeded goal_time_tolerance: holding position...");

        traj_msg_external_point_ptr_.reset();
        traj_msg_external_point_ptr_.initRT(set_hold_position());
      }
      // else, run another cycle while waiting for outside_goal_tolerance
      // to be satisfied (will stay in this state until new message arrives)
      // or outside_goal_tolerance violated within the goal_time_tolerance
    }
  }

  publish_state(state_desired_, state_current_, state_error_);
  return controller_interface::return_type::OK;
}

}  // namespace sequential_joint_trajectory_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  sequential_joint_trajectory_controller::SequentialJointTrajectoryController, controller_interface::ControllerInterface)
