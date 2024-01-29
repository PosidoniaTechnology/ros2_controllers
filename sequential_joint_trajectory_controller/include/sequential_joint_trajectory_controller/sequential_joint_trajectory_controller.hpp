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

#ifndef SEQUENTIAL_JOINT_TRAJECTORY_CONTROLLER__SEQUENTIAL_JOINT_TRAJECTORY_CONTROLLER_HPP_
#define SEQUENTIAL_JOINT_TRAJECTORY_CONTROLLER__SEQUENTIAL_JOINT_TRAJECTORY_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "angles/angles.h"
#include "joint_trajectory_controller/joint_trajectory_controller.hpp"
#include "joint_trajectory_controller/trajectory.hpp"
#include "controller_interface/controller_interface.hpp"
#include "sequential_joint_trajectory_controller_parameters.hpp"
#include "sequential_joint_trajectory_controller/visibility_control.h"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_srvs/srv/set_bool.hpp"

// TODO(anyone): Replace with controller specific messages
#include "lifecycle_msgs/msg/state.hpp"
#include "control_msgs/msg/joint_controller_state.hpp"
#include "control_msgs/msg/joint_jog.hpp"

namespace sequential_joint_trajectory_controller
{

class SequentialJointTrajectoryController : public joint_trajectory_controller::JointTrajectoryController
{
public:
  SequentialJointTrajectoryController() = default;
  ~SequentialJointTrajectoryController() override = default;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  CallbackReturn on_init() override;

protected:

  struct TimeData
  {
    TimeData() : time(0.0), period(rclcpp::Duration::from_nanoseconds(0.0)), uptime(0.0)
    {
    }
    rclcpp::Time time;
    rclcpp::Duration period;
    rclcpp::Time uptime;
  };

private:

  bool tolerance_violated_while_moving_ = false;
  
  realtime_tools::RealtimeBuffer<TimeData> time_data_;
  std::shared_ptr<sequential_joint_trajectory_controller::ParamListener> sequential_param_listener_;
  sequential_joint_trajectory_controller::Params sequential_params_;
};

}  // namespace sequential_joint_trajectory_controller

#endif  // SEQUENTIAL_JOINT_TRAJECTORY_CONTROLLER__SEQUENTIAL_JOINT_TRAJECTORY_CONTROLLER_HPP_
