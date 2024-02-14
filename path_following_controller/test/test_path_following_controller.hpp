// Copyright (c) 2024, Posidonia Technology d.o.o.
// All rights reserved.
//
// 
// Apache License, Version 2.0
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.


#ifndef PATH_FOLLOWING_CONTROLLER__TEST_PATH_FOLLOWING_CONTROLLER_HPP_
#define PATH_FOLLOWING_CONTROLLER__TEST_PATH_FOLLOWING_CONTROLLER_HPP_

#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "gmock/gmock.h"

#include "path_following_controller/path_following_controller.hpp"
#include "path_following_controller/trajectory.hpp"
#include "path_following_controller/tolerances.hpp"

using namespace testing;

using PathFollowingController = path_following_controller::PathFollowingController;
//msgs
using ControllerStateMsg = control_msgs::msg::JointTrajectoryControllerState;
using JointTrajectoryPoint = trajectory_msgs::msg::JointTrajectoryPoint;
using JointTrajectory = trajectory_msgs::msg::JointTrajectory;

namespace
{
const auto NODE_SUCCESS = controller_interface::CallbackReturn::SUCCESS;
const auto NODE_ERROR = controller_interface::CallbackReturn::ERROR;
const std::string POSITION = "position";
const std::string VELOCITY = "velocity";
const std::string ACCELERATION = "acceleration";
const std::string EFFORT = "effort";

const size_t DOF = 3;

const double INITIAL_POS_JOINT1 = 1.1;
const double INITIAL_POS_JOINT2 = 2.1;
const double INITIAL_POS_JOINT3 = 3.1;
const std::vector<double> INITIAL_POS_JOINTS = {
  INITIAL_POS_JOINT1, INITIAL_POS_JOINT2, INITIAL_POS_JOINT3};
const std::vector<double> INITIAL_VEL_JOINTS = {0.0, 0.0, 0.0};
const std::vector<double> INITIAL_ACC_JOINTS = {0.0, 0.0, 0.0};
const std::vector<double> INITIAL_EFF_JOINTS = {0.0, 0.0, 0.0};

const std::vector<std::vector<double>> THREE_POINTS_POS = {
  {{3.3, 4.4, 5.5}}, {{7.7, 8.8, 9.9}}, {{10.10, 11.11, 12.12}}
};
const std::vector<std::vector<double>> THREE_POINTS_VEL = {
  {{0.01, 0.01, 0.01}}, {{0.05, 0.05, 0.05}}, {{0.06, 0.06, 0.06}}
};

const double WAYPOINT_TOLERANCE = 0.1;

const auto DEFAULT_DELAY_BETWEEN_POINTS = rclcpp::Duration::from_seconds(0.25);
// run update for the total length of one default update_rate (0.01)
const auto SINGLE_CYCLE = rclcpp::Duration::from_seconds(0.01);

}  // namespace

namespace test_path_following_controller
{

class TestablePathFollowingController : public PathFollowingController
{
public:
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override
  {
    auto ret = PathFollowingController::on_configure(previous_state);
    // this class can still be useful without the wait set
    if (joint_command_subscriber_)
    {
      subscriber_wait_set_.add_subscription(joint_command_subscriber_);
    }
    return ret;
  }
  
  /**
   * @brief wait_for_trajectory block until a new JointTrajectory is received.
   * Requires that the executor is not spinned elsewhere between the
   *  message publication and the call to this function
   *
   * @return true if new JointTrajectory msg was received, false if timeout
   */
  bool wait_for_trajectory(
    rclcpp::Executor & executor,
    const std::chrono::milliseconds & timeout = std::chrono::milliseconds{500})
  {
    bool success = subscriber_wait_set_.wait(timeout).kind() == rclcpp::WaitResultKind::Ready;
    if (success)
    {
      executor.spin_some();
    }
    return success;
  }
   
  void set_joint_names(const std::vector<std::string> & joint_names) 
    { params_.joints = joint_names; }

  void set_command_joint_names(const std::vector<std::string> & command_joint_names) 
    { command_joint_names_ = command_joint_names; }

  void set_command_interfaces(const std::vector<std::string> & command_interfaces)
    { params_.command_interfaces = command_interfaces; }

  void set_state_interfaces(const std::vector<std::string> & state_interfaces)
    { params_.state_interfaces = state_interfaces; }
  
  bool has_position_state_interface() const { return has_position_state_interface_; }

  bool has_velocity_state_interface() const { return has_velocity_state_interface_; }

  bool has_acceleration_state_interface() const { return has_acceleration_state_interface_; }

  bool has_position_command_interface() const { return has_position_command_interface_; }

  bool has_velocity_command_interface() const { return has_velocity_command_interface_; }

  bool has_acceleration_command_interface() const { return has_acceleration_command_interface_; }

  bool has_effort_command_interface() const { return has_effort_command_interface_; }

  path_following_controller::SegmentTolerances get_tolerances() const { return default_tolerances_; }
  path_following_controller::Params get_parameters() const { return params_; }

  void trigger_declare_parameters() { param_listener_->declare_params(); }

  JointTrajectoryPoint compute_error(
    JointTrajectoryPoint & state_error,
    JointTrajectoryPoint & state_current,
    JointTrajectoryPoint & state_desired
  ){
    for (size_t i = 0; i < DOF; ++i){
      compute_error_for_joint(state_error, i, state_current, state_desired);
    }
    return state_error;
  }

  JointTrajectoryPoint get_state_feedback() { return state_current_; }
  JointTrajectoryPoint get_state_reference() { return state_desired_; }
  JointTrajectoryPoint get_state_error() { return state_error_; }
  
  bool has_active_traj() const { return has_active_trajectory(); }
  bool validate_traj_msg(JointTrajectory & msg) const { return validate_trajectory_msg(msg); }

  // trivial = only one point in trajectory
  bool has_trivial_traj() const
  {
    return has_active_trajectory() && traj_external_point_ptr_->has_nontrivial_msg() == false;
  }

  bool has_nontrivial_traj()
  {
    return has_active_trajectory() && traj_external_point_ptr_->has_nontrivial_msg();
  }

  bool is_holding() { return *(rt_is_holding_.readFromRT()); }

  rclcpp::WaitSet subscriber_wait_set_;
};

class FixturePFC : public testing::Test
{
public:

  virtual void SetUp() override {
    controller_name_ = "test_path_following_controller";
    joint_names_ = {"joint1", "joint2", "joint3"};
    joint_pos_.resize(joint_names_.size(), 0.0);
    joint_state_pos_.resize(joint_names_.size(), 0.0);
    joint_vel_.resize(joint_names_.size(), 0.0);
    joint_state_vel_.resize(joint_names_.size(), 0.0);
    joint_acc_.resize(joint_names_.size(), 0.0);
    joint_state_acc_.resize(joint_names_.size(), 0.0);
    joint_eff_.resize(joint_names_.size(), 0.0);

    // Default interface values - they will be overwritten in parameterized tests
    command_interface_types_ = {POSITION};
    state_interface_types_ = {POSITION};
 
    node_ = std::make_shared<rclcpp::Node>("trajectory_publisher_");
    trajectory_publisher_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      controller_name_ + "/joint_trajectory", rclcpp::SystemDefaultsQoS());
  }

  virtual void TearDown() override {}

  void InitializePFC(
    rclcpp::Executor & executor,
    const std::vector<rclcpp::Parameter> & parameters = {})
  {
    controller_ = std::make_shared<TestablePathFollowingController>();

    std::vector<rclcpp::Parameter> parameter_overrides;
    parameter_overrides.push_back(rclcpp::Parameter("joints", joint_names_));
    parameter_overrides.push_back(
      rclcpp::Parameter("command_interfaces", command_interface_types_));
    parameter_overrides.push_back(rclcpp::Parameter("state_interfaces", state_interface_types_));
    parameter_overrides.push_back(
      rclcpp::Parameter("allow_nonzero_velocity_at_trajectory_end", true));
    parameter_overrides.insert(parameter_overrides.end(), parameters.begin(), parameters.end());

    auto node_options = rclcpp::NodeOptions();
    node_options.parameter_overrides(parameter_overrides);

    auto ret = controller_->init(controller_name_, "", node_options);

    if (ret != controller_interface::return_type::OK)
    {
      FAIL();
    }

    executor.add_node(controller_->get_node()->get_node_base_interface());

  }

  rclcpp_lifecycle::State ConfigurePFC(){
    return controller_->configure();
  }
      
  rclcpp_lifecycle::State ActivatePFC(
    bool separate_cmd_and_state_values = false,
    const std::vector<double> initial_pos_joints = INITIAL_POS_JOINTS,
    const std::vector<double> initial_vel_joints = INITIAL_VEL_JOINTS,
    const std::vector<double> initial_acc_joints = INITIAL_ACC_JOINTS,
    const std::vector<double> initial_eff_joints = INITIAL_EFF_JOINTS)
  {
    std::vector<hardware_interface::LoanedCommandInterface> cmd_interfaces;
    std::vector<hardware_interface::LoanedStateInterface> state_interfaces;
    pos_cmd_interfaces_.reserve(joint_names_.size());
    vel_cmd_interfaces_.reserve(joint_names_.size());
    acc_cmd_interfaces_.reserve(joint_names_.size());
    eff_cmd_interfaces_.reserve(joint_names_.size());
    pos_state_interfaces_.reserve(joint_names_.size());
    vel_state_interfaces_.reserve(joint_names_.size());
    acc_state_interfaces_.reserve(joint_names_.size());

    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      pos_cmd_interfaces_.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_POSITION, &joint_pos_[i]));
      vel_cmd_interfaces_.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY, &joint_vel_[i]));
      acc_cmd_interfaces_.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_ACCELERATION, &joint_acc_[i]));
      eff_cmd_interfaces_.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_EFFORT, &joint_eff_[i]));

      pos_state_interfaces_.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_POSITION,
        separate_cmd_and_state_values ? &joint_state_pos_[i] : &joint_pos_[i]));
      vel_state_interfaces_.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY,
        separate_cmd_and_state_values ? &joint_state_vel_[i] : &joint_vel_[i]));
      acc_state_interfaces_.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_ACCELERATION,
        separate_cmd_and_state_values ? &joint_state_acc_[i] : &joint_acc_[i]));
      
      // Add to export lists and set initial values
      cmd_interfaces.emplace_back(pos_cmd_interfaces_.back());
      cmd_interfaces.back().set_value(initial_pos_joints[i]);
      cmd_interfaces.emplace_back(vel_cmd_interfaces_.back());
      cmd_interfaces.back().set_value(initial_vel_joints[i]);
      cmd_interfaces.emplace_back(acc_cmd_interfaces_.back());
      cmd_interfaces.back().set_value(initial_acc_joints[i]);
      cmd_interfaces.emplace_back(eff_cmd_interfaces_.back());
      cmd_interfaces.back().set_value(initial_eff_joints[i]);
      if (separate_cmd_and_state_values)
      {
        joint_state_pos_[i] = INITIAL_POS_JOINTS[i];
        joint_state_vel_[i] = INITIAL_VEL_JOINTS[i];
        joint_state_acc_[i] = INITIAL_ACC_JOINTS[i];
      }
      state_interfaces.emplace_back(pos_state_interfaces_.back());
      state_interfaces.emplace_back(vel_state_interfaces_.back());
      state_interfaces.emplace_back(acc_state_interfaces_.back());
    }
    controller_->assign_interfaces(std::move(cmd_interfaces), std::move(state_interfaces));
    
    auto temp = controller_->get_node()->activate();
    return temp;
  }

  void InitializeConfigureActivatePFC(
    rclcpp::Executor & ex,
    const std::vector<rclcpp::Parameter> & parameters = {},
    bool do_cmd_and_state_values_differ = false,
    const std::vector<double> initial_pos_joints = INITIAL_POS_JOINTS,
    const std::vector<double> initial_vel_joints = INITIAL_VEL_JOINTS,
    const std::vector<double> initial_acc_joints = INITIAL_ACC_JOINTS,
    const std::vector<double> initial_eff_joints = INITIAL_EFF_JOINTS)
  {
    InitializePFC(ex, parameters);

    controller_->trigger_declare_parameters();

    ConfigurePFC();
    ActivatePFC(
      do_cmd_and_state_values_differ,
      initial_pos_joints, 
      initial_vel_joints, 
      initial_acc_joints, 
      initial_eff_joints);
  }

  rclcpp_lifecycle::State DeactivatePFC(){
    return controller_->get_node()->deactivate();
  }
  rclcpp_lifecycle::State CleanUpPFC(){
    return controller_->get_node()->cleanup();
  }

  /**
   * @brief a wrapper for update() method of JTC, running synchronously with the clock
   * @param wait_time - the time span for updating the controller
   * @param update_rate - the rate at which the controller is updated
   *
   * @note use the faster updateControllerAsync() if no subscriptions etc.
   * have to be used from the waitSet/executor
   */
  void UpdatePFC(
    rclcpp::Duration wait_time = rclcpp::Duration::from_seconds(0.2),
    const rclcpp::Duration update_rate = rclcpp::Duration::from_seconds(0.01))
  {
    auto clock = rclcpp::Clock(RCL_STEADY_TIME);
    const auto start_time = clock.now();
    const auto end_time = start_time + wait_time;
    auto previous_time = start_time;

    while (clock.now() <= end_time)
    {
      auto now = clock.now();
      controller_->update(now, now - previous_time);
      previous_time = now;
      std::this_thread::sleep_for(update_rate.to_chrono<std::chrono::milliseconds>());
    }
  }

  /**
   * @brief a wrapper for update() method of JTC, running asynchronously from the clock
   * @return the time at which the update finished
   * @param wait_time - the time span for updating the controller
   * @param start_time - the time at which the update should start
   * @param update_rate - the rate at which the controller is updated
   *
   * @note this is faster than updateController() and can be used if no subscriptions etc.
   * have to be used from the waitSet/executor
   */
  rclcpp::Time UpdateAsyncPFC(
    rclcpp::Duration wait_time = rclcpp::Duration::from_seconds(0.1),
    rclcpp::Time start_time = rclcpp::Time(0, 0, RCL_STEADY_TIME),
    const rclcpp::Duration update_rate = rclcpp::Duration::from_seconds(0.01))
  {
    if (start_time == rclcpp::Time(0, 0, RCL_STEADY_TIME))
    {
      start_time = rclcpp::Clock(RCL_STEADY_TIME).now();
    }
    const auto end_time = start_time + wait_time;
    auto time_counter = start_time;
    while (time_counter <= end_time)
    {
      controller_->update(time_counter, update_rate);
      time_counter += update_rate;
    }
    return end_time;
  }

  /**
   *  @brief Publish trajectory msgs with multiple points
   *  @param delay_btwn_points delay between each points
   *  @param start_time timestamp for msg header
   *  @param points_positions vector of trajectory-positions. One point per controlled joint
   *  @param points_velocities vector of trajectory-velocities. One point per controlled joint
   *  @param joint_names names of joints, if empty, will use joint_names_ up to the number of provided
   *  points
   */
  void PublishToJointTrajectory(
    const builtin_interfaces::msg::Duration & delay_btwn_points,
    const std::vector<std::vector<double>> & points_positions,
    const std::vector<std::vector<double>> & points_velocities = {},
    const std::vector<std::string> & joint_names = {},
    rclcpp::Time start_time = rclcpp::Time())
  {
    int wait_count = 0;
    const auto topic = trajectory_publisher_->get_topic_name();
    while (node_->count_subscribers(topic) == 0)
    {
      if (wait_count >= 5)
      {
        auto error_msg = std::string("publishing to ") + topic + " but no node subscribes to it";
        throw std::runtime_error(error_msg);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      ++wait_count;
    }

    trajectory_msgs::msg::JointTrajectory traj_msg;
    if (joint_names.empty())
    {
      traj_msg.joint_names = {
        joint_names_.begin(), joint_names_.begin() + points_positions[0].size()};
    }
    else
    {
      traj_msg.joint_names = joint_names;
    }
    traj_msg.header.stamp = start_time;
    traj_msg.points.resize(points_positions.size());

    builtin_interfaces::msg::Duration duration_msg;
    duration_msg.sec = delay_btwn_points.sec;
    duration_msg.nanosec = delay_btwn_points.nanosec;
    rclcpp::Duration duration(duration_msg);
    rclcpp::Duration duration_total(duration_msg);

    for (size_t index = 0; index < points_positions.size(); ++index)
    {
      traj_msg.points[index].time_from_start = duration_total;

      traj_msg.points[index].positions.resize(points_positions[index].size());
      for (size_t j = 0; j < points_positions[index].size(); ++j)
      {
        traj_msg.points[index].positions[j] = points_positions[index][j];
      }
      duration_total = duration_total + duration;
    }

    for (size_t index = 0; index < points_velocities.size(); ++index)
    {
      traj_msg.points[index].velocities.resize(points_velocities[index].size());
      for (size_t j = 0; j < points_velocities[index].size(); ++j)
      {
        traj_msg.points[index].velocities[j] = points_velocities[index][j];
      }
    }

    trajectory_publisher_->publish(traj_msg);
  }

  void setJointAngleWraparound(bool value)
  {
    controller_->trigger_declare_parameters();
    for (size_t i = 0; i < DOF; ++i)
    {
      const std::string prefix = "gains." + joint_names_[i];
      const rclcpp::Parameter angle_wraparound(
        prefix + ".angle_wraparound", value);
      controller_->get_node()->set_parameters({angle_wraparound});
    }
  }

  // returns a trajectory with one point where the joint names
  // are in a different order than state_interfaces
  JointTrajectory createJumbledTrajectoryMsg(
    std::vector<double> position_in,
    std::vector<double> velocity_in)
  {
    JointTrajectory msg_out;
    std::vector<size_t> jumble_map = {1, 2, 0};

    std::vector<std::string> jumbled_names;
    jumbled_names.resize(DOF);

    for (size_t i = 0; i < DOF; i++)
      jumbled_names[i] = joint_names_[ jumble_map[i] ];

    msg_out.points.resize(1);
    msg_out.joint_names = jumbled_names;
    msg_out.points[0].positions.resize(DOF);
    for (size_t i = 0; i < DOF; i++)
      msg_out.points[0].positions[i] = position_in[ jumble_map[i] ];

    msg_out.points[0].velocities.resize(DOF);
    for (size_t i = 0; i < DOF; i++)
      msg_out.points[0].velocities[i] = velocity_in[ jumble_map[i] ];

    msg_out.points[0].accelerations.resize(DOF);
    
    return msg_out;
  }

  // returns a trajectory with one point with DOF-1
  // joint states defined
  JointTrajectory createPartialJointMsg(
    std::vector<double> position_in,
    std::vector<double> velocity_in)
  {
    JointTrajectory msg_out;

    const std::vector<std::string> partial_names = { joint_names_[0], joint_names_[1] };

    msg_out.points.resize(1);
    msg_out.joint_names = partial_names;
    msg_out.points[0].positions.resize(DOF-1);
    for (size_t i = 0; i < DOF-1; i++)
      msg_out.points[0].positions[i] = position_in[i];

    msg_out.points[0].velocities.resize(DOF-1);
    for (size_t i = 0; i < DOF-1; i++)
      msg_out.points[0].velocities[i] = velocity_in[i];

    msg_out.points[0].accelerations.resize(DOF-1);
    
    return msg_out;
  }

  void getStateMsg(); //returns state msg


  // functions with ASSERTIONS mixed in. Move to tests.
  // separate acting from assertion -> Arrange, Act, Assert
  void expectCommandPoint(){} // checks if the expected one 
  void waitAndCompareState(){}
  void expectHoldingPointDeactivated(){}


  std::shared_ptr<TestablePathFollowingController> controller_;

  std::string controller_name_;

  std::vector<std::string> joint_names_;
  std::vector<std::string> command_interface_types_;
  std::vector<std::string> state_interface_types_;

  std::vector<double> joint_pos_;
  std::vector<double> joint_vel_;
  std::vector<double> joint_acc_;
  std::vector<double> joint_eff_;
  std::vector<double> joint_state_pos_;
  std::vector<double> joint_state_vel_;
  std::vector<double> joint_state_acc_;
  std::vector<hardware_interface::CommandInterface> pos_cmd_interfaces_;
  std::vector<hardware_interface::CommandInterface> vel_cmd_interfaces_;
  std::vector<hardware_interface::CommandInterface> acc_cmd_interfaces_;
  std::vector<hardware_interface::CommandInterface> eff_cmd_interfaces_;
  std::vector<hardware_interface::StateInterface> pos_state_interfaces_;
  std::vector<hardware_interface::StateInterface> vel_state_interfaces_;
  std::vector<hardware_interface::StateInterface> acc_state_interfaces_;

  rclcpp::Publisher<JointTrajectory>::SharedPtr trajectory_publisher_;
  rclcpp::Node::SharedPtr node_;

  // review necessity
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr
    state_subscriber_;
  mutable std::mutex state_mutex_;
  std::shared_ptr<control_msgs::msg::JointTrajectoryControllerState> state_msg_;
  
};


class ParametrizedFixturePFC
: public FixturePFC,
  public ::testing::WithParamInterface<
    std::tuple<std::vector<std::string>, std::vector<std::string>>>
{
public:
  virtual void SetUp()
  {
    FixturePFC::SetUp();
    command_interface_types_ = std::get<0>(GetParam());
    state_interface_types_ = std::get<1>(GetParam());
  }
};

} //test_path_following_controller

#endif  // PATH_FOLLOWING_CONTROLLER__TEST_PATH_FOLLOWING_CONTROLLER_HPP_
