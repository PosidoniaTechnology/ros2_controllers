// Copyright (c) 2024, Posidonia Technology d.o.o.
// All rights reserved.
//
// Proprietary License
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

using PathFollowingController = path_following_controller::PathFollowingController;
//msgs
using ControllerStateMsg = control_msgs::msg::JointTrajectoryControllerState;
using JointTrajectoryPoint = trajectory_msgs::msg::JointTrajectoryPoint;
using JointTrajectory = trajectory_msgs::msg::JointTrajectory;

namespace
{
constexpr auto NODE_SUCCESS = controller_interface::CallbackReturn::SUCCESS;
constexpr auto NODE_ERROR = controller_interface::CallbackReturn::ERROR;
const std::string POSITION = "position";
const std::string VELOCITY = "velocity";
const std::string ACCELERATION = "acceleration";
const std::string EFFORT = "effort";
const double INITIAL_POS_JOINT1 = 1.1;
const double INITIAL_POS_JOINT2 = 2.1;
const double INITIAL_POS_JOINT3 = 3.1;
const std::vector<double> INITIAL_POS_JOINTS = {
  INITIAL_POS_JOINT1, INITIAL_POS_JOINT2, INITIAL_POS_JOINT3};
const std::vector<double> INITIAL_VEL_JOINTS = {0.0, 0.0, 0.0};
const std::vector<double> INITIAL_ACC_JOINTS = {0.0, 0.0, 0.0};
const std::vector<double> INITIAL_EFF_JOINTS = {0.0, 0.0, 0.0};

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

  path_following_controller::SegmentTolerances get_tolerances() const
    { return default_tolerances_; }

  JointTrajectoryPoint get_state_feedback() { return state_current_; }
  JointTrajectoryPoint get_state_reference() { return state_desired_; }
  JointTrajectoryPoint get_state_error() { return state_error_; }
  
  bool has_active_traj() const { return has_active_trajectory(); }

  bool has_trivial_traj() const
  {
    return has_active_trajectory() && traj_external_point_ptr_->has_nontrivial_msg() == false;
  }

  bool has_nontrivial_traj()
  {
    return has_active_trajectory() && traj_external_point_ptr_->has_nontrivial_msg();
  }

  rclcpp::WaitSet subscriber_wait_set_;
};

class FixturePFC : public testing::Test
{
public:
  //wrapped:
  // on_configure + assign wait set to subscriber
  // exposes private members and functions.
  virtual void SetUp() override {
    std::cout << "setting up fixture" << std::endl;

    controller_name_ = "test_path_following_controller";
    joint_names_ = {"joint1", "joint2", "joint3"};
    joint_pos_.resize(joint_names_.size(), 0.0);
    joint_state_pos_.resize(joint_names_.size(), 0.0);
    joint_vel_.resize(joint_names_.size(), 0.0);
    joint_state_vel_.resize(joint_names_.size(), 0.0);
    joint_acc_.resize(joint_names_.size(), 0.0);
    joint_state_acc_.resize(joint_names_.size(), 0.0);
    joint_eff_.resize(joint_names_.size(), 0.0);

    // Default interface values - they will be overwritten by parameterized tests
    command_interface_types_ = {"position"};
    state_interface_types_ = {"position"};
  }

  virtual void TearDown() override {
    std::cout << "tearing down fixture" << std::endl;
  }

  virtual void LoadPFCParameters(){};
  virtual void InitializePFC(){};
  virtual void ConfigurePFC(){};
  virtual void ActivatePFC(){};
  virtual void SetUpAndActivatePFC(){};

  void getState(); //returns state msg

  // functions with ASSERTIONS mixed in. Move to tests.
  // separate acting from assertion -> Arrange, Act, Assert
  void expectCommandPoint(){} // checks if the expected one 
  void waitAndCompareState(){}
  void expectHoldingPointDeactivated(){}
  void compareJoints(){} // compare names of itnerfaces and joints

  // examine necessity
  void TestStatePublishRateTarget(); //what


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

  // review necessity

  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr
    state_subscriber_;
  mutable std::mutex state_mutex_;
  std::shared_ptr<control_msgs::msg::JointTrajectoryControllerState> state_msg_;
  
};

class UpdateLoopFixturePFC : public FixturePFC
{
public:
  virtual void SetUp(){ FixturePFC::SetUp(); }
  virtual void TearDown(){ FixturePFC::TearDown(); }

  void updateController();
  void updateControllerAsync();
};

class GoalFromTopicFixturePFC : public UpdateLoopFixturePFC
{
public:
  virtual void SetUp(){ 
    UpdateLoopFixturePFC::SetUp(); 
    node_ = std::make_shared<rclcpp::Node>("trajectory_publisher_");
    trajectory_publisher_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      controller_name_ + "/joint_trajectory", rclcpp::SystemDefaultsQoS());
  }
  virtual void TearDown(){ FixturePFC::TearDown(); }
  
        
  bool waitForTrajectoryFromTopic();
  void subscribeToControllerState();
  void publishToControllerState();

  rclcpp::Publisher<JointTrajectory>::SharedPtr trajectory_publisher_;
  rclcpp::Node::SharedPtr node_;
};

class GoalFromActionFixturePFC : public UpdateLoopFixturePFC
{

};

} //test_path_following_controller

#endif  // PATH_FOLLOWING_CONTROLLER__TEST_PATH_FOLLOWING_CONTROLLER_HPP_
