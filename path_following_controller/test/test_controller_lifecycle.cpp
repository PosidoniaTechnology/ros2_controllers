// Copyright (c) 2024, Posidonia Technology d.o.o.
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include "test_path_following_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"

using lifecycle_msgs::msg::State;

using namespace test_path_following_controller;

TEST_P(ParametrizedFixturePFC, can_configure)
{
  rclcpp::executors::MultiThreadedExecutor ex;
  InitializePFC(ex);
  auto state = ConfigurePFC().id();
  ASSERT_EQ(state, State::PRIMARY_STATE_INACTIVE);
}

TEST_P(ParametrizedFixturePFC, can_activate)
{
  rclcpp::executors::MultiThreadedExecutor ex;
  InitializePFC(ex);
  ConfigurePFC();
  auto state = ActivatePFC().id();
  ASSERT_EQ(state, State::PRIMARY_STATE_ACTIVE);
}

TEST_P(ParametrizedFixturePFC, can_update_and_deactivate)
{
  rclcpp::executors::MultiThreadedExecutor ex;
  InitializeConfigureActivatePFC(ex);
  UpdatePFC(rclcpp::Duration::from_seconds(0.5));

  auto state = DeactivatePFC().id();
  ASSERT_EQ(state, State::PRIMARY_STATE_INACTIVE);
}

TEST_P(ParametrizedFixturePFC, can_clean_up)
{
  rclcpp::executors::MultiThreadedExecutor ex;
  InitializeConfigureActivatePFC(ex);
  UpdatePFC(rclcpp::Duration::from_seconds(0.5));
  DeactivatePFC();

  auto state = CleanUpPFC().id();
  ASSERT_EQ(state, State::PRIMARY_STATE_UNCONFIGURED);
}


TEST_P(ParametrizedFixturePFC, configured_without_activation_ignores_commands)
{
  rclcpp::executors::MultiThreadedExecutor ex;

  InitializePFC(ex);  
  ConfigurePFC();

  PublishToJointTrajectory(DEFAULT_DELAY_BETWEEN_POINTS,
                           THREE_POINTS_POS, 
                           THREE_POINTS_VEL);

  controller_->wait_for_trajectory(ex);

  UpdatePFC(rclcpp::Duration::from_seconds(0.5));

  // hw position == 0 because controller is not activated
  EXPECT_EQ(0.0, joint_pos_[0]);
  EXPECT_EQ(0.0, joint_pos_[1]);
  EXPECT_EQ(0.0, joint_pos_[2]);
}

TEST_P(ParametrizedFixturePFC, can_clean_up_after_configure)
{
  rclcpp::executors::MultiThreadedExecutor ex;
  InitializePFC(ex),
  ConfigurePFC();

  auto state = CleanUpPFC().id();
  ASSERT_EQ(state, State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_P(ParametrizedFixturePFC, controller_holds_on_activation)
{
  rclcpp::executors::MultiThreadedExecutor ex;
  InitializeConfigureActivatePFC(ex);
  UpdatePFC(rclcpp::Duration::from_seconds(0.5));
  
  EXPECT_EQ(INITIAL_POS_JOINT1, joint_pos_[0]);
  EXPECT_EQ(INITIAL_POS_JOINT2, joint_pos_[1]);
  EXPECT_EQ(INITIAL_POS_JOINT3, joint_pos_[2]);
  EXPECT_TRUE(controller_->is_holding());
}

  /*
TEST_P(ParametrizedFixturePFC, can_reactivate_from_parameters)
{
  rclcpp::executors::MultiThreadedExecutor ex;
  InitializeConfigureActivatePFC(ex);
  ASSERT_EQ(1,1);

  PublishToJointTrajectory(DEFAULT_DELAY_BETWEEN_POINTS,
                            THREE_POINTS_POS, 
                            THREE_POINTS_VEL);

  // think about how to run one cycle, process one point,
  // deactivate and then reactivate.

  EXPECT_TRUE(controller_->is_holding());
}
  */



using namespace std;
INSTANTIATE_TEST_SUITE_P(
  PositionControl, ParametrizedFixturePFC,
  ::testing::Values(
    make_tuple(vector<string>({POSITION}), vector<string>({POSITION}))//,
    //make_tuple(vector<string>({POSITION}), vector<string>({POSITION, VELOCITY})),
    //make_tuple(vector<string>({POSITION}), vector<string>({POSITION, VELOCITY, ACCELERATION}))
    )
  );

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
