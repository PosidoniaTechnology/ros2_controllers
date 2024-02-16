// Copyright (c) 2024, Posidonia Technology d.o.o.
// All rights reserved.
//
// 
// Apache License, Version 2.0
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
using namespace testing;

TEST_P(ParametrizedFixturePFC, can_configure)
{

  InitializePFC();
  auto state = ConfigurePFC().id();
  ASSERT_EQ(state, State::PRIMARY_STATE_INACTIVE);
}

TEST_P(ParametrizedFixturePFC, can_activate)
{

  InitializePFC();
  ConfigurePFC();
  auto state = ActivatePFC().id();
  ASSERT_EQ(state, State::PRIMARY_STATE_ACTIVE);
}

TEST_P(ParametrizedFixturePFC, can_update_and_deactivate)
{

  InitializeConfigureActivatePFC();
  UpdatePFC(rclcpp::Duration::from_seconds(0.5));

  auto state = DeactivatePFC().id();
  ASSERT_EQ(state, State::PRIMARY_STATE_INACTIVE);
}

TEST_P(ParametrizedFixturePFC, can_reactivate)
{

  InitializeConfigureActivatePFC();
  UpdatePFC(rclcpp::Duration::from_seconds(0.5));
  DeactivatePFC();

  auto state = ActivatePFC().id();
  ASSERT_EQ(state, State::PRIMARY_STATE_ACTIVE);
}

TEST_P(ParametrizedFixturePFC, can_clean_up)
{

  InitializeConfigureActivatePFC();
  UpdatePFC(rclcpp::Duration::from_seconds(0.5));
  DeactivatePFC();

  auto state = CleanUpPFC().id();
  ASSERT_EQ(state, State::PRIMARY_STATE_UNCONFIGURED);
}


TEST_P(ParametrizedFixturePFC, configured_without_activation_ignores_commands)
{


  InitializePFC();  
  ConfigurePFC();

  PublishToJointTrajectory(DEFAULT_DELAY_BETWEEN_POINTS,
                           THREE_POINTS_POS, 
                           THREE_POINTS_VEL);

  controller_->wait_for_trajectory(exe_);

  UpdatePFC(rclcpp::Duration::from_seconds(0.5));

  // hw position == 0 because controller is not activated
  EXPECT_THAT(joint_pos_, Each(Eq(0.0)));
}

TEST_P(ParametrizedFixturePFC, can_clean_up_after_configure)
{

  InitializePFC(),
  ConfigurePFC();

  auto state = CleanUpPFC().id();
  ASSERT_EQ(state, State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_P(ParametrizedFixturePFC, controller_holds_on_activation)
{

  InitializeConfigureActivatePFC();
  UpdatePFC(rclcpp::Duration::from_seconds(0.5));
  
  EXPECT_TRUE(controller_->is_holding());
  EXPECT_THAT(joint_pos_, ElementsAreArray(INITIAL_POS_JOINTS));
}

TEST_P(ParametrizedFixturePFC, holds_current_position_on_deactivation)
{

  InitializeConfigureActivatePFC();

  PublishToJointTrajectory(DEFAULT_DELAY_BETWEEN_POINTS,
                            THREE_POINTS_POS, 
                            THREE_POINTS_VEL);
  controller_->wait_for_trajectory(exe_);

  // 2 cycles for current_state to arrive at first point
  UpdatePFC( SINGLE_CYCLE*2 );

  auto state_on_deactivate = controller_->get_state_feedback();
  DeactivatePFC();

  // try to process more points
  UpdatePFC( SINGLE_CYCLE*2 );

  // and see if we're holding without an active trajectory
  EXPECT_FALSE(controller_->has_active_traj());
  EXPECT_TRUE(controller_->is_holding());
  EXPECT_THAT(state_on_deactivate.positions, 
    ElementsAreArray(THREE_POINTS_POS[0]));
}

TEST_P(ParametrizedFixturePFC, can_reactivate_holding_on_point_before_deactivation)
{

  InitializeConfigureActivatePFC();

  PublishToJointTrajectory(DEFAULT_DELAY_BETWEEN_POINTS,
                            THREE_POINTS_POS, 
                            THREE_POINTS_VEL);
  controller_->wait_for_trajectory(exe_);

  // 2 cycles for current_state to arrive at first point
  UpdatePFC( SINGLE_CYCLE*2 );

  auto state_on_deactivate = controller_->get_state_feedback();
  DeactivatePFC();

  // try to process more points...
  UpdatePFC( SINGLE_CYCLE*2 );
  ActivatePFC(false, state_on_deactivate.positions);

  // ...and see if we're holding at point given on deactivation
  EXPECT_FALSE(controller_->has_trivial_traj());
  EXPECT_TRUE(controller_->is_holding());
  EXPECT_THAT(state_on_deactivate.positions, 
    ElementsAreArray(THREE_POINTS_POS[0]));
}

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