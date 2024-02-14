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

using namespace test_path_following_controller;
using namespace testing;

namespace
{
    using namespace path_following_controller;

    // returns true if tolerances are the same
    bool compareWaypointTolerances(size_t dof,
                std::vector<StateTolerances> tol1,
                std::vector<StateTolerances> tol2)
    {
        for (size_t i = 0; i < dof; i++)
        {
            if(
                tol1[i].position != tol2[i].position
            ) return false;
            if(
                tol1[i].velocity != tol2[i].velocity
            ) return false;
            if(
                tol1[i].acceleration != tol2[i].acceleration
            ) return false;
        }
        return true;
    }
}

TEST_F(FixturePFC, tolerances_read_from_parameters)
{
    rclcpp::executors::MultiThreadedExecutor ex;
    InitializeConfigureActivatePFC(ex);

    UpdateAsyncPFC();

    auto current_tolerances = controller_->get_tolerances();

    for (size_t i = 0; i < DOF; i++)
    {      
        ASSERT_EQ(current_tolerances.waypoint.at(i).position, 0.0);
    }
}


TEST_F(FixturePFC, tolerances_update_dynamically)
{
    rclcpp::executors::MultiThreadedExecutor ex;
    InitializeConfigureActivatePFC(ex);
    UpdateAsyncPFC();

    auto starting_tolerances = controller_->get_tolerances();

    std::vector<rclcpp::Parameter> new_parameters{
        rclcpp::Parameter("constraints.joint1.waypoint", 1.0),
        rclcpp::Parameter("constraints.joint2.waypoint", 2.0),
        rclcpp::Parameter("constraints.joint3.waypoint", 3.0)};
    controller_->get_node()->set_parameters(new_parameters);

    UpdateAsyncPFC();

    auto updated_tolerances = controller_->get_tolerances();

    ASSERT_FALSE(compareWaypointTolerances(DOF,
                                        starting_tolerances.waypoint,
                                        updated_tolerances.waypoint));   
}

TEST_F(FixturePFC, state_reference_propagates_to_feedback_properly)
{
    rclcpp::executors::MultiThreadedExecutor ex;
    InitializeConfigureActivatePFC(ex);
    PublishToJointTrajectory(DEFAULT_DELAY_BETWEEN_POINTS,
                           THREE_POINTS_POS, 
                           THREE_POINTS_VEL);
    controller_->wait_for_trajectory(ex);

    UpdatePFC( (SINGLE_CYCLE*2) );
    auto state_reference = controller_->get_state_reference();
    auto state_feedback = controller_->get_state_feedback();
    
    // after 2 cycles the reference should be at 2nd point...
    EXPECT_THAT(state_reference.positions, 
        ElementsAreArray(THREE_POINTS_POS[1]));
    // ... and should have propagated the first point to feedback
    ASSERT_THAT(state_feedback.positions,
        ElementsAreArray(THREE_POINTS_POS[0]));
}

TEST_F(FixturePFC, error_calculation_works)
{
    rclcpp::executors::MultiThreadedExecutor ex;
    InitializeConfigureActivatePFC(ex);
    PublishToJointTrajectory(DEFAULT_DELAY_BETWEEN_POINTS,
                           THREE_POINTS_POS, 
                           THREE_POINTS_VEL);
    controller_->wait_for_trajectory(ex);

    UpdatePFC( SINGLE_CYCLE );
    JointTrajectoryPoint state_reference = controller_->get_state_reference();
    
    // initializing like this so I don't have to resize later.
    // values will be overwritten anyways.
    JointTrajectoryPoint dummy_state = state_reference;
    JointTrajectoryPoint state_error = state_reference;

    // create a 1.0 disparity in position...
    for (size_t i = 0; i < DOF; ++i){
        dummy_state.positions[i] = state_reference.positions[i] - 1.0;
    }
    
    // ...and check if the function computes the same disparity
    controller_->compute_error(state_error, dummy_state, state_reference);
    ASSERT_THAT(state_error.positions, Each(Eq(1.0)));
}

TEST_F(FixturePFC, position_error_wraps_around_when_configured)
{
    rclcpp::executors::MultiThreadedExecutor ex;
    InitializePFC(ex);
    // angle_wraparound doesn't update dynamically,
    // it must be set before configuring
    setJointAngleWraparound(true);

    ConfigurePFC();
    ActivatePFC();
    PublishToJointTrajectory(DEFAULT_DELAY_BETWEEN_POINTS,
                           THREE_POINTS_POS, 
                           THREE_POINTS_VEL);
    controller_->wait_for_trajectory(ex);
    UpdatePFC( SINGLE_CYCLE );

    JointTrajectoryPoint state_reference = controller_->get_state_reference();
    // initializing like this so I don't have to resize.
    // values will be overwritten anyways.
    JointTrajectoryPoint dummy_state = state_reference;
    JointTrajectoryPoint state_error = state_reference;

    // create a disparity in position larger than [-pi, pi]
    for (size_t i = 0; i < DOF; ++i){
        dummy_state.positions[i] = state_reference.positions[i] - 20.0;
    }

    // ...and check if it wraps around to [-pi, pi]
    controller_->compute_error(state_error, dummy_state, state_reference);

    ASSERT_THAT(state_error.positions, Each(Ge(-M_PI)));
    ASSERT_THAT(state_error.positions, Each(Le(M_PI)));
}

TEST_F(FixturePFC, traj_msg_joint_order_different_from_interfaces)
{
    rclcpp::executors::MultiThreadedExecutor ex;
    InitializeConfigureActivatePFC(ex);

    const JointTrajectory jumbled_msg = createJumbledTrajectoryMsg(THREE_POINTS_POS[0]);                                              
    PublishToJointTrajectory(DEFAULT_DELAY_BETWEEN_POINTS,
                           { jumbled_msg.points[0].positions  }, 
                           {},
                           jumbled_msg.joint_names);
    controller_->wait_for_trajectory(ex);
    UpdatePFC( SINGLE_CYCLE );

    // ensure that the message got un-jumbled when read
    JointTrajectoryPoint state_reference = controller_->get_state_reference();
    ASSERT_THAT(state_reference.positions, 
        ElementsAreArray(THREE_POINTS_POS[0]));
}

TEST_F(FixturePFC, partial_joints_traj_rejected_when_partial_joints_goal_not_allowed)
{
    rclcpp::executors::MultiThreadedExecutor ex;
    InitializeConfigureActivatePFC(ex);
    // allow_partial_joints_goal = false; by default

    // assemble a one-point partial goal msg
    JointTrajectory msg = createPartialJointMsg(THREE_POINTS_POS[0]);

    const bool is_accepted = controller_->validate_traj_msg(msg);
    EXPECT_FALSE(is_accepted);
}

TEST_F(FixturePFC, partial_joints_traj_accepted_when_partial_joints_goal_allowed)
{
    rclcpp::executors::MultiThreadedExecutor ex;
    rclcpp::Parameter partial_joints_enabled("allow_partial_joints_goal", true);
    InitializePFC(ex, {partial_joints_enabled});
    ConfigurePFC();
    ActivatePFC();

    // assemble a one-point partial goal msg
    JointTrajectory msg = createPartialJointMsg(THREE_POINTS_POS[0]);

    const bool is_accepted = controller_->validate_traj_msg(msg);
    EXPECT_TRUE(is_accepted);
}

TEST_F(FixturePFC, rejects_traj_with_empty_joint_names)
{
    rclcpp::executors::MultiThreadedExecutor ex;
    rclcpp::Parameter partial_joints_enabled("allow_partial_joints_goal", true);
    InitializePFC(ex, {partial_joints_enabled});
    ConfigurePFC();
    ActivatePFC();

    // create some msg and empty out the names
    JointTrajectory msg = createPartialJointMsg(THREE_POINTS_POS[0],
                                                THREE_POINTS_VEL[0]);
    msg.joint_names = std::vector<std::string>();

    const bool is_accepted = controller_->validate_traj_msg(msg);
    EXPECT_FALSE(is_accepted);
}

TEST_F(FixturePFC, reference_point_repeated_next_cycle_on_tolerance_fail)
{
    rclcpp::executors::MultiThreadedExecutor ex;
    // constraint set for joint1
    rclcpp::Parameter joint1_tolerance_set("constraints.joint1.waypoint", 0.2);
    InitializePFC(ex, {joint1_tolerance_set});
    ConfigurePFC();
    ActivatePFC();

    PublishToJointTrajectory(DEFAULT_DELAY_BETWEEN_POINTS,
                           THREE_POINTS_POS, 
                           THREE_POINTS_VEL);
    controller_->wait_for_trajectory(ex);

    // after one cycle, reference is at first point, but the tolerance is violated
    // after two cycles, expect that reference repeats
    UpdatePFC( SINGLE_CYCLE*2 );
    JointTrajectoryPoint state_reference = controller_->get_state_reference();
    ASSERT_THAT(state_reference.positions,
        ElementsAreArray(THREE_POINTS_POS[0]));

    // after next cycle, the first point is reached and reference moves on
    UpdatePFC( SINGLE_CYCLE );
    state_reference = controller_->get_state_reference();
    ASSERT_THAT(state_reference.positions,
        ElementsAreArray(THREE_POINTS_POS[1]));
}



int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
