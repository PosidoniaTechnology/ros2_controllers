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

using namespace test_path_following_controller;
using namespace testing;

namespace
{
    using namespace path_following_controller;

    // returns true if tolerances are the same
    bool compareStateTolerances(size_t dof,
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
        ASSERT_EQ(current_tolerances.state_tolerance.at(i).position, 0.0);
    }
}


TEST_F(FixturePFC, tolerances_update_dynamically)
{
    rclcpp::executors::MultiThreadedExecutor ex;
    InitializeConfigureActivatePFC(ex);
    UpdateAsyncPFC();

    auto starting_tolerances = controller_->get_tolerances();

    std::vector<rclcpp::Parameter> new_parameters{
        rclcpp::Parameter("constraints.joint1.trajectory", 1.0),
        rclcpp::Parameter("constraints.joint2.trajectory", 2.0),
        rclcpp::Parameter("constraints.joint3.trajectory", 3.0)};
    controller_->get_node()->set_parameters(new_parameters);

    UpdateAsyncPFC();

    auto updated_tolerances = controller_->get_tolerances();

    ASSERT_FALSE(compareStateTolerances(DOF,
        starting_tolerances.state_tolerance,
        updated_tolerances.state_tolerance));   
}

TEST_F(FixturePFC, state_reference_propagates_to_feedback_properly)
{
    rclcpp::executors::MultiThreadedExecutor ex;
    InitializeConfigureActivatePFC(ex);
    PublishToJointTrajectory(DEFAULT_DELAY_BETWEEN_POINTS,
                           THREE_POINTS_POS, 
                           THREE_POINTS_VEL);
    controller_->wait_for_trajectory(ex);

    // run 2 cycles, the controller should've processed 2/3 points
    UpdatePFC( (SINGLE_CYCLE*2) );
    auto state_reference = controller_->get_state_reference();
    auto state_feedback = controller_->get_state_feedback();
    
    // after 2 cycles the reference should be at 2nd point...
    EXPECT_THAT(state_reference.positions, 
        ElementsAreArray(THREE_POINTS_POS[1]));
    // ... and propagate the point to feedback
    ASSERT_THAT(state_feedback.positions,
        ElementsAreArray(state_feedback.positions));
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



int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
