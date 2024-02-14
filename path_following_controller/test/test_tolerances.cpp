// Copyright (c) 2024, Posidonia Technology d.o.o.
// All rights reserved.
//
// 
// Apache License, Version 2.0
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.




#include <gmock/gmock.h>

#include "path_following_controller/path_following_controller.hpp"
#include "path_following_controller/tolerances.hpp"

using namespace path_following_controller;
using namespace std;

using JointTrajectoryPoint = trajectory_msgs::msg::JointTrajectoryPoint;
using JointTrajectory = trajectory_msgs::msg::JointTrajectory;

using TrajectoryPointIter = std::vector<JointTrajectoryPoint>::iterator;
using TrajectoryPointConstIter = std::vector<JointTrajectoryPoint>::const_iterator;  

namespace
{
    const double WAYPOINT_TOLERANCE = 0.1;
    // testing only position tolerance
    const StateTolerances DEFAULT_STATE_TOLREANCE = 
        {WAYPOINT_TOLERANCE, 0.0, 0.0};

    const double TOLERANCE_VIOLATED_VALUE = 0.2;
    const double TOLERANCE_RESPECTED_VALUE = 0.05;

    size_t DOF = 2;
    const std::vector<std::string> JOINT_NAMES = {"joint1", "joint2"};
}

class TolerancesFixture : public testing::Test
{
    public:
        TolerancesFixture(){
            tolerances_ = SegmentTolerances(DOF); // 2 joint robot
            params_ = Params();
            error_state_ = JointTrajectoryPoint();
        }
        ~TolerancesFixture(){}

        void SetUp(){
            error_state_.positions.resize(DOF);

            // populate params with default position tolerance
            params_.joints.resize(DOF);   
            for(size_t i=0; i<DOF; i++)
            {
                std::string joint_name = JOINT_NAMES[i];
                params_.joints[i] = joint_name;
                params_.constraints.joints_map[joint_name] = {
                    WAYPOINT_TOLERANCE,
                    0.0
                };
            }
        }

        void TearDown(){}

        // returns TRUE if all elements are the same
        bool compareWaypointTolerances(
            std::vector<StateTolerances> tol1,
            std::vector<StateTolerances> tol2){
            for (size_t i = 0; i < DOF; i++)
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

    JointTrajectoryPoint error_state_;
    Params params_;
    SegmentTolerances tolerances_;
};

TEST_F(TolerancesFixture, gets_segment_position_tolerances_from_params)
{
    // testing only waypoint position tolerance
    for (auto &&state_tolerance : tolerances_.state_tolerance)
    {
        state_tolerance = DEFAULT_STATE_TOLREANCE;
    }

    auto tolerances_read = get_segment_tolerances(params_);

    ASSERT_TRUE(compareWaypointTolerances(
        tolerances_read.state_tolerance,
        tolerances_.state_tolerance
        ));    
}

TEST_F(TolerancesFixture, error_state_violates_position_tolerances)
{
    tolerances_ = get_segment_tolerances(params_);

    // one joint violates tolerance
    error_state_.positions[1] = TOLERANCE_VIOLATED_VALUE;

    bool show_errors = true;
    EXPECT_FALSE(check_state_tolerance(
        DOF, error_state_, tolerances_.state_tolerance,
        show_errors));
}

TEST_F(TolerancesFixture, error_state_respects_position_tolerances)
{   
    tolerances_ = get_segment_tolerances(params_);

    // one joint violates tolerance
    error_state_.positions[1] = TOLERANCE_RESPECTED_VALUE;

    EXPECT_TRUE(check_state_tolerance(
        DOF, error_state_, tolerances_.state_tolerance));
}


int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  return result;
}

