// Copyright (c) 2024, Posidonia Technology d.o.o.
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.


#include <gmock/gmock.h>

#include "path_following_controller/tolerances.hpp"

using namespace path_following_controller;
using namespace std;

using JointTrajectoryPoint = trajectory_msgs::msg::JointTrajectoryPoint;
using JointTrajectory = trajectory_msgs::msg::JointTrajectory;

using TrajectoryPointIter = std::vector<JointTrajectoryPoint>::iterator;
using TrajectoryPointConstIter = std::vector<JointTrajectoryPoint>::const_iterator;  

class TestTrajectoryFixture : public testing::Test
{
    public:
        TestTrajectoryFixture(){

        }
        ~TestTrajectoryFixture(){}

        void SetUp(){
        }

        void TearDown(){}

    private:
        
};

TEST_F(TestTrajectoryFixture, test1)
{
}

