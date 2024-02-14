// Copyright (c) 2024, Posidonia Technology d.o.o.
// All rights reserved.
//
// 
// Apache License, Version 2.0
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.


#include <gmock/gmock.h>

#include "path_following_controller/trajectory.hpp"

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
            trajectory = make_shared<Trajectory>();
            full_msg = make_shared<JointTrajectory>();
            empty_msg = make_shared<JointTrajectory>();
        }
        ~TestTrajectoryFixture(){}

        void SetUp(){

            // populate full_msg with 3 points
            full_msg->header.stamp = rclcpp::Time(0);
            for (size_t i=1; i<=3; i++)
            {
                JointTrajectoryPoint p;
                auto value = static_cast<double>(i);
                p.positions.push_back(value);
                p.time_from_start = rclcpp::Duration::from_seconds(value);
                full_msg->points.push_back(p);
            }
        }

        void TearDown(){}
    
        shared_ptr<Trajectory> trajectory;
        JointTrajectoryPoint expected_point;
        TrajectoryPointConstIter start, end;

        shared_ptr<JointTrajectory> empty_msg;
        shared_ptr<JointTrajectory> full_msg;

    private:
        
};

TEST_F(TestTrajectoryFixture, init_traj_without_msg)
{
    EXPECT_FALSE(trajectory->has_trajectory_msg());
    ASSERT_ANY_THROW(trajectory->sample(expected_point, start, end));
}

TEST_F(TestTrajectoryFixture, init_traj_empty_msg)
{
    trajectory->update(empty_msg);

    EXPECT_FALSE(trajectory->sample(expected_point, start, end));

    EXPECT_EQ(trajectory->begin(), start);
    EXPECT_EQ(trajectory->begin(), end);    
}

TEST_F(TestTrajectoryFixture, index_management)
{
    trajectory->update(empty_msg);

    EXPECT_EQ(0, trajectory->get_index());
    EXPECT_TRUE(trajectory->is_completed());

    EXPECT_FALSE(trajectory->is_at_first_point());
    EXPECT_FALSE(trajectory->is_at_last_point());

    trajectory->increment();
    EXPECT_EQ(1, trajectory->get_index());
    trajectory->reset_index();
    EXPECT_EQ(0, trajectory->get_index());
}

TEST_F(TestTrajectoryFixture, sampling)
{
    trajectory->update(full_msg);

    auto temp2 = full_msg->points[0];
    auto temp = expected_point;
    // at index = 0;
    EXPECT_TRUE(trajectory->sample(expected_point, start, end));
    EXPECT_EQ(expected_point.positions, full_msg->points[0].positions);
    EXPECT_TRUE(trajectory->is_at_first_point());
    EXPECT_EQ(trajectory->begin(), start);

    // at index = 1;
    trajectory->increment();
    EXPECT_TRUE(trajectory->sample(expected_point, start, end));
    EXPECT_EQ(expected_point.positions, full_msg->points[1].positions);

    // at index = 2;
    trajectory->increment();
    EXPECT_TRUE(trajectory->sample(expected_point, start, end));
    EXPECT_EQ(expected_point.positions, full_msg->points[2].positions);
    EXPECT_TRUE(trajectory->is_at_last_point());

    // at index = 3, out of range;
    trajectory->increment();
    EXPECT_FALSE(trajectory->sample(expected_point, start, end));
    EXPECT_EQ(expected_point.positions, full_msg->points[2].positions);
    EXPECT_TRUE(trajectory->is_completed());
}