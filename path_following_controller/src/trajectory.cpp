// Copyright (c) 2024, Posidonia Technology d.o.o.
// All rights reserved.
//
// 
// Apache License, Version 2.0
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include <iostream>

#include "path_following_controller/trajectory.hpp"

#include "hardware_interface/macros.hpp"

using JointTrajectoryPoint = trajectory_msgs::msg::JointTrajectoryPoint;
using JointTrajectory = trajectory_msgs::msg::JointTrajectory;

namespace path_following_controller
{

bool Trajectory::sample(JointTrajectoryPoint & output_state)
{
    THROW_ON_NULLPTR(trajectory_msg_)
    if(trajectory_msg_->points.empty())
    {
        return false;
    }
    output_state = JointTrajectoryPoint();

    if (is_completed())
    {
        output_state = trajectory_msg_->points.back();
        return false;
    }    

    output_state = trajectory_msg_->points[index_];

    return true;
}

void Trajectory::update(std::shared_ptr<JointTrajectory> joint_trajectory)
{
    trajectory_msg_ = joint_trajectory;
    reset_index();
}
    
} // namespace path_following_controller