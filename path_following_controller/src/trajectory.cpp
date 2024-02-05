// Copyright (c) 2024, Posidonia Technology d.o.o.
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.
#include <iostream>

#include "path_following_controller/trajectory.hpp"

#include "hardware_interface/macros.hpp"

namespace path_following_controller
{

Trajectory::Trajectory() {}

Trajectory::Trajectory(std::shared_ptr<trajectory_msgs::msg::JointTrajectory> joint_trajectory)
    :   trajectory_msg_(joint_trajectory),
        trajectory_start_time_(static_cast<rclcpp::Time>(joint_trajectory->header.stamp)){}

bool Trajectory::sample(
    /* const interpolation_methods::InterpolationMethod interpolation_method, */
    trajectory_msgs::msg::JointTrajectoryPoint & output_state,
    TrajectoryPointConstIter & start_segment_itr, TrajectoryPointConstIter & end_segment_itr)
{
    THROW_ON_NULLPTR(trajectory_msg_)

    // In the case of emtpy vector, size() returns 0 as well.
    if (index_ >= trajectory_msg_->points.size())
    {
        start_segment_itr = end();
        end_segment_itr = end();
        return false;
    }

    const std::size_t last_index = trajectory_msg_->points.size() - 1;
    output_state = JointTrajectoryPoint();


    start_segment_itr = begin() + index_;
    end_segment_itr = begin() + ( index_ + 1 );
    output_state = trajectory_msg_->points[index_];

    if(index_ == last_index)
    {   // the trajectories in msg may have empty velocities/accel, so resize them
        if (output_state.velocities.empty())
            output_state.velocities.resize(output_state.positions.size(), 0.0);
        if (output_state.accelerations.empty())
            output_state.accelerations.resize(output_state.positions.size(), 0.0);
    }
    
    index_ = index_ + 1;

    return true;
}

void Trajectory::update(std::shared_ptr<trajectory_msgs::msg::JointTrajectory> joint_trajectory)
{
    trajectory_msg_ = joint_trajectory;
    trajectory_start_time_ = static_cast<rclcpp::Time>(joint_trajectory->header.stamp);
    reset_index();
}

TrajectoryPointConstIter Trajectory::begin() const
{
    THROW_ON_NULLPTR(trajectory_msg_)
    return trajectory_msg_->points.begin();
}

TrajectoryPointConstIter Trajectory::end() const
{
    THROW_ON_NULLPTR(trajectory_msg_)
    return trajectory_msg_->points.end();
}

rclcpp::Time Trajectory::time_from_start() const { return trajectory_start_time_; }

bool Trajectory::has_trajectory_msg() const { return trajectory_msg_.get() != nullptr; }

bool Trajectory::has_nontrivial_msg() const{
    return has_trajectory_msg() && trajectory_msg_->points.size() > 1;
}

    
} // namespace path_following_controller