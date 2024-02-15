// Copyright (c) 2024, Posidonia Technology d.o.o.
// All rights reserved.
//
// 
// Apache License, Version 2.0
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.


#ifndef PATH_FOLLOWING_CONTROLLER__TRAJECTORY_HPP_
#define PATH_FOLLOWING_CONTROLLER__TRAJECTORY_HPP_

#include <memory>
#include <vector>

// TODO: ADD INTERPOLATION functionality
//#include "path_following_controller/interpolation_methods.hpp"
#include "path_following_controller/visibility_control.h"
#include "rclcpp/time.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using JointTrajectoryPoint = trajectory_msgs::msg::JointTrajectoryPoint;
using JointTrajectory = trajectory_msgs::msg::JointTrajectory;

namespace path_following_controller
{

class Trajectory
{

public:
    /**
     * Update the \p trajectory_msg with a new trajectory and reset the \p index.
     * \param[in] joint_trajectory new trajectory
    */
    void update(std::shared_ptr<JointTrajectory> joint_trajectory);

    /**
     * Sampling trajectory at given \p index.
     * \param[out] output_state Sampled state
     */
    bool sample(JointTrajectoryPoint & output_state);
    
    std::shared_ptr<JointTrajectory> get_trajectory_msg() const { return trajectory_msg_; }
    
    void increment(){ index_ = index_ + 1; }
    bool is_at_first_point() const { return index_ == 0 
                                            && trajectory_msg_
                                            && !trajectory_msg_->points.empty(); }
    bool is_at_last_point() const { return index_ == (trajectory_msg_->points.size() - 1); }
    bool is_completed() const { return index_ >= trajectory_msg_->points.size(); }

    // for testing purposes
    uint get_index() const { return index_; }
    
private:
    void reset_index(){ index_ = 0; }
    std::shared_ptr<JointTrajectory> trajectory_msg_;
    uint index_ = 0;
};

} // namespace path_following_controller


#endif // PATH_FOLLOWING_CONTROLLER__TRAJECTORY_HPP_