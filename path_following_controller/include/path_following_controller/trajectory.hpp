// Copyright (c) 2024, Posidonia Technology d.o.o.
// All rights reserved.
//
// Proprietary License
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

namespace path_following_controller
{

using JointTrajectoryPoint = trajectory_msgs::msg::JointTrajectoryPoint;
using JointTrajectory = trajectory_msgs::msg::JointTrajectory;

using TrajectoryPointIter = std::vector<JointTrajectoryPoint>::iterator;
using TrajectoryPointConstIter = std::vector<JointTrajectoryPoint>::const_iterator;    

class Trajectory
{
public:
    Trajectory();
    explicit Trajectory(std::shared_ptr<JointTrajectory> joint_trajectory);


    /**
     * Update the \p trajectory_msg with a new trajectory and reset the \p index.
     * \param[in] joint_trajectory new trajectory
    */
    void update(std::shared_ptr<JointTrajectory> joint_trajectory);

    /// Find the segment (made up of 2 points) and its expected state from the
    /// containing trajectory.
    /**
     * Sampling trajectory at given \p index.
     * If position in the \p end_segment_itr is missing it will be deduced from provided velocity, or
     * acceleration respectively. Deduction assumes that the provided velocity or acceleration have to
     * be reached at the time defined in the segment.
     *
     * Specific case returns for start_segment_itr and end_segment_itr:
     * - Sampling at index:
     * - Sampling exactly on an index:
     *    start_segment_itr = point at index, end_segment_itr = point at index + 1
     * - Sampling between indices:
     *    interpolation <NOT IMPLEMENTED>
     * - Sampling after last point returns last point:
     *    start_segment_itr = --end(), end_segment_itr = end()
     * - Sampling empty msg
     *    return false
     *
     * \param[in] interpolation_method <NOT IMPLEMENTED> Specify whether splines, another method, or no interpolation at
     * all. \param[out] output_state Sampled state. \param[out] start_segment_itr
     * Iterator to the segment start. See description above. \param[out] end_segment_itr
     * Iterator to the segment end. See description above.
     */
    bool sample(
        /*const interpolation_methods::InterpolationMethod interpolation_method,*/
        JointTrajectoryPoint & output_state,
        TrajectoryPointConstIter & start_segment_itr, TrajectoryPointConstIter & end_segment_itr);

    TrajectoryPointConstIter begin() const;
    TrajectoryPointConstIter end() const;
    rclcpp::Time time_from_start() const;
    
    bool has_trajectory_msg() const;    
    bool has_nontrivial_msg() const;
    std::shared_ptr<JointTrajectory> 
      get_trajectory_msg() const { return trajectory_msg_; }
    
    void reset_index(){ index_ = 0; }
    uint get_index() const { return index_; }
    void increment(){ index_ = index_ + 1; }
    
    bool is_at_first_point() const { 
      return index_ == 0
             && has_nontrivial_msg();}
  
    bool is_at_last_point() const { return index_ == (trajectory_msg_->points.size() - 1); }
    bool is_completed() const { return index_ >= trajectory_msg_->points.size(); }

private:

    std::shared_ptr<JointTrajectory> trajectory_msg_;
    uint index_;
};
    

// REVIEW: MOVE TO UTILS
/**
 * \return The map between \p t1 indices (implicitly encoded in return vector indices) to \p t2
 * indices. If \p t1 is <tt>"{C, B}"</tt> and \p t2 is <tt>"{A, B, C, D}"</tt>, the associated
 * mapping vector is <tt>"{2, 1}"</tt>.
 */
template <class T>
inline std::vector<size_t> mapping(const T & t1, const T & t2)
{
  // t1 must be a subset of t2
  if (t1.size() > t2.size())
  {
    return std::vector<size_t>();
  }

  std::vector<size_t> mapping_vector(t1.size());  // Return value
  for (auto t1_it = t1.begin(); t1_it != t1.end(); ++t1_it)
  {
    auto t2_it = std::find(t2.begin(), t2.end(), *t1_it);
    if (t2.end() == t2_it)
    {
      return std::vector<size_t>();
    }
    else
    {
      const size_t t1_dist = std::distance(t1.begin(), t1_it);
      const size_t t2_dist = std::distance(t2.begin(), t2_it);
      mapping_vector[t1_dist] = t2_dist;
    }
  }
  return mapping_vector;
}

} // namespace path_following_controller


#endif // PATH_FOLLOWING_CONTROLLER__TRAJECTORY_HPP_