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
using TrajectoryPointIter = std::vector<trajectory_msgs::msg::JointTrajectoryPoint>::iterator;
using TrajectoryPointConstIter =
  std::vector<trajectory_msgs::msg::JointTrajectoryPoint>::const_iterator;    
    
class Trajectory
{
public:
    Trajectory();
    explicit Trajectory(std::shared_ptr<trajectory_msgs::msg::JointTrajectory> joint_trajectory);

    void update(std::shared_ptr<trajectory_msgs::msg::JointTrajectory> joint_trajectory);

    /// Find the segment (made up of 2 points) and its expected state from the
    /// containing trajectory.
    /**
     * Sampling trajectory at given \p sample_time.
     * If position in the \p end_segment_itr is missing it will be deduced from provided velocity, or
     * acceleration respectively. Deduction assumes that the provided velocity or acceleration have to
     * be reached at the time defined in the segment.
     *
     * Specific case returns for start_segment_itr and end_segment_itr:
     * - Sampling before the trajectory start:
     *   start_segment_itr = begin(), end_segment_itr = begin()
     * - Sampling exactly on a point of the trajectory:
     *    start_segment_itr = iterator where point is, end_segment_itr = iterator after
     * start_segment_itr
     * - Sampling between points:
     *    start_segment_itr = iterator before the sampled point, end_segment_itr = iterator after
     * start_segment_itr
     * - Sampling after entire trajectory:
     *    start_segment_itr = --end(), end_segment_itr = end()
     * - Sampling empty msg or before the time given in set_point_before_trajectory_msg()
     *    return false
     *
     * \param[in] sample_time Time at which trajectory will be sampled.
     * \param[in] interpolation_method Specify whether splines, another method, or no interpolation at
     * all. \param[out] expected_state Calculated new at \p sample_time. \param[out] start_segment_itr
     * Iterator to the start segment for given \p sample_time. See description above. \param[out]
     * end_segment_itr Iterator to the end segment for given \p sample_time. See description above.
     */
    bool sample(
        /*const interpolation_methods::InterpolationMethod interpolation_method,*/
        trajectory_msgs::msg::JointTrajectoryPoint & output_state,
        TrajectoryPointConstIter & start_segment_itr, TrajectoryPointConstIter & end_segment_itr);

    TrajectoryPointConstIter begin() const;
    
    TrajectoryPointConstIter end() const;
    
    rclcpp::Time time_from_start() const;
    
    bool has_trajectory_msg() const;
    
    bool has_nontrivial_msg() const;

    void reset_index(){ index_ = 0; }
    
    std::shared_ptr<trajectory_msgs::msg::JointTrajectory> get_trajectory_msg() const{
        return trajectory_msg_;
    }
    
    int get_index(){ return index_; }
    bool is_sampled_already() const { return !is_first_sample_; }
private:
    using JointTrajectoryPoint = trajectory_msgs::msg::JointTrajectoryPoint;
    using JointTrajectory = trajectory_msgs::msg::JointTrajectory;

    std::shared_ptr<JointTrajectory> trajectory_msg_;
    rclcpp::Time trajectory_start_time_;

    JointTrajectoryPoint state_before_new_trajectory_;

    int index_;
    
    bool is_first_sample_ = true;
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