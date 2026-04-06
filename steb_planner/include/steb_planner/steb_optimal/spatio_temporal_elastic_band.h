/*********************************************************************
 *
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2024, HeShan.
 * All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES, OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT, OR OTHERWISE, ARISING FROM, OUT OF, OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Author: HeShan
 *********************************************************************/

#ifndef SPATIO_TEMPORAL_ELASTIC_BAND_H_
#define SPATIO_TEMPORAL_ELASTIC_BAND_H_
#include "steb_planner/steb_obstacle/dynamic_obstacles.h"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <boost/optional.hpp>

#include <tf2/buffer_core.h>

#include <cassert>
#include <complex>
#include <iterator>
#include <string>

// G2O Types
#include "steb_planner/g2o_types/vertex_st.h"

namespace steb_planner
{

//! Container of poses that represent the spatial part of the trajectory
using PositionSequence = std::vector<VertexST *>;

class SpatioTemporalElasticBand
{
public:
  SpatioTemporalElasticBand();

  virtual ~SpatioTemporalElasticBand();

  // get the vertex of a pose by index
  VertexST * getPoseVertex(int index)
  {
    assert(index < sizePoses());
    return position_vec_.at(index);
  }

  // get the complete pose sequence
  PositionSequence & getPoses() { return position_vec_; };
  // get the complete pose sequence (read-only)
  const PositionSequence & getPosesReadOnly() const { return position_vec_; };

  // get the pose of the pose sequence by index
  Eigen::Vector3d & getPose(int index)
  {
    assert(index < sizePoses());
    return position_vec_.at(index)->position();
  }

  // get the pose of the pose sequence by index  (read-only)
  const Eigen::Vector3d & getPoseReadOnly(int index) const
  {
    assert(index < sizePoses());
    return position_vec_.at(index)->position();
  }

  // get the last pose in the pose sequence
  Eigen::Vector3d & getBackPose() { return position_vec_.back()->position(); }
  // get the last Pose in the pose sequence (read-only)
  const Eigen::Vector3d & getBackPoseReadOnly() const { return position_vec_.back()->position(); }

  // Append a new pose vertex to the back of the pose sequence,
  // fixed Mark the pose to be fixed or unfixed during trajectory optimization
  void addPose(const Eigen::Vector3d & pose, bool fixed = false);
  void addPose(const Eigen::Ref<const Eigen::Vector2d> & position, double time, bool fixed = false);
  void addPose(double x, double y, double time, bool fixed = false);

  // Insert a new pose vertex at index in the pose sequence
  // index element position inside the internal PoseSequence
  // PoseST element to insert into the internal PoseSequence
  void insertPose(int index, const Eigen::Vector3d & pose);
  void insertPose(int index, const Eigen::Ref<const Eigen::Vector2d> & position, double time);
  void insertPose(int index, double x, double y, double time);

  // Delete pose at index in the pose sequence
  void deletePose(int index);
  // Delete multiple elements from index
  void deletePoses(int index, int number);

  // Initialize a trajectory from a trajectory pose sequence.
  bool initTrajectoryToGoal(
    const tf2::Transform tf_map_to_ego, const TrajectoryPointsContainer & plan,
    Eigen::Vector3d handle_start);

  // Hot-Start from an existing trajectory with updated start and goal poses.
  void updateAndPruneTEB(
    const tf2::Transform tf_map_to_ego, Eigen::Vector3d handle_start,
    boost::optional<const Eigen::Vector3d &> new_start,
    boost::optional<const Eigen::Vector3d &> new_goal,
    const TrajectoryPointsContainer & global_plan, int min_samples = 3);

  // Resize the trajectory by removing or inserting a pose_ST depending on a reference
  // spatio-temporal resolution.
  void autoResize(
    double resize_resolution, double resize_resolution_hysteresis, int min_samples = 3,
    int max_samples = 1000, bool fast_mode = false);

  // Set a pose vertex at index of the pose sequence to be fixed or unfixed during optimization.
  void setPoseVertexFixed(int index, bool status);

  // clear all poses and timediffs from the trajectory.
  void clearTimedElasticBand();

  // Get the length of the internal pose sequence
  int sizePoses() const { return (int)position_vec_.size(); };

  // Calculate the length (accumulated euclidean distance) of the trajectory
  double getAccumulatedDistance() const;

  // Check whether the trajectory is initialized (nonzero pose and timediff sequences)
  bool isInit() const { return !position_vec_.empty(); }

  // Find the closest point on the trajectory w.r.t. to a provided reference point.
  int findClosestTrajectoryPose(
    const Eigen::Ref<const Eigen::Vector2d> & ref_point, double * distance = nullptr,
    int begin_idx = 0) const;

  // Find the closest point on the trajectory w.r.t. to a provided reference line.
  int findClosestTrajectoryPose(
    const Eigen::Ref<const Eigen::Vector2d> & ref_line_start,
    const Eigen::Ref<const Eigen::Vector2d> & ref_line_end, double * distance = nullptr) const;

  // Find the closest point on the trajectory w.r.t. to a provided reference polygon.
  int findClosestTrajectoryPose(
    const Point2dContainer & vertices, double * distance = nullptr) const;

  // Find the closest point on the trajectory w.r.t to a provided obstacle type
  int findClosestTrajectoryPose(const Obstacle & obstacle, double * distance = nullptr) const;

  bool findNearestTwoPose(
    boost::optional<const Eigen::Vector3d &> target_pose, int & nearest_idx,
    int & second_nearest_idx);

  void printSTEB(std::string print_header);

protected:
  PositionSequence
    position_vec_;  //!< Internal container storing the sequence of optimzable pose vertices
  tf2::Transform tf_last_map_to_ego;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace steb_planner

#endif /* SPATIO_TEMPORAL_ELASTIC_BAND_H_ */
