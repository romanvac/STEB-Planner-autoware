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

#include <steb_planner/steb_optimal/spatio_temporal_elastic_band.h>

#include <cstddef>
#include <limits>
#include <utility>

namespace steb_planner
{

SpatioTemporalElasticBand::SpatioTemporalElasticBand()
{
  tf_last_map_to_ego.setIdentity();
}

SpatioTemporalElasticBand::~SpatioTemporalElasticBand()
{
  RCLCPP_DEBUG(rclcpp::get_logger("steb_planner"), "Destructor Timed_Elastic_Band...");
  clearTimedElasticBand();
}

bool SpatioTemporalElasticBand::initTrajectoryToGoal(
  const tf2::Transform tf_map_to_ego, const TrajectoryPointsContainer & plan,
  Eigen::Vector3d handle_start)
{
  tf_last_map_to_ego = tf_map_to_ego;

  if (!isInit()) {
    Eigen::Vector3d start{plan.front().x(), plan.front().y(), plan.front().w()};
    Eigen::Vector3d goal{plan.back().x(), plan.back().y(), plan.back().w()};

    addPose(handle_start);
    setPoseVertexFixed(0, true);

    addPose(start);               // add starting point with given orientation
    setPoseVertexFixed(1, true);  // StartConf is a fixed constraint during optimization

    for (int i = 1; i < (int)plan.size(); ++i) {
      Eigen::Vector3d intermediate_pose{plan[i].x(), plan[i].y(), plan[i].w()};
      addPose(intermediate_pose);
      //      if (i < 5 && plan.size() > 5)
      //      {
      //        setPoseVertexFixed(i,true);
      //      }
    }
  } else  // size!=0
  {
    RCLCPP_WARN(
      rclcpp::get_logger("steb_planner"),
      "Cannot init STEB between given configuration and goal, because STEB vectors are not empty "
      "or STEB is already initialized");
    RCLCPP_WARN(
      rclcpp::get_logger("steb_planner"), "Number of STEB configurations: %d", sizePoses());
    return false;
  }

  return true;
}

void SpatioTemporalElasticBand::updateAndPruneTEB(
  const tf2::Transform tf_map_to_ego, Eigen::Vector3d handle_start,
  boost::optional<const Eigen::Vector3d &> new_start,
  boost::optional<const Eigen::Vector3d &> new_goal, const TrajectoryPointsContainer & global_plan,
  int min_samples)
{
  // first and simple approach: change only start confs (and virtual start conf for inital velocity)
  // TEST if optimizer can handle this "hard" placement

  // @hs: transform to new ego frame;
  tf2::Transform tf_new_ego_to_last_ego = tf_map_to_ego.inverse() * tf_last_map_to_ego;
  for (auto & pose_st : getPoses()) {
    tf2::Transform pose_last;
    pose_last.setIdentity();
    pose_last.setOrigin(tf2::Vector3(pose_st->x(), pose_st->y(), 0.0));

    tf2::Transform pose_new = tf_new_ego_to_last_ego * pose_last;
    pose_st->x() = pose_new.getOrigin().x();
    pose_st->y() = pose_new.getOrigin().y();
  }
  tf_last_map_to_ego = tf_map_to_ego;

  // if (false)
  if (new_start && sizePoses() > 0) {
    // find nearest state (using l2-norm) in order to prune the trajectory
    // (remove already passed states)
    //   Eigen::Vector2d dist_vector_0(new_start.get().x()- getPose(0).x(), new_start.get().y() -
    //   getPose(0).y()); double dist_cache = dist_vector_0.norm(); double dist_cache =
    //   (new_start.get() - getPose(0)).norm();
    double dist_cache = std::numeric_limits<double>::max();

    double dist;
    int lookahead = std::min<int>(
      sizePoses() - min_samples, 20);  // satisfy min_samples, otherwise max 10 samples

    int nearest_idx = -1;
    int second_nearest_idx = -1;
    for (int i = 0; i <= lookahead; ++i) {
      // 这里直接不要后面的点，但可能有些问题。
      if (getPoseReadOnly(i).x() < 0.0) continue;

      dist = std::sqrt(
        std::pow(new_start.get().x() - getPoseReadOnly(i).x(), 2) +
        std::pow(new_start.get().y() - getPoseReadOnly(i).y(), 2));
      if (dist < dist_cache) {
        dist_cache = dist;
        nearest_idx = i;
      }
    }
    int second_nearest_idx_candidate_1 = std::max<int>(0, nearest_idx - 1);
    int second_nearest_idx_candidate_2 = std::min<int>(nearest_idx + 1, sizePoses() - 1);

    if (second_nearest_idx_candidate_1 == nearest_idx)
      second_nearest_idx = second_nearest_idx_candidate_2;
    else if (second_nearest_idx_candidate_2 == nearest_idx)
      second_nearest_idx = second_nearest_idx_candidate_1;
    else {
      double dist_1 = std::sqrt(
        std::pow(new_start.get().x() - getPoseReadOnly(second_nearest_idx_candidate_1).x(), 2) +
        std::pow(new_start.get().y() - getPoseReadOnly(second_nearest_idx_candidate_1).y(), 2));
      double dist_2 = std::sqrt(
        std::pow(new_start.get().x() - getPoseReadOnly(second_nearest_idx_candidate_2).x(), 2) +
        std::pow(new_start.get().y() - getPoseReadOnly(second_nearest_idx_candidate_2).y(), 2));
      second_nearest_idx =
        dist_1 < dist_2 ? second_nearest_idx_candidate_1 : second_nearest_idx_candidate_2;
    }

    //    std::cout << "-------------- nearest_idx: "<< nearest_idx << ", " << second_nearest_idx <<
    //    std::endl;

    double current_pose_st_time = 0;
    // prune trajectory at the beginning (and extrapolate sequences at the end if the horizon is
    // fixed)
    if (nearest_idx != -1 && second_nearest_idx != -1) {
      int front_index = std::max<int>(nearest_idx, second_nearest_idx);
      int back_index = std::min<int>(nearest_idx, second_nearest_idx);

      // vector_A * vector_B = |v_A|*|v_B|*cos(theta)
      Eigen::Vector2d vector_a{
        new_start->x() - getPose(back_index).x(), new_start->y() - getPose(back_index).y()};
      Eigen::Vector2d vector_b{
        getPose(front_index).x() - getPose(back_index).x(),
        getPose(front_index).y() - getPose(back_index).y()};
      double proportion = (vector_a.dot(vector_b)) / vector_b.squaredNorm();

      current_pose_st_time =
        getPosesReadOnly().at(back_index)->t() +
        (getPosesReadOnly().at(front_index)->t() - getPosesReadOnly().at(back_index)->t()) *
          proportion;

      // nearest_idx is equal to the number of samples to be removed (since it counts from 0 ;-) )
      // WARNING delete starting at pose 1, and overwrite the original pose(0) with new_start, since
      // getPose(0) is fixed during optimization! delete first states such that the closest state is
      // the new first one
      //      deletePoses(0, std::max<int>(nearest_idx, second_nearest_idx));
      for (int j = 2; j < front_index; ++j) {
        deletePose(j);
      }
    }

    // 更新时间t
    for (auto & pose : getPoses()) {
      pose->t() -= current_pose_st_time;
    }

    // update start
    getPose(0) = std::move(handle_start);
    getPose(1) = *new_start;
  }

  //  UNUSED(global_plan);
  // 插入新的全局点
  if (true) {
    //    std::cout << "-------------- 0 --------------- "<< global_plan.size() <<std::endl;
    Eigen::Vector2d point;
    double dist_cache = std::numeric_limits<double>::max();
    int back_nearest_idx = -1;
    int back_second_nearest_idx = -1;
    Eigen::Vector2d point_back{getPoses().back()->x(), getPoses().back()->y()};
    //    std::cout <<"point back: "<< point_back << std::endl;
    for (int i = 0; i < (int)global_plan.size(); ++i) {
      point = {global_plan.at(i).x(), global_plan.at(i).y()};
      double dist = (point - point_back).norm();
      //      std::cout <<"i: "<< i <<": "<< point << ", "<< dist << std::endl;
      if (dist < dist_cache) {
        dist_cache = dist;
        back_nearest_idx = i;
      }
      //      std::cout <<", "<< back_nearest_idx <<"," <<dist_cache<< std::endl;
    }
    int second_nearest_idx_candidate_1 = std::max<int>(0, back_nearest_idx - 1);
    int second_nearest_idx_candidate_2 =
      std::min<int>(back_nearest_idx + 1, global_plan.size() - 1);

    //    std::cout << "-------------- back_nearest_idx0: "<< back_nearest_idx << ", " <<
    //    back_second_nearest_idx << std::endl;
    if (second_nearest_idx_candidate_1 == back_nearest_idx)
      back_second_nearest_idx = second_nearest_idx_candidate_2;
    else if (second_nearest_idx_candidate_2 == back_nearest_idx)
      back_second_nearest_idx = second_nearest_idx_candidate_1;
    else {
      double dist_1 = std::sqrt(
        std::pow(getPoses().back()->x() - global_plan.at(second_nearest_idx_candidate_1).x(), 2) +
        std::pow(getPoses().back()->y() - global_plan.at(second_nearest_idx_candidate_1).y(), 2));
      double dist_2 = std::sqrt(
        std::pow(getPoses().back()->x() - global_plan.at(second_nearest_idx_candidate_2).x(), 2) +
        std::pow(getPoses().back()->y() - global_plan.at(second_nearest_idx_candidate_2).y(), 2));
      back_second_nearest_idx =
        dist_1 < dist_2 ? second_nearest_idx_candidate_1 : second_nearest_idx_candidate_2;
    }

    //    std::cout << "-------------- back_nearest_idx1: "<< back_nearest_idx << ", " <<
    //    back_second_nearest_idx << std::endl;
    if (
      back_nearest_idx > -1 && back_nearest_idx < (int)global_plan.size() - min_samples &&
      back_second_nearest_idx > -1 &&
      back_second_nearest_idx < (int)global_plan.size() - min_samples) {
      // front is index in the front of target pose
      int front_index = std::max<int>(back_nearest_idx, back_second_nearest_idx);
      int back_index = std::min<int>(back_nearest_idx, back_second_nearest_idx);

      Eigen::Vector2d vector_a{
        point_back.x() - global_plan.at(back_index).x(),
        point_back.y() - global_plan.at(back_index).y()};
      Eigen::Vector2d vector_b{
        global_plan.at(front_index).x() - global_plan.at(back_index).x(),
        global_plan.at(front_index).y() - global_plan.at(back_index).y()};
      double proportion = (vector_a.dot(vector_b)) / vector_b.squaredNorm();

      //      std::cout << "-------------- back_nearest_idx2: "<< back_nearest_idx << ", " <<
      //      back_second_nearest_idx << std::endl;
      // front_index pose from global plan to steb_pose
      double new_time_start =
        getPosesReadOnly().back()->t() +
        (global_plan.at(front_index).w() - global_plan.at(back_index).w()) * proportion;
      Eigen::Vector3d front_pose{
        global_plan.at(front_index).x(), global_plan.at(front_index).y(), new_time_start};
      addPose(front_pose);
      for (int i = front_index + 1; i < (int)global_plan.size(); ++i) {
        Eigen::Vector3d intermediate_pose{
          global_plan[i].x(), global_plan[i].y(),
          global_plan[i].w() - global_plan[front_index].w() + new_time_start};
        addPose(intermediate_pose);
      }
    }
  } else {
    UNUSED(new_goal);
    if (new_goal && sizePoses() > 0) {
      getBackPose() = *new_goal;
    }
  }

  setPoseVertexFixed(sizePoses() - 1, false);
}

void SpatioTemporalElasticBand::clearTimedElasticBand()
{
  for (PositionSequence::iterator pose_it = position_vec_.begin(); pose_it != position_vec_.end();
       ++pose_it)
    delete *pose_it;
  position_vec_.clear();
}

void SpatioTemporalElasticBand::addPose(const Eigen::Vector3d & pose, bool fixed)
{
  VertexST * pose_vertex = new VertexST(pose, fixed);
  position_vec_.push_back(pose_vertex);
  return;
}

void SpatioTemporalElasticBand::addPose(
  const Eigen::Ref<const Eigen::Vector2d> & position, double time, bool fixed)
{
  VertexST * pose_vertex = new VertexST(position.x(), position.y(), time, fixed);
  position_vec_.push_back(pose_vertex);
  return;
}

void SpatioTemporalElasticBand::addPose(double x, double y, double time, bool fixed)
{
  VertexST * pose_vertex = new VertexST(x, y, time, fixed);
  position_vec_.push_back(pose_vertex);
  return;
}

void SpatioTemporalElasticBand::deletePose(int index)
{
  assert(index < static_cast<int>(position_vec_.size()));
  delete position_vec_.at(index);
  position_vec_.erase(position_vec_.begin() + index);
}

void SpatioTemporalElasticBand::deletePoses(int index, int number)
{
  assert(index + number <= (int)position_vec_.size());
  for (int i = index; i < index + number; ++i) delete position_vec_.at(i);
  position_vec_.erase(position_vec_.begin() + index, position_vec_.begin() + index + number);
}

void SpatioTemporalElasticBand::insertPose(int index, const Eigen::Vector3d & pose)
{
  VertexST * pose_vertex = new VertexST(pose);
  position_vec_.insert(position_vec_.begin() + index, pose_vertex);
}

void SpatioTemporalElasticBand::insertPose(
  int index, const Eigen::Ref<const Eigen::Vector2d> & position, double time)
{
  VertexST * pose_vertex = new VertexST(position.x(), position.y(), time);
  position_vec_.insert(position_vec_.begin() + index, pose_vertex);
}

void SpatioTemporalElasticBand::insertPose(int index, double x, double y, double time)
{
  VertexST * pose_vertex = new VertexST(x, y, time);
  position_vec_.insert(position_vec_.begin() + index, pose_vertex);
}

void SpatioTemporalElasticBand::setPoseVertexFixed(int index, bool status)
{
  assert(index < static_cast<int>(sizePoses()));
  position_vec_.at(index)->setFixed(status);
}

void SpatioTemporalElasticBand::autoResize(
  double resize_resolution, double resize_resolution_hysteresis, int min_samples, int max_samples,
  bool fast_mode)
{
  /// iterate through all STEB states and add/remove states!
  bool modified = true;

  for (int rep = 0; rep < 100 && modified; ++rep) {
    modified = false;
    for (int i = 1; i < sizePoses() - 1; ++i) {
      if (
        (getPose(i) - getPose(i + 1)).norm() > resize_resolution + resize_resolution_hysteresis &&
        sizePoses() < max_samples) {
        insertPose(
          i + 1, (getPose(i).x() + getPose(i + 1).x()) * 0.5,
          (getPose(i).y() + getPose(i + 1).y()) * 0.5, (getPose(i).z() + getPose(i + 1).z()) * 0.5);

        i--;  // check the updated pose diff again
        modified = true;
      } else if (
        (getPose(i) - getPose(i + 1)).norm() < resize_resolution - resize_resolution_hysteresis &&
        sizePoses() > min_samples)
      // only remove samples if size is larger than min_samples.
      {
        if (i < ((int)sizePoses() - 1)) {
          deletePose(i + 1);
          i--;  // check the updated pose diff again
        } else {
          // last motion should be adjusted, shift time to the interval before
          deletePose(i);
        }

        modified = true;
      }
    }
    if (fast_mode) break;
  }
}

double SpatioTemporalElasticBand::getAccumulatedDistance() const
{
  double dist = 0;

  for (int i = 1; i < sizePoses(); ++i) {
    Eigen::Vector2d dist_vector{
      getPoseReadOnly(i).x() - getPoseReadOnly(i - 1).x(),
      getPoseReadOnly(i).y() - getPoseReadOnly(i - 1).y()};
    dist += dist_vector.norm();
  }
  return dist;
}

int SpatioTemporalElasticBand::findClosestTrajectoryPose(
  const Eigen::Ref<const Eigen::Vector2d> & ref_point, double * distance, int begin_idx) const
{
  int n = sizePoses();
  if (begin_idx < 0 || begin_idx >= n) return -1;

  double min_dist_sq = std::numeric_limits<double>::max();
  int min_idx = -1;

  for (int i = begin_idx; i < n; i++) {
    Eigen::Vector2d point{getPoseReadOnly(i).x(), getPoseReadOnly(i).y()};
    double dist_sq = (ref_point - point).squaredNorm();
    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      min_idx = i;
    }
  }

  if (distance) *distance = std::sqrt(min_dist_sq);

  return min_idx;
}

int SpatioTemporalElasticBand::findClosestTrajectoryPose(
  const Eigen::Ref<const Eigen::Vector2d> & ref_line_start,
  const Eigen::Ref<const Eigen::Vector2d> & ref_line_end, double * distance) const
{
  double min_dist = std::numeric_limits<double>::max();
  int min_idx = -1;

  for (int i = 0; i < sizePoses(); i++) {
    Eigen::Vector2d point{getPoseReadOnly(i).x(), getPoseReadOnly(i).y()};
    double dist = distance_point_to_segment_2d(point, ref_line_start, ref_line_end);
    if (dist < min_dist) {
      min_dist = dist;
      min_idx = i;
    }
  }

  if (distance) *distance = min_dist;
  return min_idx;
}

int SpatioTemporalElasticBand::findClosestTrajectoryPose(
  const Point2dContainer & vertices, double * distance) const
{
  if (vertices.empty())
    return 0;
  else if (vertices.size() == 1)
    return findClosestTrajectoryPose(vertices.front());
  else if (vertices.size() == 2)
    return findClosestTrajectoryPose(vertices.front(), vertices.back());

  double min_dist = std::numeric_limits<double>::max();
  int min_idx = -1;

  for (int i = 0; i < sizePoses(); i++) {
    Eigen::Vector2d point{getPoseReadOnly(i).x(), getPoseReadOnly(i).y()};
    double dist_to_polygon = std::numeric_limits<double>::max();
    for (int j = 0; j < (int)vertices.size() - 1; ++j) {
      dist_to_polygon = std::min(
        dist_to_polygon, distance_point_to_segment_2d(point, vertices[j], vertices[j + 1]));
    }
    dist_to_polygon = std::min(
      dist_to_polygon, distance_point_to_segment_2d(point, vertices.back(), vertices.front()));
    if (dist_to_polygon < min_dist) {
      min_dist = dist_to_polygon;
      min_idx = i;
    }
  }

  if (distance) *distance = min_dist;

  return min_idx;
}

int SpatioTemporalElasticBand::findClosestTrajectoryPose(
  const Obstacle & obstacle, double * distance) const
{
  const STEBPillObstacle * lobst = dynamic_cast<const STEBPillObstacle *>(&obstacle);
  if (lobst) return findClosestTrajectoryPose(lobst->start(), lobst->end(), distance);

  return findClosestTrajectoryPose(obstacle.getCentroid(), distance);
}

bool SpatioTemporalElasticBand::findNearestTwoPose(
  boost::optional<const Eigen::Vector3d &> target_pose, int & nearest_idx, int & second_nearest_idx)
{
  double dist_cache = std::numeric_limits<double>::max();

  int nearest_idx_local = 0;
  for (int i = 0; i < sizePoses(); ++i) {
    // 这里直接不要后面的点，但可能有些问题。
    if (getPoseReadOnly(i).x() < 0.0) continue;

    double dist = std::sqrt(
      std::pow(target_pose.get().x() - getPoseReadOnly(i).x(), 2) +
      std::pow(target_pose.get().y() - getPoseReadOnly(i).y(), 2));
    if (dist < dist_cache) {
      dist_cache = dist;
      nearest_idx_local = i;
    }
  }
  nearest_idx = nearest_idx_local;
  int second_nearest_idx_candidate_1 = std::max<int>(0, nearest_idx - 1);
  int second_nearest_idx_candidate_2 = std::min<int>(nearest_idx + 1, sizePoses() - 1);
  double dist_1 = std::sqrt(
    std::pow(target_pose.get().x() - getPoseReadOnly(second_nearest_idx_candidate_1).x(), 2) +
    std::pow(target_pose.get().y() - getPoseReadOnly(second_nearest_idx_candidate_1).y(), 2));
  double dist_2 = std::sqrt(
    std::pow(target_pose.get().x() - getPoseReadOnly(second_nearest_idx_candidate_2).x(), 2) +
    std::pow(target_pose.get().y() - getPoseReadOnly(second_nearest_idx_candidate_2).y(), 2));
  second_nearest_idx =
    dist_1 < dist_2 ? second_nearest_idx_candidate_1 : second_nearest_idx_candidate_2;

  return true;
}

void SpatioTemporalElasticBand::printSTEB(std::string print_header)
{
  std::cout << "---------- " << print_header << " ----------" << std::endl;
  std::cout << "steb size: " << position_vec_.size() << std::endl;
  for (size_t i = 0; i < position_vec_.size(); ++i) {
    std::cout << "No." << i << " : ( " << position_vec_.at(i)->position().x() << ", "
              << position_vec_.at(i)->position().y() << ", " << position_vec_.at(i)->position().z()
              << " ) " << std::endl;
  }
}

}  // namespace steb_planner
