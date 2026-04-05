#pragma once

#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>

#include "autoware_auto_planning_msgs/msg/path.hpp"
#include "autoware_planning_msgs/msg/path.hpp"
//
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
//
#include "nav_msgs/msg/occupancy_grid.hpp"
//
#include "autoware_auto_perception_msgs/msg/predicted_objects.hpp"
#include "autoware_perception_msgs/msg/predicted_objects.hpp"

#include <memory>

// ─── Path ────────────────────────────────────────────────────────────────────
nav_msgs::msg::OccupancyGrid boundsToOccupancyGrid(
  const autoware_planning_msgs::msg::Path & new_path, double resolution = 0.05)
{
  const auto & left_bound = new_path.left_bound;
  const auto & right_bound = new_path.right_bound;
  const auto & header = new_path.header;

  // bounding box
  double min_x = 1e9;
  double min_y = 1e9;
  double max_x = -1e9;
  double max_y = -1e9;
  for (const auto & p : left_bound) {
    min_x = std::min(min_x, p.x);
    max_x = std::max(max_x, p.x);
    min_y = std::min(min_y, p.y);
    max_y = std::max(max_y, p.y);
  }
  for (const auto & p : right_bound) {
    min_x = std::min(min_x, p.x);
    max_x = std::max(max_x, p.x);
    min_y = std::min(min_y, p.y);
    max_y = std::max(max_y, p.y);
  }

  int width = static_cast<int>((max_x - min_x) / resolution) + 2;
  int height = static_cast<int>((max_y - min_y) / resolution) + 2;

  // полигон: left_bound вперёд + right_bound назад = замкнутый контур дороги
  std::vector<cv::Point> poly;
  for (const auto & p : left_bound)
    poly.emplace_back(
      static_cast<int>((p.x - min_x) / resolution), static_cast<int>((p.y - min_y) / resolution));
  for (auto it = right_bound.rbegin(); it != right_bound.rend(); ++it)
    poly.emplace_back(
      static_cast<int>((it->x - min_x) / resolution),
      static_cast<int>((it->y - min_y) / resolution));

  // рисуем: всё = занято (100), внутри дороги = свободно (0)
  cv::Mat mat(height, width, CV_8SC1, cv::Scalar(100));
  std::vector<std::vector<cv::Point>> polys = {poly};
  cv::fillPoly(mat, polys, cv::Scalar(0));

  nav_msgs::msg::OccupancyGrid grid;
  grid.header = header;
  grid.info.resolution = resolution;
  grid.info.width = width;
  grid.info.height = height;
  grid.info.origin.position.x = min_x;
  grid.info.origin.position.y = min_y;
  grid.info.origin.position.z = 0.0;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(mat.data, mat.data + width * height);
  return grid;
}

inline autoware_auto_planning_msgs::msg::Path::SharedPtr toAutoPath(
  const autoware_planning_msgs::msg::Path & new_path)
{
  autoware_auto_planning_msgs::msg::Path auto_path;
  auto_path.header = new_path.header;

  for (const auto & pt : new_path.points) {
    autoware_auto_planning_msgs::msg::PathPoint auto_pt;
    auto_pt.pose = pt.pose;
    auto_pt.longitudinal_velocity_mps = pt.longitudinal_velocity_mps;
    auto_pt.lateral_velocity_mps = pt.lateral_velocity_mps;
    auto_pt.heading_rate_rps = pt.heading_rate_rps;
    auto_path.points.push_back(auto_pt);
  }

  // drivable_area - оставляем дефолтным
  // STEB строит drivable area сам через costmap/g2o
  auto_path.drivable_area = boundsToOccupancyGrid(new_path);

  return std::make_shared<autoware_auto_planning_msgs::msg::Path>(auto_path);
}

// ─── Trajectory (auto -> new) ──────────────────────────────────────────────
inline autoware_planning_msgs::msg::TrajectoryPoint fromAutoTrajectoryPoint(
  const autoware_auto_planning_msgs::msg::TrajectoryPoint & src)
{
  autoware_planning_msgs::msg::TrajectoryPoint dst;
  dst.pose = src.pose;
  dst.longitudinal_velocity_mps = src.longitudinal_velocity_mps;
  dst.lateral_velocity_mps = src.lateral_velocity_mps;
  dst.acceleration_mps2 = src.acceleration_mps2;
  dst.heading_rate_rps = src.heading_rate_rps;
  dst.front_wheel_angle_rad = src.front_wheel_angle_rad;
  dst.rear_wheel_angle_rad = src.rear_wheel_angle_rad;
  // time_from_start — нет в auto, остаётся нулевым (Duration{})
  return dst;
}

inline autoware_planning_msgs::msg::Trajectory fromAutoTrajectory(
  const autoware_auto_planning_msgs::msg::Trajectory & src)
{
  autoware_planning_msgs::msg::Trajectory dst;
  dst.header = src.header;
  for (const auto & pt : src.points) {
    dst.points.push_back(fromAutoTrajectoryPoint(pt));
  }
  return dst;
}

// ─── PredictedObjects ────────────────────────────────────────────────────────
inline autoware_auto_perception_msgs::msg::ObjectClassification toAutoClassification(
  const autoware_perception_msgs::msg::ObjectClassification & src)
{
  autoware_auto_perception_msgs::msg::ObjectClassification dst;
  dst.label = src.label;
  dst.probability = src.probability;
  return dst;
}

inline autoware_auto_perception_msgs::msg::Shape toAutoShape(
  const autoware_perception_msgs::msg::Shape & src)
{
  autoware_auto_perception_msgs::msg::Shape dst;
  dst.type = src.type;
  dst.footprint = src.footprint;    // geometry_msgs/Polygon — одинаков
  dst.dimensions = src.dimensions;  // geometry_msgs/Vector3 — одинаков
  return dst;
}

inline autoware_auto_perception_msgs::msg::PredictedPath toAutoPredictedPath(
  const autoware_perception_msgs::msg::PredictedPath & src)
{
  autoware_auto_perception_msgs::msg::PredictedPath dst;
  dst.time_step = src.time_step;
  dst.confidence = src.confidence;
  // BoundedVector<Pose, 100> — нельзя присвоить из std::vector напрямую
  for (const auto & pose : src.path) {
    dst.path.push_back(pose);
  }
  return dst;
}

inline autoware_auto_perception_msgs::msg::PredictedObject toAutoPredictedObject(
  const autoware_perception_msgs::msg::PredictedObject & src)
{
  autoware_auto_perception_msgs::msg::PredictedObject dst;
  dst.object_id = src.object_id;
  dst.existence_probability = src.existence_probability;

  for (const auto & c : src.classification) {
    dst.classification.push_back(toAutoClassification(c));
  }

  dst.shape = toAutoShape(src.shape);

  dst.kinematics.initial_pose_with_covariance = src.kinematics.initial_pose_with_covariance;
  dst.kinematics.initial_twist_with_covariance = src.kinematics.initial_twist_with_covariance;
  dst.kinematics.initial_acceleration_with_covariance =
    src.kinematics.initial_acceleration_with_covariance;

  for (const auto & path : src.kinematics.predicted_paths) {
    dst.kinematics.predicted_paths.push_back(toAutoPredictedPath(path));
  }
  return dst;
}

inline autoware_auto_perception_msgs::msg::PredictedObjects toAutoPredictedObjects(
  const autoware_perception_msgs::msg::PredictedObjects & src)
{
  autoware_auto_perception_msgs::msg::PredictedObjects dst;
  dst.header = src.header;
  for (const auto & obj : src.objects) {
    dst.objects.push_back(toAutoPredictedObject(obj));
  }
  return dst;
}
