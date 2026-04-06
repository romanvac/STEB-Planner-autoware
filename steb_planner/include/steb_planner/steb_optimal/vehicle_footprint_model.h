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

#ifndef VEHICLE_FOOTPRINT_MODEL_H
#define VEHICLE_FOOTPRINT_MODEL_H

#include <visualization_msgs/msg/marker.hpp>

#include <boost/shared_ptr.hpp>

#include <steb_planner/steb_obstacle/dynamic_obstacles.h>
#include <steb_planner/steb_optimal/pose_se2.h>

namespace steb_planner
{

class BaseVehicleFootprintModel
{
public:
  // Default constructor of the abstract obstacle class
  BaseVehicleFootprintModel() {}

  // Virtual destructor
  virtual ~BaseVehicleFootprintModel() {}

  // Calculate the distance between the vehicle and an obstacle
  virtual double calculateSpatioDistance(
    const PoseSE2 & current_pose, const Obstacle * obstacle) const = 0;

  // calculate the distance between the vehicle and the predicted location of an obstacle at time t
  virtual double calculateSpatioTemporalDistance(
    const PoseSE2 & current_pose, double t, const Obstacle * obstacle) const = 0;

  // Visualize the robot using a markers
  virtual void visualizeVehicle(
    const PoseSE2 & current_pose, std::vector<visualization_msgs::msg::Marker> & markers,
    const std_msgs::msg::ColorRGBA & color) const = 0;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//! Abbrev. for shared obstacle pointers
typedef boost::shared_ptr<BaseVehicleFootprintModel> VehicleFootprintModelPtr;
//! Abbrev. for shared obstacle const pointers
typedef boost::shared_ptr<const BaseVehicleFootprintModel> VehicleFootprintModelConstPtr;

// PointVehicleShape Class that defines a point-vehicle
class PointVehicleFootprint : public BaseVehicleFootprintModel
{
public:
  PointVehicleFootprint() {}

  PointVehicleFootprint(const double min_obstacle_dist) : min_obstacle_dist_(min_obstacle_dist) {}

  virtual ~PointVehicleFootprint() {}

  virtual double calculateSpatioDistance(
    const PoseSE2 & current_pose, const Obstacle * obstacle) const
  {
    return obstacle->getMinimumDistance(current_pose.position());
  }

  virtual double calculateSpatioTemporalDistance(
    const PoseSE2 & current_pose, double t, const Obstacle * obstacle) const
  {
    return obstacle->getMinimumSpatioTemporalDistance(current_pose.position(), t);
  }

  // Visualize the robot using a markers
  virtual void visualizeVehicle(
    const PoseSE2 & current_pose, std::vector<visualization_msgs::msg::Marker> & markers,
    const std_msgs::msg::ColorRGBA & color) const
  {
    // point footprint
    markers.push_back(visualization_msgs::msg::Marker());
    visualization_msgs::msg::Marker & marker = markers.back();
    marker.type = visualization_msgs::msg::Marker::POINTS;
    current_pose.toPoseMsg(marker.pose);  // all points are transformed into the robot frame!
    marker.points.push_back(geometry_msgs::msg::Point());
    marker.scale.x = 0.025;
    marker.color = color;

    if (min_obstacle_dist_ <= 0) {
      return;
    }

    // footprint with min_obstacle_dist
    markers.push_back(visualization_msgs::msg::Marker());
    visualization_msgs::msg::Marker & marker2 = markers.back();
    marker2.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker2.scale.x = 0.025;
    marker2.color = color;
    current_pose.toPoseMsg(marker2.pose);  // all points are transformed into the robot frame!

    const double n = 9;
    const double r = min_obstacle_dist_;
    for (double theta = 0; theta <= 2 * M_PI; theta += M_PI / n) {
      geometry_msgs::msg::Point pt;
      pt.x = r * cos(theta);
      pt.y = r * sin(theta);
      marker2.points.push_back(pt);
    }
  }

private:
  const double min_obstacle_dist_ = 0.0;
};

// ThreeSameCirclesVehicleFootprint Class that approximates the vehicle with three same shifted
// circles
class ThreeCirclesVehicleFootprint : public BaseVehicleFootprintModel
{
public:
  /**
   * @brief Default constructor of the abstract obstacle class
   * @param front_offset shift the center of the front circle along the robot orientation starting
   * from the center at the rear axis (in meters)
   * @param mid_offset shift the center of the mid circle along the robot orientation starting from
   * the center at the rear axis (in meters)
   * @param radius radius of the circle
   */
  ThreeCirclesVehicleFootprint(double front_offset, double mid_offset, double radius)
  : front_offset_(front_offset), mid_offset_(mid_offset), radius_(radius)
  {
  }

  virtual ~ThreeCirclesVehicleFootprint() {}

  void setParameters(double front_offset, double mid_offset, double radius)
  {
    front_offset_ = front_offset;
    mid_offset_ = mid_offset;
    radius_ = radius;
  }

  virtual double calculateSpatioDistance(
    const PoseSE2 & current_pose, const Obstacle * obstacle) const
  {
    Eigen::Vector2d dir = current_pose.orientationUnitVec();
    double dist_front =
      obstacle->getMinimumDistance(current_pose.position() + front_offset_ * dir) - radius_;
    double dist_mid =
      obstacle->getMinimumDistance(current_pose.position() - mid_offset_ * dir) - radius_;
    double dist_rear = obstacle->getMinimumDistance(current_pose.position()) - radius_;
    return std::min(dist_front, std::min(dist_mid, dist_rear));
  }

  virtual double calculateSpatioTemporalDistance(
    const PoseSE2 & current_pose, double t, const Obstacle * obstacle) const
  {
    Eigen::Vector2d dir = current_pose.orientationUnitVec();
    double dist_front =
      obstacle->getMinimumSpatioTemporalDistance(current_pose.position() + front_offset_ * dir, t) -
      radius_;
    double dist_mid =
      obstacle->getMinimumSpatioTemporalDistance(current_pose.position() - mid_offset_ * dir, t) -
      radius_;
    double dist_rear =
      obstacle->getMinimumSpatioTemporalDistance(current_pose.position(), t) - radius_;
    return std::min(dist_front, std::min(dist_rear, dist_mid));
  }

  virtual void visualizeVehicle(
    const PoseSE2 & current_pose, std::vector<visualization_msgs::msg::Marker> & markers,
    const std_msgs::msg::ColorRGBA & color) const
  {
    Eigen::Vector2d dir = current_pose.orientationUnitVec();
    if (radius_ > 0) {
      markers.push_back(visualization_msgs::msg::Marker());
      visualization_msgs::msg::Marker & marker1 = markers.back();
      marker1.type = visualization_msgs::msg::Marker::CYLINDER;
      current_pose.toPoseMsg(marker1.pose);
      marker1.pose.position.x += front_offset_ * dir.x();
      marker1.pose.position.y += front_offset_ * dir.y();
      marker1.scale.x = marker1.scale.y = 2 * radius_;  // scale = diameter
                                                        //       marker1.scale.z = 0.05;
      marker1.color = color;
    }
  }

private:
  double front_offset_;
  double mid_offset_;
  double radius_;
};

// @class LineVehicleFootprint Class that approximates the vehicle with line segment (zero-width)
class LineVehicleFootprint : public BaseVehicleFootprintModel
{
public:
  /**
   * @brief Default constructor of the abstract obstacle class
   * @param line_start start coordinates (only x and y) of the line (w.r.t. robot center at (0,0))
   * @param line_end end coordinates (only x and y) of the line (w.r.t. robot center at (0,0))
   */
  LineVehicleFootprint(
    const geometry_msgs::msg::Point & line_start, const geometry_msgs::msg::Point & line_end)
  {
    setLine(line_start, line_end);
  }

  LineVehicleFootprint(
    const Eigen::Vector2d & line_start, const Eigen::Vector2d & line_end,
    const double min_obstacle_dist)
  : min_obstacle_dist_(min_obstacle_dist)
  {
    setLine(line_start, line_end);
  }

  virtual ~LineVehicleFootprint() {}

  void setLine(
    const geometry_msgs::msg::Point & line_start, const geometry_msgs::msg::Point & line_end)
  {
    line_start_.x() = line_start.x;
    line_start_.y() = line_start.y;
    line_end_.x() = line_end.x;
    line_end_.y() = line_end.y;
  }

  void setLine(const Eigen::Vector2d & line_start, const Eigen::Vector2d & line_end)
  {
    line_start_ = line_start;
    line_end_ = line_end;
  }

  virtual double calculateSpatioDistance(
    const PoseSE2 & current_pose, const Obstacle * obstacle) const
  {
    Eigen::Vector2d line_start_world;
    Eigen::Vector2d line_end_world;
    transformToWorld(current_pose, line_start_world, line_end_world);
    return obstacle->getMinimumDistance(line_start_world, line_end_world);
  }

  virtual double calculateSpatioTemporalDistance(
    const PoseSE2 & current_pose, double t, const Obstacle * obstacle) const
  {
    Eigen::Vector2d line_start_world;
    Eigen::Vector2d line_end_world;
    transformToWorld(current_pose, line_start_world, line_end_world);
    return obstacle->getMinimumSpatioTemporalDistance(line_start_world, line_end_world, t);
  }

  virtual void visualizeVehicle(
    const PoseSE2 & current_pose, std::vector<visualization_msgs::msg::Marker> & markers,
    const std_msgs::msg::ColorRGBA & color) const
  {
    markers.push_back(visualization_msgs::msg::Marker());
    visualization_msgs::msg::Marker & marker = markers.back();
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    current_pose.toPoseMsg(marker.pose);  // all points are transformed into the robot frame!

    // line
    geometry_msgs::msg::Point line_start_world;
    line_start_world.x = line_start_.x();
    line_start_world.y = line_start_.y();
    line_start_world.z = 0;
    marker.points.push_back(line_start_world);

    geometry_msgs::msg::Point line_end_world;
    line_end_world.x = line_end_.x();
    line_end_world.y = line_end_.y();
    line_end_world.z = 0;
    marker.points.push_back(line_end_world);

    marker.scale.x = 0.025;
    marker.color = color;

    if (min_obstacle_dist_ <= 0) {
      return;
    }

    // footprint with min_obstacle_dist
    markers.push_back(visualization_msgs::msg::Marker());
    visualization_msgs::msg::Marker & marker2 = markers.back();
    marker2.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker2.scale.x = 0.025;
    marker2.color = color;
    current_pose.toPoseMsg(marker2.pose);  // all points are transformed into the robot frame!

    const double n = 9;
    const double r = min_obstacle_dist_;
    const double ori = atan2(line_end_.y() - line_start_.y(), line_end_.x() - line_start_.x());

    // first half-circle
    for (double theta = M_PI_2 + ori; theta <= 3 * M_PI_2 + ori; theta += M_PI / n) {
      geometry_msgs::msg::Point pt;
      pt.x = line_start_.x() + r * cos(theta);
      pt.y = line_start_.y() + r * sin(theta);
      marker2.points.push_back(pt);
    }

    // second half-circle
    for (double theta = -M_PI_2 + ori; theta <= M_PI_2 + ori; theta += M_PI / n) {
      geometry_msgs::msg::Point pt;
      pt.x = line_end_.x() + r * cos(theta);
      pt.y = line_end_.y() + r * sin(theta);
      marker2.points.push_back(pt);
    }

    // duplicate 1st point to close shape
    geometry_msgs::msg::Point pt;
    pt.x = line_start_.x() + r * cos(M_PI_2 + ori);
    pt.y = line_start_.y() + r * sin(M_PI_2 + ori);
    marker2.points.push_back(pt);
  }

private:
  /**
   * @brief Transforms a line to the world frame manually
   * @param current_pose Current robot pose
   * @param[out] line_start line_start_ in the world frame
   * @param[out] line_end line_end_ in the world frame
   */
  void transformToWorld(
    const PoseSE2 & current_pose, Eigen::Vector2d & line_start_world,
    Eigen::Vector2d & line_end_world) const
  {
    double cos_th = std::cos(current_pose.theta());
    double sin_th = std::sin(current_pose.theta());
    line_start_world.x() = current_pose.x() + cos_th * line_start_.x() - sin_th * line_start_.y();
    line_start_world.y() = current_pose.y() + sin_th * line_start_.x() + cos_th * line_start_.y();
    line_end_world.x() = current_pose.x() + cos_th * line_end_.x() - sin_th * line_end_.y();
    line_end_world.y() = current_pose.y() + sin_th * line_end_.x() + cos_th * line_end_.y();
  }

  Eigen::Vector2d line_start_;
  Eigen::Vector2d line_end_;
  const double min_obstacle_dist_ = 0.0;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace steb_planner

#endif /* VEHICLE_FOOTPRINT_MODEL_H */
