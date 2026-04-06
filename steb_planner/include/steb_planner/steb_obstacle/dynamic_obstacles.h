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

#ifndef OBSTACLES_H
#define OBSTACLES_H

#include "steb_planner/distance_calculations.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>

#include <cmath>
#include <complex>

#define UNUSED(x) (void)(x)

namespace steb_planner
{

// Obstacle Abstract class that defines the interface for modelling obstacles
class Obstacle
{
public:
  Obstacle() : dynamic_(false), centroid_velocity_(0.0), emergence_time_(0.0) {}

  virtual ~Obstacle() {}

  // Get centroid coordinates of the obstacle
  virtual const Eigen::Vector2d & getCentroid() const = 0;

  // Get centroid coordinates of the obstacle as complex number
  virtual std::complex<double> getCentroidCplx() const = 0;

  // Get piont coordinates of the line obstacle
  virtual const Eigen::Vector2d & getStarPoint() const = 0;
  virtual const Eigen::Vector2d & getEndPoint() const = 0;

  // Check if a given point collides with the obstacle
  virtual bool checkCollision(const Eigen::Vector2d & position, double min_dist) const = 0;

  // Check if a given line segment between two points intersects with the obstacle
  virtual bool checkLineIntersection(
    const Eigen::Vector2d & line_start, const Eigen::Vector2d & line_end,
    double min_dist = 0) const = 0;

  // Get the minimum euclidean distance to the obstacle (point as reference)
  virtual double getMinimumDistance(const Eigen::Vector2d & position) const = 0;

  // Get the minimum euclidean distance to the obstacle (line as reference)
  virtual double getMinimumDistance(
    const Eigen::Vector2d & line_start, const Eigen::Vector2d & line_end) const = 0;

  // Get the minimum euclidean distance to the obstacle (polygon as reference)
  virtual double getMinimumDistance(const Point2dContainer & polygon) const = 0;

  // Get the closest point on the boundary of the obstacle w.r.t. a specified reference position
  virtual Eigen::Vector2d getClosestPoint(const Eigen::Vector2d & position) const = 0;

  // Get the minimum spatiotemporal distance to the dynamic point obstacle
  virtual double getMinimumSpatioTemporalDistance(
    const Eigen::Vector2d & position, double t) const = 0;

  // Get the minimum spatiotemporal distance to the dynamic line obstacle
  virtual double getMinimumSpatioTemporalDistance(
    const Eigen::Vector2d & line_start, const Eigen::Vector2d & line_end, double t) const = 0;

  // Check if the obstacle is a moving with a (non-zero) velocity
  bool isDynamic() const { return dynamic_; }

  // Set the 2d velocity (vx, vy) of the obstacle w.r.t to the centroid
  void setCentroidVelocity(const double & vel)
  {
    centroid_velocity_ = vel;
    dynamic_ = true;
  }

  // Get the obstacle velocity
  const double & getCentroidVelocity() const { return centroid_velocity_; }

  // set the obstacle emergency time at a position
  void setEmergenceTime(const double & time) { emergence_time_ = time; }

  // get the emergence time
  const double & getEmergenceTime() const { return emergence_time_; }

protected:
  bool dynamic_;  // if obstacle is a moving obstacle
  double centroid_velocity_;
  double emergence_time_;  // emergence time of obstacles at the position

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//! Abbrev. for shared obstacle pointers
typedef std::shared_ptr<Obstacle> ObstaclePtr;
//! Abbrev. for shared obstacle const pointers
typedef std::shared_ptr<const Obstacle> ObstacleConstPtr;
//! Abbrev. for containers storing multiple obstacles
typedef std::vector<ObstaclePtr> ObstContainer;

// PillObstacle of Implements a 2D pill/stadium/capsular-shaped obstacle (line + distance/radius)
class STEBPillObstacle : public Obstacle
{
public:
  STEBPillObstacle() : Obstacle()
  {
    start_.setZero();
    end_.setZero();
    centroid_.setZero();
  }

  // Construct LineObstacle using 2d position vectors as start and end of the line
  STEBPillObstacle(
    const Eigen::Ref<const Eigen::Vector2d> & line_start,
    const Eigen::Ref<const Eigen::Vector2d> & line_end, double radius, double t)
  : Obstacle(), start_(line_start), end_(line_end), radius_(radius)
  {
    calcCentroid();
    setEmergenceTime(t);
  }

  // Construct LineObstacle using start and end coordinates
  STEBPillObstacle(double x1, double y1, double x2, double y2, double radius)
  : Obstacle(), radius_(radius)
  {
    start_.x() = x1;
    start_.y() = y1;
    end_.x() = x2;
    end_.y() = y2;
    calcCentroid();
  }

  virtual bool checkCollision(const Eigen::Vector2d & point, double min_dist) const
  {
    return getMinimumDistance(point) <= min_dist;
  }

  virtual bool checkLineIntersection(
    const Eigen::Vector2d & line_start, const Eigen::Vector2d & line_end, double min_dist = 0) const
  {
    UNUSED(min_dist);
    return check_line_segments_intersection_2d(line_start, line_end, start_, end_);
  }

  virtual double getMinimumDistance(const Eigen::Vector2d & position) const
  {
    return distance_point_to_segment_2d(position, start_, end_) - radius_;
  }

  virtual double getMinimumDistance(
    const Eigen::Vector2d & line_start, const Eigen::Vector2d & line_end) const
  {
    return distance_segment_to_segment_2d(start_, end_, line_start, line_end) - radius_;
  }

  virtual double getMinimumDistance(const Point2dContainer & polygon) const
  {
    return distance_segment_to_polygon_2d(start_, end_, polygon) - radius_;
  }

  virtual Eigen::Vector2d getClosestPoint(const Eigen::Vector2d & position) const
  {
    Eigen::Vector2d closed_point_line = closest_point_on_line_segment_2d(position, start_, end_);
    return closed_point_line + radius_ * (position - closed_point_line).normalized();
  }

  virtual double getMinimumSpatioTemporalDistance(const Eigen::Vector2d & position, double t) const
  {
    double distance_2d = distance_point_to_segment_2d(position, start_, end_);
    double distance_3d =
      std::pow(std::pow(distance_2d, 2) + std::pow(t - emergence_time_, 2), 0.5) - radius_;
    return distance_3d;
  }

  // implements getMinimumSpatioTemporalDistance() of the base class
  virtual double getMinimumSpatioTemporalDistance(
    const Eigen::Vector2d & line_start, const Eigen::Vector2d & line_end, double t) const
  {
    double distance_2d = distance_segment_to_segment_2d(start_, end_, line_start, line_end);
    double distance_3d =
      std::pow(std::pow(distance_2d, 2) + std::pow(t - emergence_time_, 2), 0.5) - radius_;
    return distance_3d;
  }

  virtual const Eigen::Vector2d & getCentroid() const { return centroid_; }

  virtual std::complex<double> getCentroidCplx() const
  {
    return std::complex<double>(centroid_.x(), centroid_.y());
  }

  // Access or modify line
  const Eigen::Vector2d & start() const { return start_; }
  void setStart(const Eigen::Ref<const Eigen::Vector2d> & start)
  {
    start_ = start;
    calcCentroid();
  }
  const Eigen::Vector2d & end() const { return end_; }
  void setEnd(const Eigen::Ref<const Eigen::Vector2d> & end)
  {
    end_ = end;
    calcCentroid();
  }

  virtual const Eigen::Vector2d & getStarPoint() const { return start_; };
  virtual const Eigen::Vector2d & getEndPoint() const { return end_; };

protected:
  void calcCentroid() { centroid_ = 0.5 * (start_ + end_); }

private:
  Eigen::Vector2d start_;
  Eigen::Vector2d end_;
  Eigen::Vector2d centroid_;
  double radius_ = 0.0;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace steb_planner

#endif /* OBSTACLES_H */
