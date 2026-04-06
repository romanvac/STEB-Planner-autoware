//
// Created by hs on 24-1-18.
//
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

#ifndef SRC_VIA_AREA_H_
#define SRC_VIA_AREA_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <boost/pointer_cast.hpp>
#include <boost/shared_ptr.hpp>

namespace steb_planner
{

// via_area: Class that defines the interface for modelling via_area
class ViaArea
{
public:
  ViaArea() : left_bound_(0.0), right_bound_(0.0), via_time_(0.0) {}

  ViaArea(const Eigen::Ref<const Eigen::Vector3d> & via_point, const double & via_time)
  : via_point_(via_point), left_bound_(0.0), right_bound_(0.0), via_time_(via_time)
  {
  }

  virtual ~ViaArea() {}

  void setViaPoint(const Eigen::Ref<const Eigen::Vector3d> & via_point) { via_point_ = via_point; }
  void setViaPoint(double x, double y, double z) { via_point_ = {x, y, z}; }

  const Eigen::Vector3d & getViaPoint() const { return via_point_; }

  void setLeftBound(const double & left_bound) { left_bound_ = left_bound; }
  void setRightBound(const double & right_bound) { right_bound_ = right_bound; }

  const double & getLeftBound() const { return left_bound_; }
  const double & getRightBound() const { return right_bound_; }

  void setViaTime(const double & time) { via_time_ = time; }

  const double & getViaTime() const { return via_time_; }

protected:
  Eigen::Vector3d via_point_;
  double left_bound_;
  double right_bound_;
  double via_time_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//! Abbrev. for shared obstacle pointers
typedef boost::shared_ptr<ViaArea> ViaAreaPtr;
//! Abbrev. for shared obstacle const pointers
typedef boost::shared_ptr<const ViaArea> ViaAreaConstPtr;
//! Abbrev. for containers storing multiple obstacles
typedef std::vector<ViaAreaPtr> ViaAreaContainer;

}  // namespace steb_planner

#endif  // SRC_VIA_AREA_H_
