//
// Created by hs on 24-3-13.
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

#ifndef VERTEX_ST_H_
#define VERTEX_ST_H_

#include <g2o/config.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/hyper_graph_action.h>
#include <g2o/stuff/misc.h>

namespace steb_planner
{

// VertexPose: This class stores and wraps a ST pose (position and orientation) into a vertex that
// can be optimized via g2o
class VertexST : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
  VertexST(bool fixed = false)
  {
    setToOriginImpl();
    setFixed(fixed);
  }

  VertexST(const Eigen::Ref<const Eigen::Vector3d> & st, bool fixed = false)
  {
    _estimate = st;
    setFixed(fixed);
  }

  VertexST(double x, double y, double t, bool fixed = false)
  {
    _estimate.x() = x;
    _estimate.y() = y;
    _estimate.z() = t;
    setFixed(fixed);
  }

  // Access the vertex
  inline Eigen::Vector3d & position() { return _estimate; }

  inline const Eigen::Vector3d & position() const { return _estimate; }

  // Access the x-coordinate the pose
  inline double & x() { return _estimate.x(); }

  inline const double & x() const { return _estimate.x(); }

  // Access the y-coordinate the pose
  inline double & y() { return _estimate.y(); }

  inline const double & y() const { return _estimate.y(); }

  // Access the time part of the pose
  inline double & t() { return _estimate.z(); }

  inline const double & t() const { return _estimate.z(); }

  // Set the underlying estimate to zero.
  virtual void setToOriginImpl() override { _estimate.setZero(); }

  // Define the plus increment
  virtual void oplusImpl(const double * update) override
  {
    _estimate.x() += update[0];
    _estimate.y() += update[1];
    _estimate.z() += update[2];
  }

  // Read an estimate from an input stream.
  virtual bool read(std::istream & is) override
  {
    is >> _estimate.x() >> _estimate.y() >> _estimate.z();
    return true;
  }

  // Write the estimate to an output stream
  virtual bool write(std::ostream & os) const override
  {
    os << _estimate.x() << " " << _estimate.y() << " " << _estimate.z();
    return os.good();
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace steb_planner

#endif  // VERTEX_ST_H_
