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

#ifndef _EDGE_KINEMATICS_H
#define _EDGE_KINEMATICS_H

#include <steb_planner/g2o_types/base_teb_edges.h>
#include <steb_planner/g2o_types/penalties.h>
#include <steb_planner/g2o_types/vertex_st.h>
#include <steb_planner/steb_optimal/steb_config.h>

#include <cmath>

namespace steb_planner
{

// EdgeKinematicsCarlike: Edge defining the cost function for satisfying the non-holonomic
// kinematics of a carlike mobile robot.
class EdgeKinematicsCarlike : public BaseTebMultiEdge<2, double>
{
public:
  EdgeKinematicsCarlike()
  {
    this->setMeasurement(0.);
    this->resize(3);
  }

  // cost function
  void computeError()
  {
    STEB_ASSERT_MSG(steb_cfg_, "You must call setSTEBConfig on EdgeKinematicsCarlike()");
    const VertexST * conf1 = static_cast<const VertexST *>(_vertices[0]);
    const VertexST * conf2 = static_cast<const VertexST *>(_vertices[1]);
    const VertexST * conf3 = static_cast<const VertexST *>(_vertices[2]);

    const Eigen::Vector2d deltaS1{
      conf2->estimate().x() - conf1->estimate().x(), conf2->estimate().y() - conf1->estimate().y()};
    const Eigen::Vector2d deltaS2{
      conf3->estimate().x() - conf2->estimate().x(), conf3->estimate().y() - conf2->estimate().y()};

    const double theta1 = std::atan2(deltaS1.y(), deltaS1.x());
    const double theta2 = std::atan2(deltaS2.y(), deltaS2.x());

    // non holonomic constraint
    _error[0] =
      fabs((cos(theta1) + cos(theta2)) * deltaS1[1] - (sin(theta1) + sin(theta2)) * deltaS1[0]);
    //    _error[0] = g2o::normalize_theta(std::abs(theta1 - theta2));

    //    double cosValNew = deltaS1.dot(deltaS2) / (deltaS1.norm()*deltaS2.norm()); //角度cos值
    //    double angleNew = std::acos(cosValNew);     //弧度

    // limit minimum turning radius
    double angle_diff = g2o::normalize_theta(std::abs(theta1 - theta2));
    //    double angle_diff = g2o::normalize_theta( angleNew );

    if (angle_diff == 0) {
      _error[1] = 0;                                    // straight line motion
    } else if (steb_cfg_->trajectory.exact_arc_length)  // use exact computation of the radius
    {
      _error[1] = penaltyBoundFromBelow(
        fabs(deltaS1.norm() / (2 * sin(angle_diff / 2))),
        steb_cfg_->vehicle_param.min_turning_radius, 0.0);
    } else {
      _error[1] = penaltyBoundFromBelow(
        deltaS1.norm() / fabs(angle_diff), steb_cfg_->vehicle_param.min_turning_radius, 0.0);
      // This edge is not affected by the epsilon parameter, the user might add an exra margin to
      // the min_turning_radius parameter.
    }

    STEB_ASSERT_MSG(
      std::isfinite(_error[0]) && std::isfinite(_error[1]),
      "EdgeKinematicsCarlike::computeError() _error[0]=%f _error[1]=%f\n", _error[0], _error[1]);
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace steb_planner

#endif
