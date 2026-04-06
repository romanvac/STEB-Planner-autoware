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

#ifndef EDGE_TIMEOPTIMAL_H_
#define EDGE_TIMEOPTIMAL_H_

#include <float.h>

//#include <base_local_planner/BaseLocalPlannerConfig.h>

#include <Eigen/Core>

#include <steb_planner/g2o_types/base_teb_edges.h>
#include <steb_planner/g2o_types/penalties.h>
#include <steb_planner/g2o_types/vertex_st.h>
#include <steb_planner/steb_optimal/steb_config.h>

namespace steb_planner
{

// EdgeTimeOptimal: Edge defining the cost function for minimizing transition time of the
// trajectory.
class EdgeTimeOptimal : public BaseTebBinaryEdge<2, double, VertexST, VertexST>
{
public:
  EdgeTimeOptimal() { this->setMeasurement(0.); }

  // cost function
  void computeError()
  {
    STEB_ASSERT_MSG(steb_cfg_, "You must call setSTEBConfig on EdgeTimeOptimal()");
    const VertexST * pose1 = static_cast<const VertexST *>(_vertices[0]);
    const VertexST * pose2 = static_cast<const VertexST *>(_vertices[1]);

    _error[0] = std::fabs(pose2->t() - pose1->t());
    //    _error[0] = 0;
    //    std::cout << pose2->t() << ", " << pose1->t() <<std::endl;

    // Additional cost
    _error[1] = penaltyBoundFromBelow(pose2->t() - pose1->t(), 0.0, 0.0);

    STEB_ASSERT_MSG(
      std::isfinite(_error[0]), "EdgeTimeOptimal::computeError() _error[0]=%f\n", _error[0]);
    STEB_ASSERT_MSG(
      std::isfinite(_error[1]), "EdgeTimeOptimal::computeError() _error[1]=%f\n", _error[1]);
  }

#ifdef USE_ANALYTIC_JACOBI
  // Jacobi matrix of the cost function specified in computeError().
  void linearizeOplus()
  {
    STEB_ASSERT_MSG(steb_cfg_, "You must call setSTEBConfig on EdgeTimeOptimal()");
    _jacobianOplusXi(0, 0) = 1;
  }
#endif

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class EdgeTime : public BaseTebMultiEdge<2, double>
{
public:
  EdgeTime()
  {
    this->setMeasurement(0.);
    this->resize(3);
  }

  // cost function
  void computeError()
  {
    STEB_ASSERT_MSG(steb_cfg_, "You must call setSTEBConfig on EdgeTimeOptimal()");
    const VertexST * pose1 = static_cast<const VertexST *>(_vertices[0]);
    const VertexST * pose2 = static_cast<const VertexST *>(_vertices[1]);
    const VertexST * pose3 = static_cast<const VertexST *>(_vertices[2]);

    const double delta_t_1 = pose2->t() - pose1->t();
    const double delta_t_2 = pose3->t() - pose2->t();

    _error[0] = std::fabs(delta_t_2 - delta_t_1);
    //    std::cout << pose2->t() << ", " << pose1->t() <<std::endl;

    // Additional cost
    _error[1] = penaltyBoundFromBelow(pose2->t() - pose1->t(), 0.0, 0.0);

    STEB_ASSERT_MSG(
      std::isfinite(_error[0]), "EdgeTimeOptimal::computeError() _error[0]=%f\n", _error[0]);
    STEB_ASSERT_MSG(
      std::isfinite(_error[1]), "EdgeTimeOptimal::computeError() _error[1]=%f\n", _error[1]);
  }

#ifdef USE_ANALYTIC_JACOBI
#endif

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace steb_planner

#endif /* EDGE_TIMEOPTIMAL_H_ */
