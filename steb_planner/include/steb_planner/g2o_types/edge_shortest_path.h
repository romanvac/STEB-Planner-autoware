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

#ifndef EDGE_SHORTEST_PATH_H_
#define EDGE_SHORTEST_PATH_H_

#include <Eigen/Core>

#include <float.h>
#include <steb_planner/g2o_types/base_teb_edges.h>
#include <steb_planner/g2o_types/vertex_st.h>

namespace steb_planner
{

// EdgeShortestPath: Edge defining the cost function for minimizing the distance between two
// consectuive poses.
class EdgeShortestPath : public BaseTebBinaryEdge<1, double, VertexST, VertexST>
{
public:
  EdgeShortestPath() { this->setMeasurement(0.); }

  // cost function
  void computeError()
  {
    STEB_ASSERT_MSG(steb_cfg_, "You must call setTebConfig on EdgeShortestPath()");
    const VertexST * pose1 = static_cast<const VertexST *>(_vertices[0]);
    const VertexST * pose2 = static_cast<const VertexST *>(_vertices[1]);

    //    _error[0] = (pose2->position() - pose1->position()).norm();

    Eigen::Vector2d dist{
      pose2->position().x() - pose1->position().x(), pose2->position().y() - pose1->position().y()};
    _error[0] = dist.norm();

    STEB_ASSERT_MSG(
      std::isfinite(_error[0]), "EdgeShortestPath::computeError() _error[0]=%f\n", _error[0]);
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace steb_planner

#endif /* EDGE_SHORTEST_PATH_H_ */
