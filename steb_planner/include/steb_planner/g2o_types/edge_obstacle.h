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

#ifndef EDGE_OBSTACLE_H_
#define EDGE_OBSTACLE_H_

#include <steb_planner/g2o_types/base_teb_edges.h>
#include <steb_planner/g2o_types/penalties.h>
#include <steb_planner/g2o_types/vertex_st.h>
#include <steb_planner/steb_obstacle/dynamic_obstacles.h>
#include <steb_planner/steb_optimal/steb_config.h>
#include <steb_planner/steb_optimal/vehicle_footprint_model.h>

namespace steb_planner
{

// EdgeObstacle: Edge defining the cost function for keeping a minimum distance from obstacles.
class EdgeDynamicObstacleSpatio : public BaseTebUnaryEdge<2, const Obstacle *, VertexST>
{
public:
  EdgeDynamicObstacleSpatio() { _measurement = NULL; }

  void computeError()
  {
    STEB_ASSERT_MSG(
      steb_cfg_ && _measurement,
      "You must call setSTEBConfig() and setObstacle() on EdgeDynamicObstacleSpatio()");
    const VertexST * bandpt = static_cast<const VertexST *>(_vertices[0]);

    PoseSE2 current_pose(bandpt->x(), bandpt->y(), 0.0);
    double dist = steb_cfg_->ego_vehicle_model->calculateSpatioDistance(current_pose, _measurement);

    // Original obstacle cost.
    _error[0] = penaltyBoundFromBelow(
      dist, steb_cfg_->obstacles.min_obstacle_dist, steb_cfg_->optim.penalty_epsilon);

    if (
      steb_cfg_->optim.obstacle_cost_exponent != 1.0 &&
      steb_cfg_->obstacles.min_obstacle_dist > 0.0) {
      _error[0] = steb_cfg_->obstacles.min_obstacle_dist *
                  std::pow(
                    _error[0] / steb_cfg_->obstacles.min_obstacle_dist,
                    steb_cfg_->optim.obstacle_cost_exponent);
    }

    // Additional linear inflation cost
    _error[1] = penaltyBoundFromBelow(dist, steb_cfg_->obstacles.inflation_dist, 0.0);

    STEB_ASSERT_MSG(
      std::isfinite(_error[0]), "EdgeDynamicObstacleSpatio::computeError() _error[0]=%f\n",
      _error[0]);
  }

  void setParameters(const Obstacle * obstacle) { _measurement = obstacle; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// EdgeObstacleTemporal: Edge defining the cost function for keeping a minimum time distance from
// inflated obstacles.
class EdgeDynamicObstacleTemporal : public BaseTebUnaryEdge<2, const Obstacle *, VertexST>
{
public:
  EdgeDynamicObstacleTemporal() { _measurement = NULL; }

  void computeError()
  {
    STEB_ASSERT_MSG(steb_cfg_ && _measurement, "You must call setSTEBConfig()");
    const VertexST * bandpt = static_cast<const VertexST *>(_vertices[0]);

    double dist = std::fabs(bandpt->t() - _measurement->getEmergenceTime());

    _error[0] = penaltyBoundFromBelow(
      dist, steb_cfg_->obstacles.min_obstacle_dist, steb_cfg_->optim.penalty_epsilon);

    if (
      steb_cfg_->optim.obstacle_cost_exponent != 1.0 &&
      steb_cfg_->obstacles.min_obstacle_dist > 0.0) {
      _error[0] = steb_cfg_->obstacles.min_obstacle_dist *
                  std::pow(
                    _error[0] / steb_cfg_->obstacles.min_obstacle_dist,
                    steb_cfg_->optim.obstacle_cost_exponent);
    }

    // Additional linear inflation cost
    _error[1] = penaltyBoundFromBelow(dist, steb_cfg_->obstacles.inflation_dist, 0.0);

    STEB_ASSERT_MSG(
      std::isfinite(_error[0]) && std::isfinite(_error[1]),
      "EdgeDynamicObstacleTemporal::computeError() _error[0]=%f, _error[1]=%f\n", _error[0],
      _error[1]);
  }

  void setParameters(const Obstacle * obstacle) { _measurement = obstacle; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// EdgeInflatedObstacle: Edge defining the cost function for keeping a minimum ST distance from
// obstacles.
class EdgeDynamicObstacleST : public BaseTebMultiEdge<2, const Obstacle *>
{
public:
  EdgeDynamicObstacleST()
  {
    _measurement = NULL;
    this->resize(2);
  }

  void computeError()
  {
    STEB_ASSERT_MSG(
      steb_cfg_ && _measurement,
      "You must call setSTEBConfig() and setParameters() on EdgeDynamicObstacleST()");
    const VertexST * conf1 = static_cast<const VertexST *>(_vertices[0]);
    const VertexST * conf2 = static_cast<const VertexST *>(_vertices[1]);

    const Eigen::Vector2d diff1 = {
      conf2->estimate().x() - conf1->estimate().x(), conf2->estimate().y() - conf1->estimate().y()};

    const double theta1 = std::atan2(diff1.y(), diff1.x());

    PoseSE2 current_pose{conf1->x(), conf1->y(), theta1};
    double dist = steb_cfg_->ego_vehicle_model->calculateSpatioTemporalDistance(
      current_pose, conf1->estimate().z(), _measurement);

    _error[0] = penaltyBoundFromBelow(
      dist, steb_cfg_->obstacles.min_obstacle_dist, steb_cfg_->optim.penalty_epsilon);

    if (
      steb_cfg_->optim.obstacle_cost_exponent != 1.0 &&
      steb_cfg_->obstacles.min_obstacle_dist > 0.0) {
      _error[0] = steb_cfg_->obstacles.min_obstacle_dist *
                  std::pow(
                    _error[0] / steb_cfg_->obstacles.min_obstacle_dist,
                    steb_cfg_->optim.obstacle_cost_exponent);
    }

    // Additional linear inflation cost
    //    _error[1] = penaltyBoundFromBelow(dist, steb_cfg_->obstacles.inflation_dist, 0.0);
    _error[1] = 0.0;

    STEB_ASSERT_MSG(
      std::isfinite(_error[0]) && std::isfinite(_error[1]),
      "EdgeSTEBDynamicObstacleST::computeError() _error[0]=%f, _error[1]=%f\n", _error[0],
      _error[1]);
  }

  void setParameters(const Obstacle * obstacle) { _measurement = obstacle; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace steb_planner

#endif
