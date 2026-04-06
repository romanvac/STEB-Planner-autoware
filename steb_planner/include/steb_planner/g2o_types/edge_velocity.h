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

#ifndef EDGE_VELOCITY_H
#define EDGE_VELOCITY_H

#include <steb_planner/g2o_types/base_teb_edges.h>
#include <steb_planner/g2o_types/penalties.h>
#include <steb_planner/g2o_types/vertex_st.h>
#include <steb_planner/steb_optimal/steb_config.h>

#include <iostream>

namespace steb_planner
{

// EdgeVelocity: Edge defining the cost function for limiting the translational and rotational
// velocity.
class EdgeVelocity : public BaseTebMultiEdge<3, double>
{
public:
  EdgeVelocity()
  {
    this->setMeasurement(0.);
    this->resize(3);
  }

  // cost function
  void computeError()
  {
    STEB_ASSERT_MSG(steb_cfg_, "You must call setSTEBConfig on EdgeVelocity()");
    const VertexST * conf1 = static_cast<const VertexST *>(_vertices[0]);
    const VertexST * conf2 = static_cast<const VertexST *>(_vertices[1]);
    const VertexST * conf3 = static_cast<const VertexST *>(_vertices[2]);
    //    const VertexTimeDiff* deltaT = static_cast<const VertexTimeDiff*>(_vertices[2]);
    // TODO: @HS,deltaT1 存在=inf的情况，导致error=nan，进一步让程序崩溃
    // conf2->t() 会 =inf
    const double deltaT1 = conf2->t() - conf1->t() + 0.001;

    const Eigen::Vector2d deltaS1{
      conf2->estimate().x() - conf1->estimate().x(), conf2->estimate().y() - conf1->estimate().y()};
    const Eigen::Vector2d deltaS2{
      conf3->estimate().x() - conf2->estimate().x(), conf3->estimate().y() - conf2->estimate().y()};

    double dist = deltaS1.norm();

    const double theta1 = std::atan2(deltaS1.y(), deltaS1.x());
    const double theta2 = std::atan2(deltaS2.y(), deltaS2.x());
    const double angle_diff = g2o::normalize_theta(std::abs(theta2 - theta1));

    if (steb_cfg_->trajectory.exact_arc_length && angle_diff != 0) {
      double radius = dist / (2 * sin(angle_diff / 2));
      dist = fabs(angle_diff * radius);  // actual arg length!
    }
    double sign = std::cos(theta2 - theta1) > 0.0 ? 1.0 : -1.0;
    double vel = sign * dist / deltaT1;

    // vel *= g2o::sign(deltaS[0]*cos(conf1->theta()) + deltaS[1]*sin(conf1->theta())); // consider
    // direction vel *= fast_sigmoid( 100 * (deltaS.x()*cos(conf1->theta()) +
    // deltaS.y()*sin(conf1->theta())) ); // consider direction

    const double angle_vel = angle_diff / deltaT1;

    //    _error[0] = penaltyBoundToInterval(vel, -steb_cfg_->robot.max_vel_x_backwards,
    //    steb_cfg_->robot.max_vel_x, steb_cfg_->optim.penalty_epsilon);
    _error[0] = penaltyBoundToInterval(
      vel, 0.0, steb_cfg_->vehicle_param.max_vel_x, steb_cfg_->optim.penalty_epsilon);
    _error[1] = penaltyBoundToInterval(
      angle_vel, steb_cfg_->vehicle_param.max_vel_theta, steb_cfg_->optim.penalty_epsilon);
    _error[2] = std::fabs(angle_vel);

    STEB_ASSERT_MSG(
      std::isfinite(_error[0]), "EdgeVelocity::computeError() _error[0]=%f _error[1]=%f\n",
      _error[0], _error[1]);
  }

#ifdef USE_ANALYTIC_JACOBI
#if 0  // TODO the hardcoded jacobian does not include the changing direction (just the absolute
       // value)
       //  Change accordingly...

  /**
   * @brief Jacobi matrix of the cost function specified in computeError().
   */
  void linearizeOplus()
  {
    ROS_ASSERT_MSG(steb_cfg_, "You must call setSTEBConfig on EdgeVelocity()");
    const VertexPose* conf1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* conf2 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexTimeDiff* deltaT = static_cast<const VertexTimeDiff*>(_vertices[2]);

    Eigen::Vector2d deltaS = conf2->position() - conf1->position();
    double dist = deltaS.norm();
    double aux1 = dist*deltaT->estimate();
    double aux2 = 1/deltaT->estimate();

    double vel = dist * aux2;
    double omega = g2o::normalize_theta(conf2->theta() - conf1->theta()) * aux2;

    double dev_border_vel = penaltyBoundToIntervalDerivative(vel, -steb_cfg_->robot.max_vel_x_backwards, steb_cfg_->robot.max_vel_x,steb_cfg_->optim.penalty_epsilon);
    double dev_border_omega = penaltyBoundToIntervalDerivative(omega, steb_cfg_->robot.max_vel_theta,steb_cfg_->optim.penalty_epsilon);

    _jacobianOplus[0].resize(2,3); // conf1
    _jacobianOplus[1].resize(2,3); // conf2
    _jacobianOplus[2].resize(2,1); // deltaT

//  if (aux1==0) aux1=1e-6;
//  if (aux2==0) aux2=1e-6;

    if (dev_border_vel!=0)
    {
      double aux3 = dev_border_vel / aux1;
      _jacobianOplus[0](0,0) = -deltaS[0] * aux3; // vel x1
      _jacobianOplus[0](0,1) = -deltaS[1] * aux3; // vel y1
      _jacobianOplus[1](0,0) = deltaS[0] * aux3; // vel x2
      _jacobianOplus[1](0,1) = deltaS[1] * aux3; // vel y2
      _jacobianOplus[2](0,0) = -vel * aux2 * dev_border_vel; // vel deltaT
    }
    else
    {
      _jacobianOplus[0](0,0) = 0; // vel x1
      _jacobianOplus[0](0,1) = 0; // vel y1
      _jacobianOplus[1](0,0) = 0; // vel x2
      _jacobianOplus[1](0,1) = 0; // vel y2
      _jacobianOplus[2](0,0) = 0; // vel deltaT
    }

    if (dev_border_omega!=0)
    {
      double aux4 = aux2 * dev_border_omega;
      _jacobianOplus[2](1,0) = -omega * aux4; // omega deltaT
      _jacobianOplus[0](1,2) = -aux4; // omega angle1
      _jacobianOplus[1](1,2) = aux4; // omega angle2
    }
    else
    {
      _jacobianOplus[2](1,0) = 0; // omega deltaT
      _jacobianOplus[0](1,2) = 0; // omega angle1
      _jacobianOplus[1](1,2) = 0; // omega angle2
    }

    _jacobianOplus[0](1,0) = 0; // omega x1
    _jacobianOplus[0](1,1) = 0; // omega y1
    _jacobianOplus[1](1,0) = 0; // omega x2
    _jacobianOplus[1](1,1) = 0; // omega y2
    _jacobianOplus[0](0,2) = 0; // vel angle1
    _jacobianOplus[1](0,2) = 0; // vel angle2
  }
#endif
#endif

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// EdgeINITVelocity
class EdgeInitVelocity
: public BaseTebBinaryEdge<2, const nav_msgs::msg::Odometry *, VertexST, VertexST>
{
public:
  EdgeInitVelocity()
  {
    this->setMeasurement(NULL);
    this->resize(2);
  }

  void computeError()
  {
    STEB_ASSERT_MSG(steb_cfg_, "You must call setSTEBConfig on EdgeVelocity()");
    const VertexST * conf1 = static_cast<const VertexST *>(_vertices[0]);
    const VertexST * conf2 = static_cast<const VertexST *>(_vertices[1]);

    const double deltaT1 = conf2->t() - conf1->t() + 0.0001;

    const Eigen::Vector2d deltaS1{
      conf2->estimate().x() - conf1->estimate().x(), conf2->estimate().y() - conf1->estimate().y()};

    double dist = deltaS1.norm();
    const double theta1 = std::atan2(deltaS1.y(), deltaS1.x());
    //    const double angle_diff = g2o::normalize_theta(std::abs(conf2->yaw() - conf1->yaw()));
    //
    //    if (steb_cfg_->trajectory.exact_arc_length && angle_diff != 0)
    //    {
    //      double radius =  dist/(2*sin(angle_diff/2));
    //      dist = fabs( angle_diff * radius ); // actual arg length!
    //    }
    double vel = dist / deltaT1;

    //     vel *= g2o::sign(deltaS[0]*cos(conf1->theta()) + deltaS[1]*sin(conf1->theta())); //
    //     consider direction
    //    vel *= fast_sigmoid( 100 * (deltaS.x()*cos(conf1->theta()) +
    //    deltaS.y()*sin(conf1->theta())) ); // consider direction

    _error[0] = std::fabs(vel - _measurement->twist.twist.linear.x);
    _error[1] = std::fabs(theta1) * 57.0;
    //    _error[1] = penaltyBoundToInterval(dist, 0.5, 0.0);

    STEB_ASSERT_MSG(
      std::isfinite(_error[0]), "EdgeVelocity::computeError() _error[0]=%f \n", _error[0]);
  }

  void setParameters(const nav_msgs::msg::Odometry * odometry) { _measurement = odometry; }

#ifdef USE_ANALYTIC_JACOBI
#if 0  // TODO the hardcoded jacobian does not include the changing direction (just the absolute
       // value)
       //  Change accordingly...

  /**
   * @brief Jacobi matrix of the cost function specified in computeError().
   */
  void linearizeOplus()
  {
    ROS_ASSERT_MSG(steb_cfg_, "You must call setSTEBConfig on EdgeVelocity()");
    const VertexPose* conf1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* conf2 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexTimeDiff* deltaT = static_cast<const VertexTimeDiff*>(_vertices[2]);

    Eigen::Vector2d deltaS = conf2->position() - conf1->position();
    double dist = deltaS.norm();
    double aux1 = dist*deltaT->estimate();
    double aux2 = 1/deltaT->estimate();

    double vel = dist * aux2;
    double omega = g2o::normalize_theta(conf2->theta() - conf1->theta()) * aux2;

    double dev_border_vel = penaltyBoundToIntervalDerivative(vel, -steb_cfg_->robot.max_vel_x_backwards, steb_cfg_->robot.max_vel_x,steb_cfg_->optim.penalty_epsilon);
    double dev_border_omega = penaltyBoundToIntervalDerivative(omega, steb_cfg_->robot.max_vel_theta,steb_cfg_->optim.penalty_epsilon);

    _jacobianOplus[0].resize(2,3); // conf1
    _jacobianOplus[1].resize(2,3); // conf2
    _jacobianOplus[2].resize(2,1); // deltaT

//  if (aux1==0) aux1=1e-6;
//  if (aux2==0) aux2=1e-6;

    if (dev_border_vel!=0)
    {
      double aux3 = dev_border_vel / aux1;
      _jacobianOplus[0](0,0) = -deltaS[0] * aux3; // vel x1
      _jacobianOplus[0](0,1) = -deltaS[1] * aux3; // vel y1
      _jacobianOplus[1](0,0) = deltaS[0] * aux3; // vel x2
      _jacobianOplus[1](0,1) = deltaS[1] * aux3; // vel y2
      _jacobianOplus[2](0,0) = -vel * aux2 * dev_border_vel; // vel deltaT
    }
    else
    {
      _jacobianOplus[0](0,0) = 0; // vel x1
      _jacobianOplus[0](0,1) = 0; // vel y1
      _jacobianOplus[1](0,0) = 0; // vel x2
      _jacobianOplus[1](0,1) = 0; // vel y2
      _jacobianOplus[2](0,0) = 0; // vel deltaT
    }

    if (dev_border_omega!=0)
    {
      double aux4 = aux2 * dev_border_omega;
      _jacobianOplus[2](1,0) = -omega * aux4; // omega deltaT
      _jacobianOplus[0](1,2) = -aux4; // omega angle1
      _jacobianOplus[1](1,2) = aux4; // omega angle2
    }
    else
    {
      _jacobianOplus[2](1,0) = 0; // omega deltaT
      _jacobianOplus[0](1,2) = 0; // omega angle1
      _jacobianOplus[1](1,2) = 0; // omega angle2
    }

    _jacobianOplus[0](1,0) = 0; // omega x1
    _jacobianOplus[0](1,1) = 0; // omega y1
    _jacobianOplus[1](1,0) = 0; // omega x2
    _jacobianOplus[1](1,1) = 0; // omega y2
    _jacobianOplus[0](0,2) = 0; // vel angle1
    _jacobianOplus[1](0,2) = 0; // vel angle2
  }
#endif
#endif

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace steb_planner

#endif
