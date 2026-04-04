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


#ifndef EDGE_ACCELERATION_H_
#define EDGE_ACCELERATION_H_

#include <geometry_msgs/msg/twist.hpp>

#include <steb_planner/g2o_types/vertex_st.h>
#include <steb_planner/g2o_types/penalties.h>
#include <steb_planner/g2o_types/base_teb_edges.h>

#include <steb_planner/steb_optimal/steb_config.h>


namespace steb_planner
{

// EdgeAcceleration: Edge defining the cost function for limiting the translational and rotational acceleration.
class EdgeAcceleration : public BaseTebMultiEdge<4, double>
{
public:

  EdgeAcceleration()
  {
    this->setMeasurement(0.);
    this->resize(4);
  }

  void computeError() override
  {
    STEB_ASSERT_MSG(steb_cfg_, "You must call setTebConfig on EdgeAcceleration()");
    const auto* pose1 = dynamic_cast<const VertexST*>(_vertices[0]);
    const auto* pose2 = dynamic_cast<const VertexST*>(_vertices[1]);
    const auto* pose3 = dynamic_cast<const VertexST*>(_vertices[2]);
    const auto* pose4 = dynamic_cast<const VertexST*>(_vertices[3]);

    // VELOCITY & ACCELERATION
    const Eigen::Vector2d diff1 = {pose2->estimate().x() - pose1->estimate().x(),
                                   pose2->estimate().y() - pose1->estimate().y()};
    const Eigen::Vector2d diff2 = {pose3->estimate().x() - pose2->estimate().x(),
                                   pose3->estimate().y() - pose2->estimate().y()};
    const Eigen::Vector2d diff3 = {pose4->estimate().x() - pose3->estimate().x(),
                                   pose4->estimate().y() - pose3->estimate().y()};


    double dist1 = diff1.norm();
    double dist2 = diff2.norm();
    double dist3 = diff3.norm();

    const double theta1 = std::atan2(diff1.y(), diff1.x());
    const double theta2 = std::atan2(diff2.y(), diff2.x());
    const double theta3 = std::atan2(diff3.y(), diff3.x());

    const double angle_diff1 = g2o::normalize_theta(theta2 - theta1);
    const double angle_diff2 = g2o::normalize_theta(theta3 - theta2);

    double t1 = pose2->t() - pose1->t() + 0.0001;
    double t2 = pose3->t() - pose2->t() + 0.0001;
    double t3 = pose4->t() - pose3->t() + 0.0001;

    // velocity
    double velocity1 = dist1 / t1;
    double velocity2 = dist2 / t2;
    double velocity3 = dist3 / t3;

    double angle_velocity1 = angle_diff1 / t1;
    double angle_velocity2 = angle_diff2 / t2;

    // acceleration
    double accel1 = (velocity2 - velocity1) * 2.0 / (t1 + t2);
    double accel2 = (velocity3 - velocity2) * 2.0 / (t2 + t3);

    double angle_accel1 = std::fabs( (angle_velocity2 -angle_velocity1) * 2.0 / (t1 + t2) );

    // jerk
    double jerk1 = std::fabs( accel2 - accel1 );
    // std::fabs(accel2 - accel1);
    // std::fabs(accel2 - accel1) * 3 / (t1 + t2 + t3);



//    if (steb_cfg_->trajectory.exact_arc_length) // use exact arc length instead of Euclidean approximation
//    {
//      if (angle_diff1 != 0)
//      {
//        const double radius =  dist1/(2*sin(angle_diff1/2));
//        dist1 = fabs( angle_diff1 * radius ); // actual arg length!
//      }
//      if (angle_diff2 != 0)
//      {
//        const double radius =  dist2/(2*sin(angle_diff2/2));
//        dist2 = fabs( angle_diff2 * radius ); // actual arg length!
//      }
//    }
//
//    double vel1 = dist1 / dt1->dt();
//    double vel2 = dist2 / dt2->dt();
//
//
//    // consider directions
//    //     vel1 *= g2o::sign(diff1[0]*cos(pose1->theta()) + diff1[1]*sin(pose1->theta()));
//    //     vel2 *= g2o::sign(diff2[0]*cos(pose2->theta()) + diff2[1]*sin(pose2->theta()));
//    vel1 *= fast_sigmoid( 100*(diff1.x()*cos(pose1->theta()) + diff1.y()*sin(pose1->theta())) );
//    vel2 *= fast_sigmoid( 100*(diff2.x()*cos(pose2->theta()) + diff2.y()*sin(pose2->theta())) );
//
//    const double acc_lin  = (vel2 - vel1)*2 / ( dt1->dt() + dt2->dt() );




    _error[0] = penaltyBoundToInterval((accel1 + accel2 ) / 2.0, steb_cfg_->vehicle_param.max_longitudinal_acc, steb_cfg_->optim.penalty_epsilon);

    // ANGULAR ACCELERATION
//    const double omega1 = angle_diff1 / dt1->dt();
//    const double omega2 = angle_diff2 / dt2->dt();
//    const double acc_rot  = (omega2 - omega1)*2 / ( dt1->dt() + dt2->dt() );

    _error[1] = penaltyBoundToInterval(angle_accel1, steb_cfg_->vehicle_param.max_theta_acc, steb_cfg_->optim.penalty_epsilon);

    _error[2] = jerk1;

    _error[3] = std::fabs(angle_accel1);


    STEB_ASSERT_MSG(std::isfinite(_error[0]), "EdgeAcceleration::computeError() translational: _error[0]=%f\n",_error[0]);
    STEB_ASSERT_MSG(std::isfinite(_error[1]), "EdgeAcceleration::computeError() rotational: _error[1]=%f\n",_error[1]);
  }



#ifdef USE_ANALYTIC_JACOBI
#if 0
  /*
   * @brief Jacobi matrix of the cost function specified in computeError().
   */
  void linearizeOplus()
  {
    ROS_ASSERT_MSG(steb_cfg_, "You must call setTebConfig on EdgeAcceleration()");
    const VertexPointXY* conf1 = static_cast<const VertexPointXY*>(_vertices[0]);
    const VertexPointXY* conf2 = static_cast<const VertexPointXY*>(_vertices[1]);
    const VertexPointXY* conf3 = static_cast<const VertexPointXY*>(_vertices[2]);
    const VertexTimeDiff* deltaT1 = static_cast<const VertexTimeDiff*>(_vertices[3]);
    const VertexTimeDiff* deltaT2 = static_cast<const VertexTimeDiff*>(_vertices[4]);
    const VertexOrientation* angle1 = static_cast<const VertexOrientation*>(_vertices[5]);
    const VertexOrientation* angle2 = static_cast<const VertexOrientation*>(_vertices[6]);
    const VertexOrientation* angle3 = static_cast<const VertexOrientation*>(_vertices[7]);

    Eigen::Vector2d deltaS1 = conf2->estimate() - conf1->estimate();
    Eigen::Vector2d deltaS2 = conf3->estimate() - conf2->estimate();
    double dist1 = deltaS1.norm();
    double dist2 = deltaS2.norm();

    double sum_time = deltaT1->estimate() + deltaT2->estimate();
    double sum_time_inv = 1 / sum_time;
    double dt1_inv = 1/deltaT1->estimate();
    double dt2_inv = 1/deltaT2->estimate();
    double aux0 = 2/sum_time_inv;
    double aux1 = dist1 * deltaT1->estimate();
    double aux2 = dist2 * deltaT2->estimate();

    double vel1 = dist1 * dt1_inv;
    double vel2 = dist2 * dt2_inv;
    double omega1 = g2o::normalize_theta( angle2->estimate() - angle1->estimate() ) * dt1_inv;
    double omega2 = g2o::normalize_theta( angle3->estimate() - angle2->estimate() ) * dt2_inv;
    double acc = (vel2 - vel1) * aux0;
    double omegadot = (omega2 - omega1) * aux0;
    double aux3 = -acc/2;
    double aux4 = -omegadot/2;

    double dev_border_acc = penaltyBoundToIntervalDerivative(acc, tebConfig.robot_acceleration_max_trans,optimizationConfig.optimization_boundaries_epsilon,optimizationConfig.optimization_boundaries_scale,optimizationConfig.optimization_boundaries_order);
    double dev_border_omegadot = penaltyBoundToIntervalDerivative(omegadot, tebConfig.robot_acceleration_max_rot,optimizationConfig.optimization_boundaries_epsilon,optimizationConfig.optimization_boundaries_scale,optimizationConfig.optimization_boundaries_order);

    _jacobianOplus[0].resize(2,2); // conf1
    _jacobianOplus[1].resize(2,2); // conf2
    _jacobianOplus[2].resize(2,2); // conf3
    _jacobianOplus[3].resize(2,1); // deltaT1
    _jacobianOplus[4].resize(2,1); // deltaT2
    _jacobianOplus[5].resize(2,1); // angle1
    _jacobianOplus[6].resize(2,1); // angle2
    _jacobianOplus[7].resize(2,1); // angle3

    if (aux1==0) aux1=1e-20;
    if (aux2==0) aux2=1e-20;

    if (dev_border_acc!=0)
    {
      // TODO: double aux = aux0 * dev_border_acc;
      // double aux123 = aux / aux1;
      _jacobianOplus[0](0,0) = aux0 * deltaS1[0] / aux1 * dev_border_acc; // acc x1
      _jacobianOplus[0](0,1) = aux0 * deltaS1[1] / aux1 * dev_border_acc; // acc y1
      _jacobianOplus[1](0,0) = -aux0 * ( deltaS1[0] / aux1 + deltaS2[0] / aux2 ) * dev_border_acc; // acc x2
      _jacobianOplus[1](0,1) = -aux0 * ( deltaS1[1] / aux1 + deltaS2[1] / aux2 ) * dev_border_acc; // acc y2
      _jacobianOplus[2](0,0) = aux0 * deltaS2[0] / aux2 * dev_border_acc; // acc x3
      _jacobianOplus[2](0,1) = aux0 * deltaS2[1] / aux2 * dev_border_acc; // acc y3
      _jacobianOplus[2](0,0) = 0;
      _jacobianOplus[2](0,1) = 0;
      _jacobianOplus[3](0,0) = aux0 * (aux3 + vel1 * dt1_inv) * dev_border_acc; // acc deltaT1
      _jacobianOplus[4](0,0) = aux0 * (aux3 - vel2 * dt2_inv) * dev_border_acc; // acc deltaT2
    }
    else
    {
      _jacobianOplus[0](0,0) = 0; // acc x1
      _jacobianOplus[0](0,1) = 0; // acc y1
      _jacobianOplus[1](0,0) = 0; // acc x2
      _jacobianOplus[1](0,1) = 0; // acc y2
      _jacobianOplus[2](0,0) = 0; // acc x3
      _jacobianOplus[2](0,1) = 0; // acc y3
      _jacobianOplus[3](0,0) = 0; // acc deltaT1
      _jacobianOplus[4](0,0) = 0; // acc deltaT2
    }

    if (dev_border_omegadot!=0)
    {
      _jacobianOplus[3](1,0) = aux0 * ( aux4 + omega1 * dt1_inv ) * dev_border_omegadot; // omegadot deltaT1
      _jacobianOplus[4](1,0) = aux0 * ( aux4 - omega2 * dt2_inv ) * dev_border_omegadot; // omegadot deltaT2
      _jacobianOplus[5](1,0) = aux0 * dt1_inv * dev_border_omegadot; // omegadot angle1
      _jacobianOplus[6](1,0) = -aux0 * ( dt1_inv + dt2_inv ) * dev_border_omegadot; // omegadot angle2
      _jacobianOplus[7](1,0) = aux0 * dt2_inv * dev_border_omegadot; // omegadot angle3
    }
    else
    {
      _jacobianOplus[3](1,0) = 0; // omegadot deltaT1
      _jacobianOplus[4](1,0) = 0; // omegadot deltaT2
      _jacobianOplus[5](1,0) = 0; // omegadot angle1
      _jacobianOplus[6](1,0) = 0; // omegadot angle2
      _jacobianOplus[7](1,0) = 0; // omegadot angle3
    }

    _jacobianOplus[0](1,0) = 0; // omegadot x1
    _jacobianOplus[0](1,1) = 0; // omegadot y1
    _jacobianOplus[1](1,0) = 0; // omegadot x2
    _jacobianOplus[1](1,1) = 0; // omegadot y2
    _jacobianOplus[2](1,0) = 0; // omegadot x3
    _jacobianOplus[2](1,1) = 0; // omegadot y3
    _jacobianOplus[5](0,0) = 0; // acc angle1
    _jacobianOplus[6](0,0) = 0; // acc angle2
    _jacobianOplus[7](0,0) = 0; // acc angle3
    }
#endif
#endif


public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};


    
} // end namespace

#endif /* EDGE_ACCELERATION_H_ */
