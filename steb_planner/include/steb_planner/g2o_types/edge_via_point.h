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

#ifndef EDGE_VIA_POINT_H_
#define EDGE_VIA_POINT_H_


#include <steb_planner/g2o_types/vertex_st.h>
#include <steb_planner/g2o_types/base_teb_edges.h>
#include <steb_planner/g2o_types/penalties.h>

#include <steb_planner/steb_obstacle/via_area.h>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


namespace steb_planner
{



// EdgeGoalPoint: Edge defining the cost function for constraining the goal point
class EdgeGoalPoint : public BaseTebBinaryEdge<3, const Eigen::Vector4d*, VertexST, VertexST>
{
public:

  EdgeGoalPoint()
  {
    _measurement = NULL;
  }

  void computeError()
  {
    STEB_ASSERT_MSG(steb_cfg_ && _measurement, "You must call setSTEBConfig(), setGoalPoint() on EdgeGoalPoint()");
    const VertexST* conf1 = static_cast<const VertexST*>(_vertices[0]);
    const VertexST* conf2 = static_cast<const VertexST*>(_vertices[1]);

    const Eigen::Vector2d deltaS{conf2->estimate().x() - conf1->estimate().x() ,
                                 conf2->estimate().y() - conf1->estimate().y() };
    const double theta = std::atan2(deltaS.y(), deltaS.x());

    const Eigen::Vector2d deltaS1{conf2->estimate().x() - _measurement->x(),
                                  conf2->estimate().y() - _measurement->y()};
    double dist = deltaS1.norm();

    _error[0] = std::fabs(dist);

    _error[1] = std::fabs(g2o::normalize_theta(theta - _measurement->z())) * 57.0;

    _error[2] = std::fabs(conf2->t() - _measurement->w());


    STEB_ASSERT_MSG(std::isfinite(_error[0]), "EdgeGoalPoint::computeError() _error[0]=%f\n",_error[0]);
    STEB_ASSERT_MSG(std::isfinite(_error[1]), "EdgeGoalPoint::computeError() _error[1]=%f\n",_error[1]);
  }



  // Set all parameters at once: goal_point 2D position vector containing the position of the goal point
  void setParameters(const Eigen::Vector4d* goal_point)
  {
    _measurement = goal_point;
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};



// EdgeViaPoint: Edge defining the cost function for pushing a configuration towards a via point
class EdgeViaPoint : public BaseTebBinaryEdge<4, const ViaAreaContainer*, VertexST, VertexST>
{
public:

  EdgeViaPoint()
  {
    _measurement = NULL;
  }

  void computeError()
  {
    STEB_ASSERT_MSG(steb_cfg_ && _measurement, "You must call setSTEBConfig(), setParameters() on EdgeViaPoint()");
    const VertexST* bandpt1 = static_cast<const VertexST*>(_vertices[0]);
    const VertexST* bandpt2 = static_cast<const VertexST*>(_vertices[1]);

//    Eigen::Vector2d dist{bandpt1->position().x() - _measurement->getViaPoint().x(),
//                         bandpt1->position().y() - _measurement->getViaPoint().y()};
//    _error[0] = dist.norm();

    const Eigen::Vector2d deltaS{bandpt2->estimate().x() - bandpt1->estimate().x(),
                                 bandpt2->estimate().y() - bandpt1->estimate().y()};
    const double theta = std::atan2(deltaS.y(), deltaS.x());

    // 计算新pose在路由点pose Y方向的投影
    // TODO:还没搞完
//    const double angle_diff = std::abs(g2o::normalize_theta(_measurement->at(_nearest_index.front())->getViaPoint().z() - theta));
//    constexpr double vehicle_length = 5.0;
//    constexpr double vehicle_width = 2.0;
//    double offset = vehicle_length * std::sin(angle_diff) + std::cos(angle_diff) * vehicle_width * 0.5;
//    UNUSED(offset);

    tf2::Transform tf_map_to_via_point;
    tf_map_to_via_point.setOrigin(tf2::Vector3(_measurement->at(_nearest_index.front())->getViaPoint().x(),
                                               _measurement->at(_nearest_index.front())->getViaPoint().y(), 0.0));
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, _measurement->at(_nearest_index.front())->getViaPoint().z());
    tf_map_to_via_point.setRotation(quaternion);

    tf2::Transform tf_map_to_new_pose;
    tf_map_to_new_pose.setOrigin(tf2::Vector3(bandpt1->position().x(), bandpt1->position().y(), 0.0));
    tf2::Quaternion quaternion_2;
    quaternion_2.setRPY(0.0, 0.0, theta);
    tf_map_to_new_pose.setRotation (quaternion_2);

    tf2::Transform tf_via_point_to_new_pose = tf_map_to_via_point.inverse() * tf_map_to_new_pose;

    // 让轨迹靠近全局路径
    _error[0] = std::fabs(tf_via_point_to_new_pose.getOrigin().y()) ;
//    + std::fabs(_measurement->at(_nearest_index.front())->getViaTime() - bandpt1->t())
//    _error[5] = std::fabs(_measurement->at(_nearest_index.front())->getViaTime() - bandpt1->t());

    _error[1] = penaltyBoundToInterval(tf_via_point_to_new_pose.getOrigin().y(),
                                       _measurement->at(_nearest_index.front())->getRightBound(),
                                       _measurement->at(_nearest_index.front())->getLeftBound(), 
                                       steb_cfg_->optim.penalty_epsilon);

    tf2::Transform middle_center;
    middle_center.setIdentity();
    middle_center.setOrigin(tf2::Vector3(steb_cfg_->vehicle_param.rear_axle_2_center, 0.0, 0.0));
    tf2::Transform tf_map_to_middle_center = tf_map_to_new_pose * middle_center;

    tf2::Transform tf_map_to_middle_via_point;
    tf_map_to_middle_via_point.setOrigin(tf2::Vector3(_measurement->at(_nearest_index[1])->getViaPoint().x(),
                                                      _measurement->at(_nearest_index[1])->getViaPoint().y(), 0.0));
    tf2::Quaternion quaternion_middle;
    quaternion_middle.setRPY(0.0, 0.0, _measurement->at(_nearest_index[1])->getViaPoint().z());
    tf_map_to_middle_via_point.setRotation(quaternion_middle);
    tf2::Transform tf_middle_via_point_to_middle_center = tf_map_to_middle_via_point.inverse() * tf_map_to_middle_center;
    _error[2] = penaltyBoundToInterval(tf_middle_via_point_to_middle_center.getOrigin().y(),
                                       _measurement->at(_nearest_index[1])->getRightBound(),
                                       _measurement->at(_nearest_index[1])->getLeftBound(),
                                       steb_cfg_->optim.penalty_epsilon);

    tf2::Transform front_center;
    front_center.setIdentity();
    front_center.setOrigin(tf2::Vector3(steb_cfg_->vehicle_param.wheel_base, 0.0, 0.0));
    tf2::Transform tf_map_to_front_center = tf_map_to_new_pose * front_center;

    tf2::Transform tf_map_to_front_via_point;
    tf_map_to_front_via_point.setOrigin(tf2::Vector3(_measurement->at(_nearest_index[2])->getViaPoint().x(),
                                                     _measurement->at(_nearest_index[2])->getViaPoint().y(), 0.0));
    tf2::Quaternion quaternion_front;
    quaternion_front.setRPY(0.0, 0.0, _measurement->at(_nearest_index[2])->getViaPoint().z());
    tf_map_to_front_via_point.setRotation(quaternion_front);

    tf2::Transform tf_via_front_point_to_front_center = tf_map_to_front_via_point.inverse() * tf_map_to_front_center;
    _error[3] = penaltyBoundToInterval(tf_via_front_point_to_front_center.getOrigin().y(),
                                       _measurement->at(_nearest_index[2])->getRightBound(),
                                       _measurement->at(_nearest_index[2])->getLeftBound(),
                                       steb_cfg_->optim.penalty_epsilon);
//    _error[2] = std::fabs(bandpt1->t() - _measurement->getViaTime());
//    _error[2] = 0.0;

    STEB_ASSERT_MSG(std::isfinite(_error[0]), "EdgeViaPoint::computeError() _error[0]=%f\n",_error[0]);
    STEB_ASSERT_MSG(std::isfinite(_error[1]), "EdgeViaPoint::computeError() _error[1]=%f\n",_error[1]);
    STEB_ASSERT_MSG(std::isfinite(_error[2]), "EdgeViaPoint::computeError() _error[2]=%f\n",_error[2]);
    STEB_ASSERT_MSG(std::isfinite(_error[3]), "EdgeViaPoint::computeError() _error[3]=%f\n",_error[3]);
  }

  void setNearestViaPointIndex(std::vector<int> nearest_index)
  {
    _nearest_index = nearest_index;
  }

  void setParameters(const ViaAreaContainer* via_area)
  {
    _measurement = via_area;
  }

  inline int findNearestViaPoints(Eigen::Vector4d & pose, int start_index = 0)
  {
    int nearest_index = -1;
    double min_dist_sq = std::numeric_limits<double>::max();
    Eigen::Vector2d current_position{pose.x(), pose.y()};
    for (int j = start_index; j < static_cast<int>(_measurement->size()); ++j)
    {
      Eigen::Vector2d vp_it_2d{_measurement->at(j)->getViaPoint().x(),
                               _measurement->at(j)->getViaPoint().y()};
      double dist_sq = (current_position - vp_it_2d).squaredNorm();
      if (dist_sq < min_dist_sq)
      {
        min_dist_sq = dist_sq;
        nearest_index = j;
      }
    }
    return nearest_index;
  }

  inline std::vector<Eigen::Vector4d> findCenterOfCollisionCircle(Eigen::Vector4d & pose)
  {
    std::vector<Eigen::Vector4d> center_vector;

    tf2::Transform current_pose_tf;
    current_pose_tf.setOrigin(tf2::Vector3(pose.x(), pose.y(), 0.0));
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, pose.z());
    current_pose_tf.setRotation(quaternion);

    tf2::Transform middle_center;
    middle_center.setIdentity();
    middle_center.setOrigin(tf2::Vector3(steb_cfg_->vehicle_param.rear_axle_2_center, 0.0, 0.0));
    middle_center = current_pose_tf * middle_center;
    Eigen::Vector4d middle_center_eigen {middle_center.getOrigin().x(), middle_center.getOrigin().y(),
                                        tf2::getYaw(middle_center.getRotation()),  pose.w()};

    tf2::Transform front_center;
    middle_center.setIdentity();
    middle_center.setOrigin(tf2::Vector3(steb_cfg_->vehicle_param.wheel_base, 0.0, 0.0));
    front_center = current_pose_tf * front_center;
    Eigen::Vector4d front_center_eigen {front_center.getOrigin().x(), front_center.getOrigin().y(),
                                       tf2::getYaw(front_center.getRotation()),  pose.w()};

    center_vector.push_back(middle_center_eigen);
    center_vector.push_back(front_center_eigen);

    return center_vector;

  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::vector<int> _nearest_index;
};
  
    

} // end namespace

#endif
