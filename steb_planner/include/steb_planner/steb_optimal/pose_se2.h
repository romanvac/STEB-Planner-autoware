/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2016,
*  TU Dortmund - Institute of Control Theory and Systems Engineering.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the institute nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Christoph Rösmann
*********************************************************************/

#ifndef POSE_SE2_H_
#define POSE_SE2_H_

#include <g2o/stuff/misc.h>

#include <Eigen/Core>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>

#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "steb_planner/misc.h"

namespace steb_planner
{

// PoseSE2: This class implements a pose in the domain SE2
class PoseSE2
{
public:

 PoseSE2()
 {
   setZero();
 }

 PoseSE2(const Eigen::Ref<const Eigen::Vector2d>& position, double theta)
 {
   _position = position;
   _theta = theta;
 }

 PoseSE2(double x, double y, double theta)
 {
   _position.coeffRef(0) = x;
   _position.coeffRef(1) = y;
   _theta = theta;
 }


 PoseSE2(const geometry_msgs::msg::PoseStamped& pose)
     :PoseSE2(pose.pose)
 { }

 PoseSE2(const geometry_msgs::msg::Pose& pose)
 {
   _position.coeffRef(0) = pose.position.x;
   _position.coeffRef(1) = pose.position.y;
   _theta = tf2::getYaw( pose.orientation );
 }

 PoseSE2(const geometry_msgs::msg::Pose2D& pose)
 {
   _position.coeffRef(0) = pose.x;
   _position.coeffRef(1) = pose.y;
   _theta = pose.theta;
 }

 PoseSE2(const PoseSE2& pose)
 {
   _position = pose._position;
   _theta = pose._theta;
 }

 ~PoseSE2() {}


 // Access the 2D position part
 Eigen::Vector2d& position() {return _position;}

 // Access the 2D position part (read-only)
 const Eigen::Vector2d& position() const {return _position;}

 // Access the x-coordinate the pose
 double& x() {return _position.coeffRef(0);}

 const double& x() const {return _position.coeffRef(0);}

 // Access the y-coordinate the pose
 double& y() {return _position.coeffRef(1);}

 const double& y() const {return _position.coeffRef(1);}

 // Access the orientation part (yaw angle) of the pose
 double& theta() {return _theta;}

 const double& theta() const {return _theta;}

 //Set pose to [0,0,0]
 void setZero()
 {
   _position.setZero();
   _theta = 0;
 }

 // Convert PoseSE2 to a geometry_msgs::msg::Pose
 void toPoseMsg(geometry_msgs::msg::Pose& pose) const
 {
   pose.position.x = _position.x();
   pose.position.y = _position.y();
   pose.position.z = 0;
   tf2::Quaternion q;
   q.setRPY(0, 0, _theta);
   pose.orientation = tf2::toMsg(q);
 }

 // Convert PoseSE2 to a geometry_msgs::msg::Pose2D
 void toPoseMsg(geometry_msgs::msg::Pose2D& pose) const
 {
   pose.x = _position.x();
   pose.y = _position.y();
   pose.theta = _theta;
 }

 // Return the unit vector of the current orientation
 Eigen::Vector2d orientationUnitVec() const {return Eigen::Vector2d(std::cos(_theta), std::sin(_theta));}



 // Scale all SE2 components (x,y,theta) and normalize theta afterwards to [-pi, pi]
 void scale(double factor)
 {
   _position *= factor;
   _theta = g2o::normalize_theta( _theta*factor );
 }

 // Increment the pose by adding an array
 void plus(const double* pose_as_array)
 {
   _position.coeffRef(0) += pose_as_array[0];
   _position.coeffRef(1) += pose_as_array[1];
   _theta = g2o::normalize_theta( _theta + pose_as_array[2] );
 }

 // Get the mean / average of two poses and store it in the caller class
 void averageInPlace(const PoseSE2& pose1, const PoseSE2& pose2)
 {
   _position = (pose1._position + pose2._position)/2;
   _theta = g2o::average_angle(pose1._theta, pose2._theta);
 }

 // Get the mean / average of two poses and return the result (static)
 static PoseSE2 average(const PoseSE2& pose1, const PoseSE2& pose2)
 {
   return PoseSE2( (pose1._position + pose2._position)/2 , g2o::average_angle(pose1._theta, pose2._theta) );
 }

 // Rotate pose globally
 void rotateGlobal(double angle, bool adjust_theta=true)
 {
   double new_x = std::cos(angle)*_position.x() - std::sin(angle)*_position.y();
   double new_y = std::sin(angle)*_position.x() + std::cos(angle)*_position.y();
   _position.x() = new_x;
   _position.y() = new_y;
   if (adjust_theta)
     _theta = g2o::normalize_theta(_theta+angle);
 }


 // Asignment operator
 PoseSE2& operator=( const PoseSE2& rhs )
 {
   if (&rhs != this)
   {
     _position = rhs._position;
     _theta = rhs._theta;
   }
   return *this;
 }

 // Compound assignment operator (addition)
 PoseSE2& operator+=(const PoseSE2& rhs)
 {
   _position += rhs._position;
   _theta = g2o::normalize_theta(_theta + rhs._theta);
   return *this;
 }

 // Arithmetic operator overload for additions
 friend PoseSE2 operator+(PoseSE2 lhs, const PoseSE2& rhs)
 {
   return lhs += rhs;
 }

 // Compound assignment operator (subtraction)
 PoseSE2& operator-=(const PoseSE2& rhs)
 {
   _position -= rhs._position;
   _theta = g2o::normalize_theta(_theta - rhs._theta);
   return *this;
 }

 // Arithmetic operator overload for subtractions
 friend PoseSE2 operator-(PoseSE2 lhs, const PoseSE2& rhs)
 {
   return lhs -= rhs;
 }

 // Multiply pose with scalar and return copy without normalizing theta
 friend PoseSE2 operator*(PoseSE2 pose, double scalar)
 {
   pose._position *= scalar;
   pose._theta *= scalar;
   return pose;
 }

 friend PoseSE2 operator*(double scalar, PoseSE2 pose)
 {
   pose._position *= scalar;
   pose._theta *= scalar;
   return pose;
 }

 // Output stream operator
 friend std::ostream& operator<< (std::ostream& stream, const PoseSE2& pose)
 {
   stream << "x: " << pose._position[0] << " y: " << pose._position[1] << " theta: " << pose._theta;
   return stream;
 }


private:

 Eigen::Vector2d _position;
 double _theta;

public:
 EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


} // namespace steb_planner

#endif // POSE_SE2_H_
