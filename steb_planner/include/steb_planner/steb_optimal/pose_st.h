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

#ifndef POSE_ST_H_
#define POSE_ST_H_

#include <g2o/stuff/misc.h>
#include <Eigen/Core>

#include <tf/transform_datatypes.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "steb_planner/misc.h"

namespace steb_planner
{

// PoseST: The pose consist of the position x and y and a time.
class PoseST
{
public:
    
  PoseST()
  {
    setZero();
  }
      
  PoseST(const Eigen::Ref<const Eigen::Vector2d>& position, double t)
  {
      _position = Eigen::Vector3d {position.x(), position.y(), t};
  }
  
  PoseST(double x, double y, double t)
  {
      _position.coeffRef(0) = x;
      _position.coeffRef(1) = y;
      _position.coeffRef(2) = t;
  }

  PoseST(Eigen::Vector3d position)
  {
    _position = position;
  }

  ~PoseST() = default;
  
  
  Eigen::Vector2d& position()
  {
    Eigen::Vector2d position{_position.x() ,_position.y()};
    return position;
  }

  const Eigen::Vector2d& position() const
  {
    Eigen::Vector2d position{_position.x() ,_position.y()};
    return position;
  }
  
  // Access the x-coordinate the pose
  double& x() {return _position.coeffRef(0);}
  
  const double& x() const {return _position.coeffRef(0);}
  
  // Access the y-coordinate the pose
  double& y() {return _position.coeffRef(1);}
  
  const double& y() const {return _position.coeffRef(1);}
  
  // Access the time part of the pose
  double& t() {return _position.coeffRef(2);}
  
  const double& t() const {return _position.coeffRef(2);}
  
  // Set pose to [0,0,0]
  void setZero()
  {
    _position.setZero();
  }

	  
  // Increment the pose by adding a double[3] array
  void plus(const double* pose_as_array)
  {
    _position.coeffRef(0) += pose_as_array[0];
    _position.coeffRef(1) += pose_as_array[1];
    _position.coeffRef(2) += pose_as_array[2];
  }
  
  // Get the mean / average of two poses and store it in the caller class
  void averageInPlace(const PoseST& pose1, const PoseST& pose2)
  {
    _position = (pose1._position + pose2._position)/2;
  }
  
  // Get the mean / average of two poses and return the result (static)
  static PoseST average(const PoseST& pose1, const PoseST& pose2)
  {
    Eigen::Vector3d average_pose = (pose1._position + pose2._position) / 2;
    return PoseST(average_pose);
  }
  

  PoseST& operator=( const PoseST& rhs )
  {
    if (&rhs != this)
    {
	_position = rhs._position;
    }
    return *this;
  }

  PoseST& operator+=(const PoseST& rhs)
  {
    _position += rhs._position;
    return *this;
  }
  
  friend PoseST operator+(PoseST lhs, const PoseST& rhs)
  {
    return lhs += rhs;
  }
  
  PoseST& operator-=(const PoseST& rhs)
  {
    _position -= rhs._position;
    return *this;
  }
  
  friend PoseST operator-(PoseST lhs, const PoseST& rhs)
  {
    return lhs -= rhs;
  }
  
  friend PoseST operator*(PoseST pose, double scalar)
  {
    pose._position *= scalar;
    return pose;
  }
  
  friend PoseST operator*(double scalar, PoseST pose)
  {
    pose._position *= scalar;
    return pose;
  }
  
	friend std::ostream& operator<< (std::ostream& stream, const PoseST& pose)
	{
		stream << "x: " << pose._position[0] << " y: " << pose._position[1] << " t: " <<pose._position[2];
    return stream;
	}

      
private:
  
  Eigen::Vector3d _position;
      
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  
};


} // namespace steb_planner

#endif // POSE_ST_H_
