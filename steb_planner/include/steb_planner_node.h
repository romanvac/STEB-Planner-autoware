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

#include <rclcpp/rclcpp.hpp>

#include "autoware_auto_perception_msgs/msg/predicted_object.hpp"
#include "autoware_auto_perception_msgs/msg/predicted_objects.hpp"
#include "autoware_auto_planning_msgs/msg/path.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <filesystem>
#include <fstream>
#include <iostream>

// new msgs
#include "autoware_perception_msgs/msg/predicted_objects.hpp"
#include "autoware_planning_msgs/msg/path.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"

// g2o custom edges and vertices for the STEB planner
#include "steb_planner/steb_obstacle/dynamic_obstacles.h"
#include "steb_planner/steb_optimal/spatio_temporal_elastic_band.h"
#include "steb_planner/steb_optimal/steb_config.h"
#include "steb_planner/steb_optimal/steb_planner.h"
#include "steb_planner/tic_toc.h"

#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <steb_planner/g2o_types/edge_acceleration.h>
#include <steb_planner/g2o_types/edge_kinematics.h>
#include <steb_planner/g2o_types/edge_obstacle.h>
#include <steb_planner/g2o_types/edge_shortest_path.h>
#include <steb_planner/g2o_types/edge_time_optimal.h>
#include <steb_planner/g2o_types/edge_velocity.h>
#include <steb_planner/g2o_types/edge_via_point.h>

class STEBPlannerNode : public rclcpp::Node
{
public:
  explicit STEBPlannerNode(const rclcpp::NodeOptions & options);

private:
  // ros tf
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_ptr_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_ptr_;

  // subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<autoware_planning_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<autoware_perception_msgs::msg::PredictedObjects>::SharedPtr objects_sub_;
  // publishers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_viz_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_obstacle_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_trajectory_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_costmap_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_costmap_origin_image_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr debug_occ_map_pub_;
  rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr traj_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr exe_time_pub_;
  // id
  int debug_trajectory_pub_id_ = 0;

  // Config
  steb_planner::STEBConfig steb_config_;
  // Subscription for parameter change
  rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;

  // data input
  std::unique_ptr<geometry_msgs::msg::TwistStamped> current_twist_ptr_;
  std::unique_ptr<autoware_auto_perception_msgs::msg::PredictedObjects> objects_ptr_;
  std::vector<steb_planner::ObstaclePtr> dynamic_obst_vector_;
  nav_msgs::msg::Odometry current_odom_;
  nav_msgs::msg::OccupancyGrid occ_map_;

  // steb planner
  std::unique_ptr<steb_planner::STEBPlanner> steb_planner_;
  steb_planner::TrajectoryPointsContainer via_points_;
  steb_planner::TrajectoryPointsContainer steb_optimized_points_;

  // mutex
  std::mutex obst_mutex_;

  // test data log output
  std::fstream ofs_;

  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr);
  void onPath(const autoware_planning_msgs::msg::Path::SharedPtr);
  void onObjects(const autoware_perception_msgs::msg::PredictedObjects::SharedPtr);

  void publishDebugMarker(const rclcpp::Time & time);

  inline std::string getTime()
  {
    time_t timep;
    time(&timep);
    char tmp[64];
    strftime(tmp, sizeof(tmp), "%Y-%m-%d_%H-%M-%S", localtime(&timep));
    return tmp;
  }

  inline geometry_msgs::msg::Quaternion createQuaternionFromRPY(
    const double roll, const double pitch, const double yaw)
  {
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    return tf2::toMsg(q);
  }

  inline geometry_msgs::msg::Quaternion createQuaternionFromYaw(const double yaw)
  {
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    return tf2::toMsg(q);
  }

  static inline double getInterPosesDistance2D(
    const geometry_msgs::msg::Pose & pose_1, const geometry_msgs::msg::Pose & pose_2)
  {
    double delta_x = pose_2.position.x - pose_1.position.x;
    double delta_y = pose_2.position.y - pose_1.position.y;

    return std::sqrt(delta_x * delta_x + delta_y * delta_y);
  }

  static inline double getInterPosesDistance2D(
    const Eigen::Vector4d & pose_st1, const Eigen::Vector4d & pose_st2)
  {
    double delta_x = pose_st2.x() - pose_st1.x();
    double delta_y = pose_st2.y() - pose_st1.y();

    return std::sqrt(delta_x * delta_x + delta_y * delta_y);
  }

  // get the bezier points
  inline Eigen::Vector3d bezierPoint(const std::vector<Eigen::Vector3d> & controlPoints, double t)
  {
    int n = controlPoints.size() - 1;
    Eigen::Vector3d point = {0, 0, 0};

    for (int i = 0; i <= n; ++i) {
      double binomialCoeff = tgamma(n + 1) / (tgamma(i + 1) * tgamma(n - i + 1));
      double factor = binomialCoeff * pow(1 - t, n - i) * pow(t, i);
      point.x() += factor * controlPoints[i].x();
      point.y() += factor * controlPoints[i].y();
      point.z() += factor * controlPoints[i].z();
    }

    return point;
  }

  // bezier interpolator
  inline std::vector<Eigen::Vector3d> generateBezierCurve(
    const std::vector<Eigen::Vector3d> & controlPoints, int numPoints)
  {
    std::vector<Eigen::Vector3d> curvePoints;

    for (int i = 0; i <= numPoints; ++i) {
      double t = static_cast<double>(i) / numPoints;
      curvePoints.push_back(bezierPoint(controlPoints, t));
    }

    return curvePoints;
  }
};
