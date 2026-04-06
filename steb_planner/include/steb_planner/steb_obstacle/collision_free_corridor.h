//
// Created by hs on 24-2-1.
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

#ifndef SRC_COLLISION_FREE_CORRIDOR_CALC_H_
#define SRC_COLLISION_FREE_CORRIDOR_CALC_H_

#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"

#include <boost/none.hpp>
#include <boost/optional.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "tf2/utils.h"

#include "steb_planner/tic_toc.h"
#include "steb_planner/trajectory/trajectory.hpp"
#include "steb_planner/steb_optimal/steb_config.h"
#include "autoware_auto_perception_msgs/msg/predicted_object.hpp"
#include "autoware_auto_planning_msgs/msg/path_point.hpp"
#include "autoware_auto_perception_msgs/msg/predicted_objects.hpp"
#define UNUSED(x) (void)(x)

namespace steb_planner
{

#define PI 3.14159265358979323846
#define GRAVITY 9.80665

struct CVMaps
{
  cv::Mat drivable_area_grid_map;
  cv::Mat drivable_area_clearance_map;
  cv::Mat only_objects_grid_map;
  cv::Mat drivable_with_objects_grid_map;
  cv::Mat drivable_with_objects_clearance_map;
  cv::Mat debug_map;
  nav_msgs::msg::MapMetaData map_info;
};

struct PolygonPoints
{
  std::vector<geometry_msgs::msg::Point> points_in_image;
  std::vector<geometry_msgs::msg::Point> points_in_map;
};

enum class CollisionType { NO_COLLISION = 0, OUT_OF_SIGHT = 1, OUT_OF_ROAD = 2, OBJECT = 3 };

struct Bounds
{
  Bounds() = default;
  Bounds(const double lower_bound_, const double upper_bound_, 
         CollisionType lower_collision_type_, CollisionType upper_collision_type_)
      : lower_bound(lower_bound_), upper_bound(upper_bound_),
        lower_collision_type(lower_collision_type_), upper_collision_type(upper_collision_type_)
  { }

  double lower_bound;
  double upper_bound;

  CollisionType lower_collision_type;
  CollisionType upper_collision_type;

  bool hasCollisionWithRightObject() const { return lower_collision_type == CollisionType::OBJECT; }

  bool hasCollisionWithLeftObject() const { return upper_collision_type == CollisionType::OBJECT; }

  bool hasCollisionWithObject() const
  {
    return hasCollisionWithRightObject() || hasCollisionWithLeftObject();
  }

  void translate(const double offset)
  {
    lower_bound -= offset;
    upper_bound -= offset;
  }
};
using BoundsCandidates = std::vector<Bounds>;

class CollisionFreeCorridor
{
public:
  CollisionFreeCorridor(const STEBConfig* steb_config, const nav_msgs::msg::OccupancyGrid* occupancy_grid,
                        autoware_auto_perception_msgs::msg::PredictedObjects & objects,
                        autoware_auto_planning_msgs::msg::Path & path)
  {
    steb_config_ = steb_config;
    cv_maps_.drivable_area_grid_map = getDrivableAreaInCV(occupancy_grid);
    cv_maps_.drivable_area_clearance_map = getClearanceMap(cv_maps_.drivable_area_grid_map);

    std::vector<autoware_auto_perception_msgs::msg::PredictedObject> debug_avoiding_objects;
    cv::Mat objects_image = drawObstaclesOnImage(true, objects, 
                                                 path.points, path.drivable_area.info, 
                                                 cv_maps_.drivable_area_grid_map, cv_maps_.drivable_area_clearance_map, 
                                                 &debug_avoiding_objects);

    cv_maps_.drivable_with_objects_grid_map = getAreaWithObjects(cv_maps_.drivable_area_grid_map, objects_image);
    cv_maps_.drivable_with_objects_clearance_map = getClearanceMap(cv_maps_.drivable_with_objects_grid_map);
    cv_maps_.map_info = occupancy_grid->info;
  }
  ~CollisionFreeCorridor() = default;

  geometry_msgs::msg::Point transformToRelativeCoordinate2D(
      const geometry_msgs::msg::Point & point, const geometry_msgs::msg::Pose & origin)
  {
    // NOTE: implement transformation without defining yaw variable
    //       but directly sin/cos of yaw for fast calculation
    // const auto & q = origin.orientation;
    // const double cos_yaw = 1 - 2 * q.z * q.z;
    // const double sin_yaw = 2 * q.w * q.z;
    const double yaw = tf2::getYaw(origin.orientation);
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);

    geometry_msgs::msg::Point relative_p;
    const double tmp_x = point.x - origin.position.x;
    const double tmp_y = point.y - origin.position.y;
    relative_p.x = tmp_x * cos_yaw + tmp_y * sin_yaw;
    relative_p.y = -tmp_x * sin_yaw + tmp_y * cos_yaw;
    relative_p.z = point.z;

    return relative_p;
  }

  geometry_msgs::msg::Point transformToAbsoluteCoordinate2D(
      const geometry_msgs::msg::Point & point, const geometry_msgs::msg::Pose & origin)
  {
    // NOTE: implement transformation without defining yaw variable
    //       but directly sin/cos of yaw for fast calculation
    const auto & q = origin.orientation;
    const double cos_yaw = 1 - 2 * q.z * q.z;
    const double sin_yaw = 2 * q.w * q.z;

    geometry_msgs::msg::Point absolute_p;
    absolute_p.x = point.x * cos_yaw - point.y * sin_yaw + origin.position.x;
    absolute_p.y = point.x * sin_yaw + point.y * cos_yaw + origin.position.y;
    absolute_p.z = point.z;

    return absolute_p;
  }

  boost::optional<geometry_msgs::msg::Point> transformMapToOptionalImage(
      const geometry_msgs::msg::Point & map_point,
      const nav_msgs::msg::MapMetaData & occupancy_grid_info)
  {
    const geometry_msgs::msg::Point relative_p =
        transformToRelativeCoordinate2D(map_point, occupancy_grid_info.origin);
    const double resolution = occupancy_grid_info.resolution;
    const double map_y_height = occupancy_grid_info.height;
    const double map_x_width = occupancy_grid_info.width;
    const double map_x_in_image_resolution = relative_p.x / resolution;
    const double map_y_in_image_resolution = relative_p.y / resolution;
    const double image_x = map_y_height - map_y_in_image_resolution;
    const double image_y = map_x_width - map_x_in_image_resolution;

//    std::cout << " ###### map_point: " <<map_point.x << ", map_point y : "<<map_point.y << std::endl;
//    std::cout << " ^^^^^^ relative_p_x  : " <<relative_p.x << ", relative_p_y: "<<relative_p.y <<", resolution: "<<resolution<< std::endl;
//    std::cout << " ^^^^^^ map_x_in_image_resolution  : " <<map_x_in_image_resolution << ", map_y_in_image_resolution: "<<map_y_in_image_resolution << std::endl;
//    std::cout << " ^^^^^^ image_x  : " <<image_x << ", image_y: "<<image_y << std::endl;

    if (image_x >= 0 && image_x < static_cast<int>(map_y_height) && image_y >= 0 &&
        image_y < static_cast<int>(map_x_width)) {
      geometry_msgs::msg::Point image_point;
      image_point.x = image_x;
      image_point.y = image_y;
      return image_point;
    } else {
      return boost::none;
    }
  }

  bool transformMapToImage(
      const geometry_msgs::msg::Point & map_point,
      const nav_msgs::msg::MapMetaData & occupancy_grid_info, geometry_msgs::msg::Point & image_point)
  {
    geometry_msgs::msg::Point relative_p =
        transformToRelativeCoordinate2D(map_point, occupancy_grid_info.origin);
    const double map_y_height = occupancy_grid_info.height;
    const double map_x_width = occupancy_grid_info.width;
    const double scale = 1 / occupancy_grid_info.resolution;
    const double map_x_in_image_resolution = relative_p.x * scale;
    const double map_y_in_image_resolution = relative_p.y * scale;
    const double image_x = map_y_height - map_y_in_image_resolution;
    const double image_y = map_x_width - map_x_in_image_resolution;
    if (
        image_x >= 0 && image_x < static_cast<int>(map_y_height) && image_y >= 0 &&
        image_y < static_cast<int>(map_x_width)) {
      image_point.x = image_x;
      image_point.y = image_y;
      return true;
    } else {
      return false;
    }
  }

  PolygonPoints getPolygonPoints(
      const autoware_auto_perception_msgs::msg::PredictedObject & object,
      const nav_msgs::msg::MapMetaData & map_info)
  {
    std::vector<geometry_msgs::msg::Point> points_in_image;
    std::vector<geometry_msgs::msg::Point> points_in_map;
    PolygonPoints polygon_points;
    if (object.shape.type == object.shape.BOUNDING_BOX) {
      polygon_points = getPolygonPointsFromBB(object, map_info);
    } else if (object.shape.type == object.shape.CYLINDER) {
      polygon_points = getPolygonPointsFromCircle(object, map_info);
    } else if (object.shape.type == object.shape.POLYGON) {
      polygon_points = getPolygonPointsFromPolygon(object, map_info);
    }
    return polygon_points;
  }

  PolygonPoints getPolygonPointsFromBB(
      const autoware_auto_perception_msgs::msg::PredictedObject & object,
      const nav_msgs::msg::MapMetaData & map_info)
  {
    std::vector<geometry_msgs::msg::Point> points_in_image;
    std::vector<geometry_msgs::msg::Point> points_in_map;
    const double dim_x = object.shape.dimensions.x;
    const double dim_y = object.shape.dimensions.y;
    const std::vector<double> rel_x = {0.5 * dim_x, 0.5 * dim_x, -0.5 * dim_x, -0.5 * dim_x};
    const std::vector<double> rel_y = {0.5 * dim_y, -0.5 * dim_y, -0.5 * dim_y, 0.5 * dim_y};
    const geometry_msgs::msg::Pose object_pose = object.kinematics.initial_pose_with_covariance.pose;
    for (size_t i = 0; i < rel_x.size(); i++) {
      geometry_msgs::msg::Point rel_point;
      rel_point.x = rel_x[i];
      rel_point.y = rel_y[i];
      auto abs_point = transformToAbsoluteCoordinate2D(rel_point, object_pose);
      geometry_msgs::msg::Point image_point;
      if (transformMapToImage(abs_point, map_info, image_point)) {
        points_in_image.push_back(image_point);
        points_in_map.push_back(abs_point);
      }
    }
    PolygonPoints polygon_points;
    polygon_points.points_in_image = points_in_image;
    polygon_points.points_in_map = points_in_map;
    return polygon_points;
  }

  PolygonPoints getPolygonPointsFromCircle(
      const autoware_auto_perception_msgs::msg::PredictedObject & object,
      const nav_msgs::msg::MapMetaData & map_info)
  {
    std::vector<geometry_msgs::msg::Point> points_in_image;
    std::vector<geometry_msgs::msg::Point> points_in_map;
    const double radius = object.shape.dimensions.x;
    const geometry_msgs::msg::Point center =
        object.kinematics.initial_pose_with_covariance.pose.position;
    constexpr int num_sampling_points = 5;
    for (int i = 0; i < num_sampling_points; ++i) {
      std::vector<double> deltas = {0, 1.0};
      for (const auto & delta : deltas) {
        geometry_msgs::msg::Point point;
        point.x = std::cos(
                      ((i + delta) / static_cast<double>(num_sampling_points)) * 2.0 * M_PI +
                      M_PI / static_cast<double>(num_sampling_points)) *
                      (radius / 2.0) +
                  center.x;
        point.y = std::sin(
                      ((i + delta) / static_cast<double>(num_sampling_points)) * 2.0 * M_PI +
                      M_PI / static_cast<double>(num_sampling_points)) *
                      (radius / 2.0) +
                  center.y;
        point.z = center.z;
        geometry_msgs::msg::Point image_point;
        if (transformMapToImage(point, map_info, image_point)) {
          points_in_image.push_back(image_point);
          points_in_map.push_back(point);
        }
      }
    }
    PolygonPoints polygon_points;
    polygon_points.points_in_image = points_in_image;
    polygon_points.points_in_map = points_in_map;
    return polygon_points;
  }

  PolygonPoints getPolygonPointsFromPolygon(
      const autoware_auto_perception_msgs::msg::PredictedObject & object,
      const nav_msgs::msg::MapMetaData & map_info)
  {
    std::vector<geometry_msgs::msg::Point> points_in_image;
    std::vector<geometry_msgs::msg::Point> points_in_map;
    for (const auto & polygon_p : object.shape.footprint.points) {
      geometry_msgs::msg::Point rel_point;
      rel_point.x = polygon_p.x;
      rel_point.y = polygon_p.y;
      geometry_msgs::msg::Point point = transformToAbsoluteCoordinate2D(
          rel_point, object.kinematics.initial_pose_with_covariance.pose);
      const auto image_point = transformMapToOptionalImage(point, map_info);
      if (image_point) {
        points_in_image.push_back(image_point.get());
        points_in_map.push_back(point);
      }
    }
    PolygonPoints polygon_points;
    polygon_points.points_in_image = points_in_image;
    polygon_points.points_in_map = points_in_map;
    return polygon_points;
  }

  cv::Point toCVPoint(const geometry_msgs::msg::Point & p)
  {
    cv::Point cv_point;
    cv_point.x = p.x;
    cv_point.y = p.y;
    return cv_point;
  }

  std::vector<cv::Point> getDefaultCVPolygon(
      const std::vector<geometry_msgs::msg::Point> & points_in_image)
  {
    std::vector<cv::Point> cv_polygon;
    for (const auto & point : points_in_image) {
      cv::Point image_point = cv::Point(point.x, point.y);
      cv_polygon.push_back(image_point);
    }
    return cv_polygon;
  }

  size_t findNearestIndex(const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & points, const geometry_msgs::msg::Point & point)
  {
    if (points.empty())
      return {};


    double min_dist = std::numeric_limits<double>::max();
    size_t min_idx = 0;

    for (size_t i = 0; i < points.size(); ++i) {
      const auto dx = points.at(i).pose.position.x - point.x;
      const auto dy = points.at(i).pose.position.y - point.y;
      const auto dist = dx * dx + dy * dy;
//          tier4_autoware_utils::calcSquaredDistance2d(points.at(i), point);
      if (dist < min_dist) {
        min_dist = dist;
        min_idx = i;
      }
    }
    return min_idx;
  }

  bool isAvoidingObjectType(const autoware_auto_perception_msgs::msg::PredictedObject & object)
  {
    if (
        (object.classification.at(0).label == object.classification.at(0).UNKNOWN &&
         steb_config_->collision_free_corridor.is_avoiding_unknown) ||
        (object.classification.at(0).label == object.classification.at(0).CAR &&
         steb_config_->collision_free_corridor.is_avoiding_car) ||
        (object.classification.at(0).label == object.classification.at(0).TRUCK &&
         steb_config_->collision_free_corridor.is_avoiding_truck) ||
        (object.classification.at(0).label == object.classification.at(0).BUS &&
         steb_config_->collision_free_corridor.is_avoiding_bus) ||
        (object.classification.at(0).label == object.classification.at(0).BICYCLE &&
         steb_config_->collision_free_corridor.is_avoiding_bicycle) ||
        (object.classification.at(0).label == object.classification.at(0).MOTORCYCLE &&
         steb_config_->collision_free_corridor.is_avoiding_motorbike) ||
        (object.classification.at(0).label == object.classification.at(0).PEDESTRIAN &&
         steb_config_->collision_free_corridor.is_avoiding_pedestrian)) {
      return true;
    }
    return false;
  }


  bool isAvoidingObject(
      const PolygonPoints & polygon_points,
      const autoware_auto_perception_msgs::msg::PredictedObject & object, const cv::Mat & clearance_map,
      const nav_msgs::msg::MapMetaData & map_info,
      const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points)
  {
    if (path_points.empty()) {
      return false;
    }
    if (!isAvoidingObjectType(object)) {
      return false;
    }
    const auto image_point = transformMapToOptionalImage(
        object.kinematics.initial_pose_with_covariance.pose.position, map_info);
    if (!image_point) {
      return false;
    }

    // skip dynamic object
    const geometry_msgs::msg::Vector3 twist = object.kinematics.initial_twist_with_covariance.twist.linear;
    const double vel = std::sqrt(twist.x * twist.x + twist.y * twist.y + twist.z * twist.z);

    if ( vel > steb_config_->obstacles.max_static_obstacle_velocity ||
        !arePointsInsideDriveableArea(polygon_points.points_in_image, clearance_map)) {
      return false;
    }

    // skip object located back the beginning of path points
    const int nearest_idx = motion_utils::findNearestIndex(
        path_points, object.kinematics.initial_pose_with_covariance.pose.position);
    const auto nearest_path_point = path_points[nearest_idx];
    const auto rel_p = transformToRelativeCoordinate2D(
        object.kinematics.initial_pose_with_covariance.pose.position, nearest_path_point.pose);
    if (nearest_idx == 0 && rel_p.x < 0) {
      return false;
    }

    return true;
  }

  bool arePointsInsideDriveableArea(
      const std::vector<geometry_msgs::msg::Point> & image_points, const cv::Mat & clearance_map)
  {
    bool points_inside_area = false;
    for (const auto & image_point : image_points) {
      const float clearance =
          clearance_map.ptr<float>(static_cast<int>(image_point.y))[static_cast<int>(image_point.x)];
      if (clearance > 0) {
        points_inside_area = true;
      }
    }
    return points_inside_area;
  }

  cv::Mat getAreaWithObjects(
      const cv::Mat & drivable_area, const cv::Mat & objects_image) const
  {
    cv::Mat area_with_objects = cv::min(objects_image, drivable_area);
    return area_with_objects;
  }


  cv::Mat drawObstaclesOnImage(
      const bool enable_avoidance,
      const autoware_auto_perception_msgs::msg::PredictedObjects & objects,
      const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
      const nav_msgs::msg::MapMetaData & map_info, const cv::Mat & drivable_area,
      const cv::Mat & clearance_map,
      std::vector<autoware_auto_perception_msgs::msg::PredictedObject> * debug_avoiding_objects)
  {
    steb_planner::TicToc stop_watch_;
    stop_watch_.tic();

    std::vector<autoware_auto_planning_msgs::msg::PathPoint> path_points_inside_area;
    for (const auto & point : path_points) {
      std::vector<geometry_msgs::msg::Point> points;
      geometry_msgs::msg::Point image_point;
      if (!transformMapToImage(point.pose.position, map_info, image_point))
        continue;
  
      const float clearance = clearance_map.ptr<float>(static_cast<int>(image_point.y))[static_cast<int>(image_point.x)];
      if (clearance < 1e-5) 
        continue;
      path_points_inside_area.push_back(point);
    }

    // NOTE: objects image is too sparse so that creating cost map is heavy.
    //       Then, objects image is created by filling dilated drivable area,
    //       instead of "cv::Mat objects_image = cv::Mat::ones(clearance_map.rows, clearance_map.cols,
    //       CV_8UC1) * 255;".
    constexpr double dilate_margin = 1.0;
    cv::Mat objects_image;
    const int dilate_size = static_cast<int>((1.8 + dilate_margin) / (std::sqrt(2) * map_info.resolution)); 
    cv::dilate(drivable_area, objects_image, cv::Mat(), cv::Point(-1, -1), dilate_size);

    if (!enable_avoidance) {
      return objects_image;
    }

    // fill object
    std::vector<std::vector<cv::Point>> cv_polygons;
    std::vector<std::array<double, 2>> obj_cog_info;
    std::vector<geometry_msgs::msg::Point> obj_positions;
    for (const auto & object : objects.objects)
    {
      const PolygonPoints polygon_points = getPolygonPoints(object, map_info);
      bool avoiding = isAvoidingObject(polygon_points, object, clearance_map, 
                                     map_info, path_points_inside_area);
      RCLCPP_INFO(rclcpp::get_logger("steb_planner"),
        "Object label=%d vel=%.2f avoiding=%d poly_pts=%zu",
        object.classification.at(0).label,
        object.kinematics.initial_twist_with_covariance.twist.linear.x,
        (int)avoiding,
        polygon_points.points_in_image.size());

      if (isAvoidingObject(polygon_points, object, clearance_map, map_info,
                           path_points_inside_area))
      {
        const double lon_dist_to_path = motion_utils::calcSignedArcLength(
            path_points, 0, object.kinematics.initial_pose_with_covariance.pose.position);
        const double lat_dist_to_path = motion_utils::calcLateralOffset(
            path_points, object.kinematics.initial_pose_with_covariance.pose.position);
        obj_cog_info.push_back({lon_dist_to_path, lat_dist_to_path});
        obj_positions.push_back(object.kinematics.initial_pose_with_covariance.pose.position);

        cv_polygons.push_back(getDefaultCVPolygon(polygon_points.points_in_image));
        debug_avoiding_objects->push_back(object);
      }
    }
    for (const auto & polygon : cv_polygons) {
      cv::fillConvexPoly(objects_image, polygon, cv::Scalar(0));
    }

    // fill between objects in the same side
    if (false)
    {
      const auto get_closest_obj_point = [&](size_t idx) {
        const auto & path_point =
            path_points.at(findNearestIndex(path_points, obj_positions.at(idx)));
        double min_dist = std::numeric_limits<double>::min();
        size_t min_idx = 0;
        for (size_t p_idx = 0; p_idx < cv_polygons.at(idx).size(); ++p_idx) {
          const double dist =
              std::hypot(cv_polygons.at(idx).at(p_idx).x - path_point.pose.position.x,
                        cv_polygons.at(idx).at(p_idx).y - path_point.pose.position.y);
  //            tier4_autoware_utils::calcDistance2d(cv_polygons.at(idx).at(p_idx), path_point);
          if (dist < min_dist) {
            min_dist = dist;
            min_idx = p_idx;
          }
        }

        geometry_msgs::msg::Point geom_point;
        geom_point.x = cv_polygons.at(idx).at(min_idx).x;
        geom_point.y = cv_polygons.at(idx).at(min_idx).y;
        return geom_point;
      };

      std::vector<std::vector<cv::Point>> cv_between_polygons;
      for (size_t i = 0; i < obj_positions.size(); ++i) {
        for (size_t j = i + 1; j < obj_positions.size(); ++j) {
          const auto & obj_info1 = obj_cog_info.at(i);
          const auto & obj_info2 = obj_cog_info.at(j);

          // RCLCPP_ERROR_STREAM(rclcpp::get_logger("lat"), obj_info1.at(1) << " " << obj_info2.at(1));
          // RCLCPP_ERROR_STREAM(rclcpp::get_logger("lon"), obj_info1.at(0) << " " << obj_info2.at(0));

          constexpr double max_lon_dist_to_convex_obstacles = 30.0;
          if (obj_info1.at(1) * obj_info2.at(1) < 0 ||
              std::abs(obj_info1.at(0) - obj_info2.at(0)) > max_lon_dist_to_convex_obstacles) {
            continue;
          }

          std::vector<cv::Point> cv_points;
          cv_points.push_back(toCVPoint(obj_positions.at(i)));
          cv_points.push_back(toCVPoint(get_closest_obj_point(i)));
          cv_points.push_back(toCVPoint(get_closest_obj_point(j)));
          cv_points.push_back(toCVPoint(obj_positions.at(j)));

          cv_between_polygons.push_back(cv_points);
        }
      }
      
      for (const auto & polygon : cv_between_polygons) {
        cv::fillConvexPoly(objects_image, polygon, cv::Scalar(0));
      }
    }
    
    return objects_image;
  }

  void getOccupancyGridValue(
      const nav_msgs::msg::OccupancyGrid* og, const int i, const int j, unsigned char & value)
  {
    int i_flip = og->info.width - i - 1;
    int j_flip = og->info.height - j - 1;
    if (og->data[i_flip + j_flip * og->info.width] > 0) {
      value = 0;
    } else {
      value = 255;
    }
//    if (og->data[i_flip + j_flip * og->info.width] == 70 || og->data[i_flip + j_flip * og->info.width] == 102) {
//    if (og->data[i_flip + j_flip * og->info.width] == 102) {
//      value = 255;
//    } else {
//      value = 0;
//    }
  }

  cv::Mat getDrivableAreaInCV(const nav_msgs::msg::OccupancyGrid* occupancy_grid)
  {
    cv::Mat drivable_area = cv::Mat(occupancy_grid->info.width, occupancy_grid->info.height, CV_8UC1);

    drivable_area.forEach<unsigned char>([&](unsigned char & value, const int * position) -> void {
      getOccupancyGridValue(occupancy_grid, position[0], position[1], value);
    });

    return drivable_area;
  }

  cv::Mat getClearanceMap(const cv::Mat & drivable_area)
  {
    cv::Mat clearance_map;

    // steb_planner::TicToc tic_toc;
    // tic_toc.tic();
    cv::distanceTransform(drivable_area, clearance_map, cv::DIST_L2, 5);
    // auto time = tic_toc.toc();
    // std::cout << "*********** clearance map convert time cost: " << time << " ms, "<< std::endl;

    return clearance_map;
  }

  boost::optional<double> getClearance(
      const cv::Mat & clearance_map, const geometry_msgs::msg::Point & map_point,
      const nav_msgs::msg::MapMetaData & map_info)
  {
    const auto image_point = transformMapToOptionalImage(map_point, map_info);
    if (!image_point) {
      return boost::none;
    }
    const float clearance =
        clearance_map.ptr<float>(static_cast<int>(image_point.get().y))[static_cast<int>(image_point.get().x)] * map_info.resolution;
    // std::cout << "get clearance: " << clearance_map.ptr<float>(static_cast<int>(image_point.get().y))[static_cast<int>(image_point.get().x)]
    //           <<",  map_info.resolution" <<map_info.resolution<< ", "<< clearance<< std::endl;

    // cv_maps_.debug_map.ptr<float>(static_cast<int>(image_point.get().y))[static_cast<int>(image_point.get().x)] = 200;

    // clearance_map.at<int>(static_cast<int>(image_point.get().y), static_cast<int>(image_point.get().x)) = 200;
    // clearance_map.at<int>(static_cast<int>(image_point.get().y), static_cast<int>(image_point.get().x+1)) = 200;
    // clearance_map.at<int>(static_cast<int>(image_point.get().y+1), static_cast<int>(image_point.get().x)) = 200;
    // clearance_map.at<int>(static_cast<int>(image_point.get().y+1), static_cast<int>(image_point.get().x+1)) = 200;
    return clearance;
  }


  // 0.NO_COLLISION, 1.OUT_OF_SIGHT, 2.OUT_OF_ROAD, 3.OBJECT
  CollisionType getCollisionType(
      const bool enable_avoidance, const CVMaps & maps, const geometry_msgs::msg::Pose & avoiding_point,
      const double traversed_dist, const double bound_angle)
  {
    if (!enable_avoidance) {
      return CollisionType::NO_COLLISION;
    }

    // calculate clearance
    // const double min_soft_road_clearance = steb_config_->collision_free_corridor.soft_clearance_from_road
    //                                  + steb_config_->collision_free_corridor.extra_desired_clearance_from_road;
    const double min_soft_road_clearance = steb_config_->vehicle_param.circle_radius +
                                           steb_config_->collision_free_corridor.soft_clearance_from_road +
                                           steb_config_->collision_free_corridor.extra_desired_clearance_from_road;

    RCLCPP_INFO_ONCE(rclcpp::get_logger("steb_planner"), 
      "min_soft_road_clearance = %.3f (circle_radius=%.3f + soft=%.3f + extra=%.3f)",
      min_soft_road_clearance,
      steb_config_->vehicle_param.circle_radius,
      steb_config_->collision_free_corridor.soft_clearance_from_road,
      steb_config_->collision_free_corridor.extra_desired_clearance_from_road);
    // sleep(100);
    //      const double min_obj_clearance = vehicle_param_.width / 2.0 + param_.clearance_from_object +
    //                                       param_.soft_clearance_from_road;

    // calculate target position
    geometry_msgs::msg::Point target_pos;
    target_pos.x = avoiding_point.position.x + traversed_dist * std::cos(bound_angle);
    target_pos.y = avoiding_point.position.y + traversed_dist * std::sin(bound_angle);
    target_pos.z = 0.0;

    const auto opt_road_clearance = 
        getClearance(maps.drivable_with_objects_clearance_map, target_pos, maps.map_info);

//    const auto opt_obj_clearance =
//        getClearance(maps.only_objects_clearance_map, target_pos, maps.map_info);

    // object has more priority than road, so its condition exists first
//      if (enable_avoidance && opt_obj_clearance) {
//        const bool is_obj = opt_obj_clearance.get() < min_obj_clearance;
//        if (is_obj) {
//          return CollisionType::OBJECT;
//        }
//      }

//    std::cout << ">>>>>>>>>>>>>>> opt_road_clearance: " << traversed_dist << " | "
//              << opt_road_clearance.get() <<" : "
//              << (opt_road_clearance.get() < min_soft_road_clearance) << std::endl;

    if (opt_road_clearance) {
      const bool out_of_road = opt_road_clearance.get() < min_soft_road_clearance;
      if (out_of_road) {
        return CollisionType::OUT_OF_ROAD;
      } else {
        return CollisionType::NO_COLLISION;
      }
    }

    return CollisionType::OUT_OF_SIGHT;
  }

  inline double normalizeDegree(const double deg, const double min_deg = -180)
  {
    const auto max_deg = min_deg + 360.0;

    const auto value = std::fmod(deg, 360.0);
    if (min_deg <= value && value < max_deg) {
      return value;
    }

    return value - std::copysign(360.0, value);
  }

  inline double normalizeRadian(const double rad, const double min_rad = -PI)
  {
    const auto max_rad = min_rad + 2 * PI;

    const auto value = std::fmod(rad, 2 * PI);
    if (min_rad <= value && value < max_rad) {
      return value;
    }

    return value - std::copysign(2 * PI, value);
  }

  BoundsCandidates getBoundsCandidates(
      const bool enable_avoidance, const geometry_msgs::msg::Pose & avoiding_point, const CVMaps & maps)
  {
    BoundsCandidates bounds_candidate;

//    constexpr double max_search_lane_width = 5.0;
    const auto search_step_widths = steb_config_->collision_free_corridor.bounds_search_step_widths;

    // search right to left
    const double bound_search_direction = normalizeRadian(tf2::getYaw(avoiding_point.orientation) + M_PI_2);

    double traversed_dist = -steb_config_->collision_free_corridor.max_bound_search_width;
    double current_right_bound = -steb_config_->collision_free_corridor.max_bound_search_width;

    // calculate the initial position is empty or not
    // 0.drivable, 1.out of map, 2.out of road, 3. object
    CollisionType previous_collision_type =
        getCollisionType(enable_avoidance, maps, avoiding_point, traversed_dist, bound_search_direction);

    const auto has_collision = [&](const CollisionType & collision_type) -> bool {
      return collision_type == CollisionType::OUT_OF_ROAD || collision_type == CollisionType::OBJECT;
    };

    CollisionType latest_right_bound_collision_type = previous_collision_type;

    while (traversed_dist < steb_config_->collision_free_corridor.max_bound_search_width)
    {
      for (size_t search_idx = 0; search_idx < search_step_widths.size(); ++search_idx)
      {
        const double ds = search_step_widths.at(search_idx);
        while (true)
        {
          const CollisionType current_collision_type =
              getCollisionType(enable_avoidance, maps, avoiding_point, traversed_dist, bound_search_direction);

          if (has_collision(current_collision_type))
          {  // currently collision
            if (!has_collision(previous_collision_type))
            {
              // if target_position becomes collision from no collision or out_of_sight
              if (search_idx == search_step_widths.size() - 1)
              {
                const double left_bound = traversed_dist - ds / 2.0;
                bounds_candidate.push_back(Bounds{current_right_bound, left_bound,
                                                  latest_right_bound_collision_type,
                                                  current_collision_type});
                previous_collision_type = current_collision_type;
              }
              break;
            }
          }
          else if (current_collision_type == CollisionType::OUT_OF_SIGHT)
          {  // currently out_of_sight
            if (previous_collision_type == CollisionType::NO_COLLISION)
            {
              // if target_position becomes out_of_sight from no collision
              if (search_idx == search_step_widths.size() - 1)
              {
                const double left_bound = steb_config_->collision_free_corridor.max_bound_search_width;
                bounds_candidate.push_back(Bounds{current_right_bound, left_bound,
                                                     latest_right_bound_collision_type,
                                                     current_collision_type});
                previous_collision_type = current_collision_type;
              }
              break;
            }
          }
          else
          { // currently no collision
            if (has_collision(previous_collision_type))
            {
              // if target_position becomes no collision from collision
              if (search_idx == search_step_widths.size() - 1)
              {
                current_right_bound = traversed_dist - ds / 2.0;
                latest_right_bound_collision_type = previous_collision_type;
                previous_collision_type = current_collision_type;
              }
              break;
            }
          }

          // if target_position is longer than max_search_lane_width
          if (traversed_dist >= steb_config_->collision_free_corridor.max_bound_search_width)
          {
            if (!has_collision(previous_collision_type))
            {
              if (search_idx == search_step_widths.size() - 1)
              {
                const double left_bound = traversed_dist - ds / 2.0;
                bounds_candidate.push_back(Bounds{current_right_bound, left_bound,
                                                     latest_right_bound_collision_type,
                                                     CollisionType::OUT_OF_ROAD});
              }
            }
            break;
          }
          // go forward with ds
          traversed_dist += ds;
          previous_collision_type = current_collision_type;
        }

        if (search_idx != search_step_widths.size() - 1)
        {
          // go back with ds since target_position became empty or road/object
          // NOTE: if ds is the last of search_widths, don't have to go back
          traversed_dist -= ds;
        }
      }
    }

    // if empty
    if (bounds_candidate.empty()) {
      // NOTE: set invalid bounds so that MPT won't be solved
      const auto invalid_bounds = Bounds{-5.0, 5.0,
                                         CollisionType::OUT_OF_ROAD,
                                         CollisionType::OUT_OF_ROAD};
      bounds_candidate.push_back(invalid_bounds);
    }

    return bounds_candidate;
  }

  BoundsCandidates calcBound(Eigen::Vector3d& via_point)
  {
    geometry_msgs::msg::Pose avoiding_point;
    avoiding_point.position.x = via_point.x();
    avoiding_point.position.y = via_point.y();
    avoiding_point.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, via_point.z());
    avoiding_point.orientation = tf2::toMsg(q);
    return getBoundsCandidates(true, avoiding_point, cv_maps_);
  }

  CVMaps getCVMaps(){return cv_maps_;}

private:
  CVMaps cv_maps_;
  const STEBConfig* steb_config_;

};

} //end of namespace collision_free_corridor



#endif // SRC_COLLISION_FREE_CORRIDOR_CALC_H_
