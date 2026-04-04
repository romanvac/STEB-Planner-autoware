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
* Author: HeShan MaYalong
*********************************************************************/

#include "steb_planner_node.h"
#include "converter.hpp"

STEBPlannerNode::STEBPlannerNode(const rclcpp::NodeOptions& options)
    : Node("steb_planner_node", options)
{
  // tf
  tf_buffer_ptr_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ptr_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_ptr_);

  // subscribers
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/localization/kinematic_state", rclcpp::QoS{1},
      std::bind(&STEBPlannerNode::onOdometry, this, std::placeholders::_1));

  path_sub_ = create_subscription<autoware_planning_msgs::msg::Path>(
      "~/input/path", rclcpp::QoS{1},
      std::bind(&STEBPlannerNode::onPath, this, std::placeholders::_1));

  drivable_area_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "~/input/drivable_area", rclcpp::QoS{1},
    std::bind(&STEBPlannerNode::onDrivableArea, this, std::placeholders::_1));

  objects_sub_ = create_subscription<autoware_perception_msgs::msg::PredictedObjects>(
      "~/input/objects", rclcpp::QoS{10},
      std::bind(&STEBPlannerNode::onObjects, this, std::placeholders::_1));

  // publishers
  traj_pub_ = create_publisher<autoware_planning_msgs::msg::Trajectory>("~/output/path", 1);
  debug_viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/markers", 1);
  debug_obstacle_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/static_obstacle_markers", 1);
  debug_trajectory_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/trajectory_markers", 1);
  debug_costmap_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("~/debug/occ_map_image_pub", 1);
  debug_costmap_origin_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("~/debug/occ_map_origin_image_pub", 1);
  debug_occ_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("~/debug/occ_map_pub", 1);
  exe_time_pub_ = this->create_publisher<std_msgs::msg::Float32>("~/debug/execute_time", 1);


  // steb_planner
  steb_config_.declareParameters(this);
  steb_config_.loadRosParamFromNodeHandle(this);
  // Setup callback for changes to parameters.
  {
    parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
        this->get_node_base_interface(),
        this->get_node_topics_interface(),
        this->get_node_graph_interface(),
        this->get_node_services_interface());

    parameter_event_sub_ = parameters_client_->on_parameter_event(
        std::bind(&steb_planner::STEBConfig::on_parameter_event_callback, std::ref(steb_config_), std::placeholders::_1));
  }

  dynamic_obst_vector_.reserve(500);

  steb_planner_ = std::make_unique<steb_planner::STEBPlanner>
      (steb_planner::STEBPlanner(this, &steb_config_, &current_odom_, 
                                 &dynamic_obst_vector_, &via_points_, &occ_map_));

}

void STEBPlannerNode::onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> l(obst_mutex_);
  current_twist_ptr_ = std::make_unique<geometry_msgs::msg::TwistStamped>();
  current_twist_ptr_->header = msg->header;
  current_twist_ptr_->twist = msg->twist.twist;

  current_odom_.header = msg->header;
  current_odom_.pose = msg->pose;
  current_odom_.twist = msg->twist;
  current_odom_.child_frame_id = msg->child_frame_id;

  if (steb_config_.trajectory.current_velocity_for_debug > 0.0)
  {
    current_odom_.twist.twist.linear.x = steb_config_.trajectory.current_velocity_for_debug;
  }
}

void STEBPlannerNode::onObjects(const autoware_perception_msgs::msg::PredictedObjects::SharedPtr msg)
{
  // std::cout << "------- get obstacle msg ------- "<<msg->objects.size()<< std::endl;
  std::lock_guard<std::mutex> l(obst_mutex_);
  const auto auto_objects = toAutoPredictedObjects(*msg);
  objects_ptr_ = std::make_unique<autoware_auto_perception_msgs::msg::PredictedObjects>(auto_objects);
  
  // @hs: add dynamic obstacles
  dynamic_obst_vector_.clear();

  // transform
  geometry_msgs::msg::TransformStamped msg_tf_ego_to_predict_frame{};
  try {
    msg_tf_ego_to_predict_frame = tf_buffer_ptr_->lookupTransform(
        "base_link", objects_ptr_->header.frame_id, objects_ptr_->header.stamp,
        rclcpp::Duration::from_seconds(0.5));
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_STREAM(get_logger(), "Failed to look up transform from "
                                         << "base_link" << " to "
                                         << objects_ptr_->header.frame_id);
    return ;
  }
  tf2::Transform tf_ego_to_predict_frame;
  tf2::fromMsg(msg_tf_ego_to_predict_frame.transform, tf_ego_to_predict_frame);

  visualization_msgs::msg::MarkerArray debug_markers;
  // obstalce
  visualization_msgs::msg::Marker obstacle_marker;
  obstacle_marker.header.frame_id = "map";
  obstacle_marker.header.stamp = msg->header.stamp;
  obstacle_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  debug_markers.markers.push_back(obstacle_marker);

  int id=200;
  obstacle_marker.color.a = 0.6;
  obstacle_marker.color.r = 1.0;
  obstacle_marker.color.g = 0.0;
  obstacle_marker.color.b = 0.0;
  obstacle_marker.type = visualization_msgs::msg::Marker::CUBE;
  obstacle_marker.action = visualization_msgs::msg::Marker::ADD;

  visualization_msgs::msg::Marker line_obstacle_marker;
  line_obstacle_marker.header.frame_id = "base_link";
  line_obstacle_marker.header.stamp = msg->header.stamp;
  line_obstacle_marker.color.a = 0.2;
  line_obstacle_marker.color.r = 1.0;
  line_obstacle_marker.color.g = 0.0;
  line_obstacle_marker.color.b = 0.0;
  line_obstacle_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  line_obstacle_marker.action = visualization_msgs::msg::Marker::ADD;
  line_obstacle_marker.id = 199;
  line_obstacle_marker.scale.x = 0.2;
  line_obstacle_marker.scale.y = 0.2;
  line_obstacle_marker.scale.z = 0.2;

  // 遍历每个动态障碍物
  for (auto& object : objects_ptr_->objects)
  {
    obstacle_marker.scale.x = object.shape.dimensions.x;
    obstacle_marker.scale.y = object.shape.dimensions.y;
    obstacle_marker.scale.z = object.shape.dimensions.z;
    obstacle_marker.id = id++;
    obstacle_marker.pose = object.kinematics.initial_pose_with_covariance.pose;
    obstacle_marker.pose.position.z = obstacle_marker.scale.z * 0.5;
    debug_markers.markers.push_back(obstacle_marker);
    // @hs only process dynamic obstacles
    if (object.kinematics.initial_twist_with_covariance.twist.linear.x < 
        steb_config_.obstacles.max_static_obstacle_velocity) 
      continue ;

    // 遍历每条预测轨迹
//    for (auto & path : object.kinematics.predicted_paths)
    auto & path = object.kinematics.predicted_paths.at(0);
    {
      int time_count=0;
      double time_step = path.time_step.nanosec / 1e9;
      tf2::Transform tf_predict_traj_pose = tf2::Transform::getIdentity();
      // 遍历每条轨迹的预测点
      for (auto & position : path.path)
      {
        tf2::fromMsg(position, tf_predict_traj_pose);
        tf2::Transform tf_predict_traj_pose_in_ego = tf_ego_to_predict_frame * tf_predict_traj_pose;

        // point obstacle
//        {
//          steb_planner::ObstaclePtr new_obstacle(new steb_planner::PointObstacle(
//              tf_predict_traj_pose_in_ego.getOrigin().x(),tf_predict_traj_pose_in_ego.getOrigin().y(),
//              time_count * time_step));
//          // td::cout <<"time: "<<time_count * time_step << std::endl;
//          // set velocity
//          double yaw_in_ego = tf2::getYaw(tf_predict_traj_pose_in_ego.getRotation());
//          Eigen::Vector2d velocity{object.kinematics.initial_twist_with_covariance.twist.linear.x * std::cos(yaw_in_ego),
//                                   object.kinematics.initial_twist_with_covariance.twist.linear.x * std::sin(yaw_in_ego)};
//          new_obstacle->setCentroidVelocity(velocity);
//        }

        // pill obstacle
        tf2::Transform front_center;
        front_center.setIdentity();
        front_center.setOrigin(tf2::Vector3(object.shape.dimensions.x * 0.5, 0.0, 0.0));
        front_center = tf_predict_traj_pose_in_ego * front_center;
        Eigen::Vector2d end_point {front_center.getOrigin().x(), front_center.getOrigin().y()};

        tf2::Transform rear_center;
        rear_center.setIdentity();
        rear_center.setOrigin(tf2::Vector3(-object.shape.dimensions.x * 0.5, 0.0, 0.0));
        rear_center = tf_predict_traj_pose_in_ego * rear_center;
        Eigen::Vector2d start_point {rear_center.getOrigin().x(), rear_center.getOrigin().y()};

        steb_planner::ObstaclePtr new_obstacle(
            new steb_planner::STEBPillObstacle(start_point, end_point,
                                               object.shape.dimensions.y,
                                               time_count * time_step));
        dynamic_obst_vector_.push_back(new_obstacle);

        // obstalce line
        {
          geometry_msgs::msg::Point st_position_start;
          st_position_start.x = start_point.x();
          st_position_start.y = start_point.y();
          st_position_start.z = time_count * time_step;
          line_obstacle_marker.points.push_back(st_position_start);

          geometry_msgs::msg::Point st_position_end;
          st_position_end.x = end_point.x();
          st_position_end.y = end_point.y();
          st_position_end.z = time_count * time_step;
          line_obstacle_marker.points.push_back(st_position_end);

          debug_markers.markers.push_back(line_obstacle_marker);
        }
        time_count ++;
      }
    }
  }

  debug_obstacle_pub_->publish(debug_markers);
  // std::cout << ">>>>> obst vector size : " << dynamic_obst_vector_.size() <<std::endl;
  publishDebugMarker(msg->header.stamp);

}

void STEBPlannerNode::onDrivableArea(const nav_msgs::msg::OccupancyGrid::SharedPtr msg_ptr) {
  drivable_area_ptr_ = msg_ptr;
}


void STEBPlannerNode::onPath(const autoware_planning_msgs::msg::Path::SharedPtr path_ptr_new)
{
  const auto path_ptr = toAutoPath(*path_ptr_new);
  
  if (!drivable_area_ptr_)
    return;

  path_ptr->drivable_area = *drivable_area_ptr_;


  if (path_ptr->points.empty() || path_ptr->drivable_area.data.empty() || !objects_ptr_)
  {
    RCLCPP_WARN_STREAM(get_logger(), "[ steb_planner ]: path or driveable area is empty. ");
    return;
  }
  // get ego to map transform
  geometry_msgs::msg::TransformStamped msg_tf_ego_to_path_frame{};
  try {
    msg_tf_ego_to_path_frame = tf_buffer_ptr_->lookupTransform(
        "base_link", path_ptr->header.frame_id, path_ptr->header.stamp,
        rclcpp::Duration::from_seconds(0.5));
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_STREAM(get_logger(), "Failed to look up transform from " << "base_link" << " to "
                                                                         << path_ptr->header.frame_id);
    return ;
  }
  tf2::Transform tf_ego_to_path_frame;
  tf2::fromMsg(msg_tf_ego_to_path_frame.transform, tf_ego_to_path_frame);

  // @hs add via points
  // transform path to ego frame; x,y,yaw,v
  steb_planner::TrajectoryPointsContainer path_points_in_ego_frame;
  tf2::Transform tf_path_pose = tf2::Transform::getIdentity();
  for (auto& path_point : path_ptr->points)
  {
    tf2::fromMsg(path_point.pose, tf_path_pose);
    tf2::Transform tf_path_pose_in_ego = tf_ego_to_path_frame * tf_path_pose;
    Eigen::Vector4d p_w = {tf_path_pose_in_ego.getOrigin().x(),
                           tf_path_pose_in_ego.getOrigin().y(),
                           tf2::getYaw(tf_path_pose_in_ego.getRotation()),
                           path_point.longitudinal_velocity_mps};
    path_points_in_ego_frame.emplace_back(p_w);
  }

  // find the nearest index
  Eigen::Vector4d current_pose = {0,0,0,0};
  int nearest_index = steb_planner::findNearestIndex(path_points_in_ego_frame, current_pose);
  if (nearest_index == -1 )
    return ;

  // Remove points that have already passed by; And give time
  double forward_distance = 0.0;
  double forward_time = 0.0;
  via_points_.clear();
  via_points_.emplace_back(current_pose);
  Eigen::Vector4d last_pose = current_pose;
  for (auto path_point = path_points_in_ego_frame.begin() + nearest_index + 1;
       path_point != path_points_in_ego_frame.end(); ++path_point)
  {

    double delta_distance = getInterPosesDistance2D(*path_point, last_pose);
    forward_distance += delta_distance;
    // std::cout << "-------- distance: " << forward_distance << ",  "<< delta_distance <<std::endl;
    if (forward_distance > steb_config_.trajectory.max_global_plan_lookahead_dist)
      break ;
    // set the speed of reference trajectory for testing
    double speed_set = std::max<double>(std::min<double>(path_point->w(), steb_config_.trajectory.global_plan_velocity_set), 0.5);
    double delta_time = delta_distance / speed_set;
    forward_time += delta_time;
    Eigen::Vector4d p_w = {path_point->x(),
                           path_point->y(),
                           path_point->z(),
                           forward_time};
    via_points_.emplace_back(p_w);
    last_pose = *path_point;
  }

  std::cout << ">>>>> via_points size : " << via_points_.size() <<std::endl;


  // cost map process
  occ_map_ = path_ptr->drivable_area;

  tf2::Transform tf_map_to_map_origin;
  tf2::fromMsg(occ_map_.info.origin, tf_map_to_map_origin);
  tf2::Transform tf_ego_to_map_origin = tf_ego_to_path_frame * tf_map_to_map_origin;

  occ_map_.header.frame_id = "base_link";
  occ_map_.info.origin.position.x = tf_ego_to_map_origin.getOrigin().x();
  occ_map_.info.origin.position.y = tf_ego_to_map_origin.getOrigin().y();
  occ_map_.info.origin.position.z = tf_ego_to_map_origin.getOrigin().z();
  occ_map_.info.origin.orientation = tf2::toMsg(tf_ego_to_map_origin.getRotation());


  // run the steb
  std::cout << "==================== steb start ====================="<< std::endl;
  static steb_planner::TicToc steb_time_cost;
  steb_time_cost.tic();
  bool steb_success = false;
  {
    std::lock_guard<std::mutex> l(obst_mutex_);
    steb_planner_->setPridictedObject(*objects_ptr_);
    steb_planner_->setPath(*path_ptr);
    steb_planner_->setRvizPubTime(path_ptr->header.stamp);
    steb_success = steb_planner_->plan(tf_ego_to_path_frame.inverse(), via_points_);
  }

  auto teb_opt = steb_time_cost.toc();
  std::cout <<OUT_GREEN<< "[*********] steb time cost: " <<OUT_RESET<< teb_opt << " ms, " <<steb_success<<std::endl;
  std::cout << "====================  steb end  ====================="<< std::endl;

  // get the optimized trajectory
  static steb_planner::TicToc get_trajectory_time_cost;
  get_trajectory_time_cost.tic();
  steb_planner_->getFullTrajectory(steb_optimized_points_);
  auto get_trajectory_time = get_trajectory_time_cost.toc();
  std::cout <<OUT_GREEN<< "[*********] get_trajectory_time time cost: " <<OUT_RESET<< get_trajectory_time << " ms, " <<steb_success<<std::endl;

  autoware_auto_planning_msgs::msg::Trajectory trajectory_pub;
  trajectory_pub.header = path_ptr->header;

  for (size_t i=0; i<steb_optimized_points_.size()-1; ++i)
  {
    tf2::Transform optimized_point_in_base_link_frame;
    tf2::Vector3 origin{steb_optimized_points_.at(i).x(), steb_optimized_points_.at(i).y(), 0.0};
    optimized_point_in_base_link_frame.setOrigin(origin);
    const Eigen::Vector2d deltaS1{steb_optimized_points_.at(i+1).x() - steb_optimized_points_.at(i).x(),
                                  steb_optimized_points_.at(i+1).y() - steb_optimized_points_.at(i).y()};
    const double theta1 = std::atan2(deltaS1.y(), deltaS1.x());
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, theta1);
    optimized_point_in_base_link_frame.setRotation(quaternion);

    tf2::Transform optimized_point_in_path_frame = tf_ego_to_path_frame.inverse() * optimized_point_in_base_link_frame;

    autoware_auto_planning_msgs::msg::TrajectoryPoint trajectory_point;
    trajectory_point.pose.position.x = optimized_point_in_path_frame.getOrigin().x();
    trajectory_point.pose.position.y = optimized_point_in_path_frame.getOrigin().y();
    trajectory_point.pose.position.z = 0.0;

    trajectory_point.pose.orientation = tf2::toMsg(optimized_point_in_path_frame.getRotation());
    trajectory_point.longitudinal_velocity_mps =
        deltaS1.norm() / (steb_optimized_points_.at(i+1).w() - steb_optimized_points_.at(i).w());
    trajectory_point.rear_wheel_angle_rad = steb_optimized_points_.at(i).w();
    trajectory_pub.points.push_back(trajectory_point);
  }

  traj_pub_->publish(fromAutoTrajectory(trajectory_pub));


  // visualization the trajectory
  {
    visualization_msgs::msg::MarkerArray debug_markers;
    // obstacle
    visualization_msgs::msg::Marker obstacle_marker;
    obstacle_marker.header.frame_id = "map";
    obstacle_marker.header.stamp = trajectory_pub.header.stamp;
    obstacle_marker.color.a = 0.1;
    obstacle_marker.color.r = 0.0;
    obstacle_marker.color.g = 0.0;
    obstacle_marker.color.b = 1.0;
    obstacle_marker.type = visualization_msgs::msg::Marker::CUBE;
    obstacle_marker.action = visualization_msgs::msg::Marker::ADD;

    constexpr double visul_height = 0.4;
    obstacle_marker.scale.x = steb_config_.vehicle_param.length;
    obstacle_marker.scale.y = steb_config_.vehicle_param.width;
    obstacle_marker.scale.z = visul_height;

    //  for (int i=0; i<debug_trajectory_pub_id_; ++i)
    //  {
    //    obstacle_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    //    obstacle_marker.id = debug_trajectory_pub_id_++;
    //    debug_markers.markers.push_back(obstacle_marker);
    //  }
    obstacle_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    debug_markers.markers.push_back(obstacle_marker);

    debug_trajectory_pub_id_ = 300;
    obstacle_marker.action = visualization_msgs::msg::Marker::ADD;
    for (auto & traje:trajectory_pub.points)
    {
      tf2::Transform trajectory_point_in_map_frame;
      tf2::Vector3 origin{traje.pose.position.x, traje.pose.position.y, traje.pose.position.z + visul_height*0.5};
      trajectory_point_in_map_frame.setOrigin(origin);
      tf2::Quaternion quaternion;
      quaternion.setRPY(0.0, 0.0, tf2::getYaw(traje.pose.orientation));
      trajectory_point_in_map_frame.setRotation(quaternion);

      tf2::Transform vehicle_center_in_traje_frame = tf2::Transform::getIdentity();
      vehicle_center_in_traje_frame.getOrigin().setX(steb_config_.vehicle_param.rear_axle_2_center);

      tf2::Transform vehicle_in_path_frame = trajectory_point_in_map_frame * vehicle_center_in_traje_frame;

      obstacle_marker.id = debug_trajectory_pub_id_++;
      tf2::toMsg(vehicle_in_path_frame, obstacle_marker.pose);
      debug_markers.markers.push_back(obstacle_marker);
    }

    debug_trajectory_pub_->publish(debug_markers);
  }



  // pub the debug costmap
  steb_planner::CVMaps cv_maps = steb_planner_->getCVMaps();
  sensor_msgs::msg::Image costmap_image;
  costmap_image.header = path_ptr->header;
  cv::Mat costmap_image_cv = cv_maps.drivable_with_objects_clearance_map;

  cv_bridge::CvImage cvi;
  cvi.header = costmap_image.header;
  // clearance map using 32FC1; drivable map using mono8
//    cvi.encoding = "mono8";
  cvi.encoding = "32FC1";
  cvi.image = costmap_image_cv;
  cvi.toImageMsg(costmap_image);
  debug_costmap_image_pub_->publish(costmap_image);

  // pub the origin clearance map
  sensor_msgs::msg::Image costmap_image_origin;
  cv::Mat costmap_origin_image_cv = cv_maps.drivable_with_objects_grid_map;
  cv_bridge::CvImage cvi_origin;
  cvi_origin.header = costmap_image.header;
  // clearance map using 32FC1; drivable map using mono8
  cvi.encoding = "mono8";
  // cvi_origin.encoding = "32FC1";
  cvi_origin.image = costmap_origin_image_cv;
  cvi_origin.toImageMsg(costmap_image_origin);
  debug_costmap_origin_image_pub_->publish(costmap_image_origin);

  // publish the occ costmap
  debug_occ_map_pub_->publish(occ_map_);


  std_msgs::msg::Float32 msg;
  msg.data = teb_opt;
  exe_time_pub_->publish(msg);

  publishDebugMarker(path_ptr->header.stamp);

}




void STEBPlannerNode::publishDebugMarker(const rclcpp::Time& time)
{
  visualization_msgs::msg::MarkerArray debug_markers;

  // obstalce
  visualization_msgs::msg::Marker obstacle_marker;
  obstacle_marker.header.frame_id = "base_link";
  obstacle_marker.header.stamp = time;
  obstacle_marker.color.a = 1.0;
  obstacle_marker.color.r = 1.0;
  obstacle_marker.color.g = 0.0;
  obstacle_marker.color.b = 0.0;
  obstacle_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  obstacle_marker.scale.x = 0.3;
  obstacle_marker.scale.y = 0.3;
  obstacle_marker.scale.z = 0.3;
  obstacle_marker.id = 0;
  obstacle_marker.pose = geometry_msgs::msg::Pose();

  for (auto& obstacle : dynamic_obst_vector_)
  {
    geometry_msgs::msg::Point st_position;
    st_position.x = obstacle->getCentroid().x();
    st_position.y = obstacle->getCentroid().y();
    st_position.z = obstacle->getEmergenceTime();
    obstacle_marker.points.push_back(st_position);
  }
//  vehicle_marker.action = visualization_msgs::msg::Marker::DELETE;
//  obstacle_markers.markers.push_back(vehicle_marker);
  obstacle_marker.action = visualization_msgs::msg::Marker::ADD;
  debug_markers.markers.push_back(obstacle_marker);


  // via points
  visualization_msgs::msg::Marker via_point_marker;
  via_point_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  via_point_marker.header.frame_id = "base_link";
  via_point_marker.header.stamp = time;
  via_point_marker.color.a = 0.2;
  via_point_marker.color.r = 0.0;
  via_point_marker.color.g = 1.0;
  via_point_marker.color.b = 0.0;
  via_point_marker.scale.x = 0.5;
  via_point_marker.scale.y = 0.5;
  via_point_marker.scale.z = 0.5;
  via_point_marker.id = 1;

  for (auto & via_point : via_points_)
  {
    geometry_msgs::msg::Point point;
    point.x = via_point.x();
    point.y = via_point.y();
    point.z = via_point.w();
    via_point_marker.points.push_back(point);
//    std::cout << via_point.x() << ", "
//              << via_point.y() << ", "
//              << via_point.w() << std::endl;
  }

  via_point_marker.action = visualization_msgs::msg::Marker::ADD;
  debug_markers.markers.push_back(via_point_marker);



  // optimized points
  visualization_msgs::msg::Marker optimized_point_marker;
  optimized_point_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  optimized_point_marker.action = visualization_msgs::msg::Marker::ADD;
  optimized_point_marker.header.frame_id = "base_link";
  optimized_point_marker.header.stamp = time;
  optimized_point_marker.color.a = 0.5;
  optimized_point_marker.color.r = 0.0;
  optimized_point_marker.color.g = 1.0;
  optimized_point_marker.color.b = 0.0;
  optimized_point_marker.scale.x = 0.4;
  optimized_point_marker.scale.y = 0.4;
  optimized_point_marker.scale.z = 0.4;
  optimized_point_marker.id = 2;
  optimized_point_marker.pose = geometry_msgs::msg::Pose();


  visualization_msgs::msg::Marker path_marker;
  path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  path_marker.action = visualization_msgs::msg::Marker::ADD;
  path_marker.header.frame_id = "base_link";
  path_marker.header.stamp = time;
  path_marker.color.a = 0.5;
  path_marker.color.r = 0.0;
  path_marker.color.g = 1.0;
  path_marker.color.b = 0.0;
  path_marker.scale.x = 0.1;
  path_marker.scale.y = 0.1;
  path_marker.scale.z = 0.1;
  path_marker.id = 3;

  std::cout << ">>>>> STEBOptimizedPoints size : " << steb_optimized_points_.size() <<std::endl;
  for (auto& point : steb_optimized_points_)
  {
    geometry_msgs::msg::Point optimized_point;
    optimized_point.x = point.x();
    optimized_point.y = point.y();
    optimized_point.z = point.w();
    optimized_point_marker.points.push_back(optimized_point);
    path_marker.points.push_back(optimized_point);
//    std::cout << point.x() << ", "
//              << point.y() << ", "
//              << point.w() << std::endl;
  }
  debug_markers.markers.push_back(optimized_point_marker);
  debug_markers.markers.push_back(path_marker);


  debug_viz_pub_->publish(debug_markers);
}


int main (int argc, char ** argv)
{
  std::cout << " come on ! bro !" << std::endl;

  steb_planner::TicToc time_cost;
  time_cost.tic();

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;

  auto time = time_cost.toc();
  std::cout << "*********** node init cost: " << time << " ms " << std::endl;

  auto node = std::make_shared<STEBPlannerNode>(node_options);
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node);
  executor->spin();
  rclcpp::shutdown();

  return 0;
}