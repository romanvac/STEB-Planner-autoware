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

#include "steb_planner/steb_optimal/steb_config.h"

namespace steb_planner
{

void STEBConfig::declareParameters(rclcpp::Node * nh)
{
  // vehicle parameter
  declare_parameter_if_not_declared(
    nh, node_name + ".VehicleParam." + "width", rclcpp::ParameterValue(vehicle_param.width));
  declare_parameter_if_not_declared(
    nh, node_name + ".VehicleParam." + "length", rclcpp::ParameterValue(vehicle_param.length));
  declare_parameter_if_not_declared(
    nh, node_name + ".VehicleParam." + "height", rclcpp::ParameterValue(vehicle_param.height));
  declare_parameter_if_not_declared(
    nh, node_name + ".VehicleParam." + "front_suspension",
    rclcpp::ParameterValue(vehicle_param.front_suspension));
  declare_parameter_if_not_declared(
    nh, node_name + ".VehicleParam." + "rear_suspension",
    rclcpp::ParameterValue(vehicle_param.rear_suspension));

  declare_parameter_if_not_declared(
    nh, node_name + ".VehicleParam." + "max_steering_angle",
    rclcpp::ParameterValue(vehicle_param.max_steering_angle));
  declare_parameter_if_not_declared(
    nh, node_name + ".VehicleParam." + "min_turning_radius",
    rclcpp::ParameterValue(vehicle_param.min_turning_radius));
  declare_parameter_if_not_declared(
    nh, node_name + ".VehicleParam." + "wheelbase",
    rclcpp::ParameterValue(vehicle_param.wheel_base));
  declare_parameter_if_not_declared(
    nh, node_name + ".VehicleParam." + "rear_axle_2_center",
    rclcpp::ParameterValue(vehicle_param.rear_axle_2_center));

  declare_parameter_if_not_declared(
    nh, node_name + ".VehicleParam." + "max_vel_x",
    rclcpp::ParameterValue(vehicle_param.max_vel_x));
  declare_parameter_if_not_declared(
    nh, node_name + ".VehicleParam." + "max_vel_x_backwards",
    rclcpp::ParameterValue(vehicle_param.max_vel_x_backwards));
  declare_parameter_if_not_declared(
    nh, node_name + ".VehicleParam." + "max_vel_theta",
    rclcpp::ParameterValue(vehicle_param.max_vel_theta));
  declare_parameter_if_not_declared(
    nh, node_name + ".VehicleParam." + "max_longitudinal_acc",
    rclcpp::ParameterValue(vehicle_param.max_longitudinal_acc));
  declare_parameter_if_not_declared(
    nh, node_name + ".VehicleParam." + "max_lateral_acc",
    rclcpp::ParameterValue(vehicle_param.max_lateral_acc));
  declare_parameter_if_not_declared(
    nh, node_name + ".VehicleParam." + "max_theta_acc",
    rclcpp::ParameterValue(vehicle_param.max_theta_acc));

  declare_parameter_if_not_declared(
    nh, node_name + ".VehicleParam." + "circle_radius",
    rclcpp::ParameterValue(vehicle_param.circle_radius));

  // Trajectory
  declare_parameter_if_not_declared(
    nh, node_name + ".Trajectory." + "steb_autosize",
    rclcpp::ParameterValue(trajectory.steb_autosize));
  declare_parameter_if_not_declared(
    nh, node_name + ".Trajectory." + "auto_resize_resolution",
    rclcpp::ParameterValue(trajectory.auto_resize_resolution));
  declare_parameter_if_not_declared(
    nh, node_name + ".Trajectory." + "auto_resize_resolution_hysteresis",
    rclcpp::ParameterValue(trajectory.auto_resize_resolution_hysteresis));
  declare_parameter_if_not_declared(
    nh, node_name + ".Trajectory." + "min_samples", rclcpp::ParameterValue(trajectory.min_samples));
  declare_parameter_if_not_declared(
    nh, node_name + ".Trajectory." + "max_samples", rclcpp::ParameterValue(trajectory.max_samples));
  declare_parameter_if_not_declared(
    nh, node_name + ".Trajectory." + "max_global_plan_lookahead_dist",
    rclcpp::ParameterValue(trajectory.max_global_plan_lookahead_dist));
  declare_parameter_if_not_declared(
    nh, node_name + ".Trajectory." + "current_velocity_for_debug",
    rclcpp::ParameterValue(trajectory.current_velocity_for_debug));
  declare_parameter_if_not_declared(
    nh, node_name + ".Trajectory." + "global_plan_velocity_set",
    rclcpp::ParameterValue(trajectory.global_plan_velocity_set));
  declare_parameter_if_not_declared(
    nh, node_name + ".Trajectory." + "exact_arc_length",
    rclcpp::ParameterValue(trajectory.exact_arc_length));
  declare_parameter_if_not_declared(
    nh, node_name + ".Trajectory." + "force_reinit_new_goal_dist",
    rclcpp::ParameterValue(trajectory.force_reinit_new_goal_dist));
  declare_parameter_if_not_declared(
    nh, node_name + ".Trajectory." + "force_reinit_new_goal_angular",
    rclcpp::ParameterValue(trajectory.force_reinit_new_goal_angular));
  declare_parameter_if_not_declared(
    nh, node_name + ".Trajectory." + "free_goal_vel",
    rclcpp::ParameterValue(trajectory.free_goal_vel));

  // Obstacles
  declare_parameter_if_not_declared(
    nh, node_name + ".Obstacles." + "min_obstacle_dist",
    rclcpp::ParameterValue(obstacles.min_obstacle_dist));
  declare_parameter_if_not_declared(
    nh, node_name + ".Obstacles." + "inflation_dist",
    rclcpp::ParameterValue(obstacles.inflation_dist));
  declare_parameter_if_not_declared(
    nh, node_name + ".Obstacles." + "dynamic_obstacle_inflation_dist",
    rclcpp::ParameterValue(obstacles.dynamic_obstacle_inflation_dist));
  declare_parameter_if_not_declared(
    nh, node_name + ".Obstacles." + "include_dynamic_obstacles",
    rclcpp::ParameterValue(obstacles.include_dynamic_obstacles));

  declare_parameter_if_not_declared(
    nh, node_name + ".Obstacles." + "obstacle_poses_affected",
    rclcpp::ParameterValue(obstacles.obstacle_poses_affected));
  declare_parameter_if_not_declared(
    nh, node_name + ".Obstacles." + "obstacle_association_force_inclusion_factor",
    rclcpp::ParameterValue(obstacles.obstacle_association_force_inclusion_factor));

  // Optimization
  declare_parameter_if_not_declared(
    nh, node_name + ".Optimization." + "no_inner_iterations",
    rclcpp::ParameterValue(optim.no_inner_iterations));
  declare_parameter_if_not_declared(
    nh, node_name + ".Optimization." + "no_outer_iterations",
    rclcpp::ParameterValue(optim.no_outer_iterations));
  declare_parameter_if_not_declared(
    nh, node_name + ".Optimization." + "optimization_activate",
    rclcpp::ParameterValue(optim.optimization_activate));
  declare_parameter_if_not_declared(
    nh, node_name + ".Optimization." + "optimization_verbose",
    rclcpp::ParameterValue(optim.optimization_verbose));
  declare_parameter_if_not_declared(
    nh, node_name + ".Optimization." + "penalty_epsilon",
    rclcpp::ParameterValue(optim.penalty_epsilon));
  declare_parameter_if_not_declared(
    nh, node_name + ".Optimization." + "weight_vel_x_limit",
    rclcpp::ParameterValue(optim.weight_vel_x_limit));
  declare_parameter_if_not_declared(
    nh, node_name + ".Optimization." + "weight_vel_theta_limit",
    rclcpp::ParameterValue(optim.weight_vel_theta_limit));
  declare_parameter_if_not_declared(
    nh, node_name + ".Optimization." + "weight_acc_x_limit",
    rclcpp::ParameterValue(optim.weight_acc_x_limit));
  declare_parameter_if_not_declared(
    nh, node_name + ".Optimization." + "weight_acc_theta_limit",
    rclcpp::ParameterValue(optim.weight_acc_theta_limit));
  declare_parameter_if_not_declared(
    nh, node_name + ".Optimization." + "weight_kinematics_nh",
    rclcpp::ParameterValue(optim.weight_kinematics_nh));
  declare_parameter_if_not_declared(
    nh, node_name + ".Optimization." + "weight_kinematics_turning_radius_limit",
    rclcpp::ParameterValue(optim.weight_kinematics_turning_radius_limit));
  declare_parameter_if_not_declared(
    nh, node_name + ".Optimization." + "weight_time_minimize",
    rclcpp::ParameterValue(optim.weight_time_minimize));
  declare_parameter_if_not_declared(
    nh, node_name + ".Optimization." + "weight_shortest_path",
    rclcpp::ParameterValue(optim.weight_shortest_path));
  declare_parameter_if_not_declared(
    nh, node_name + ".Optimization." + "weight_obstacle_free_limit",
    rclcpp::ParameterValue(optim.weight_obstacle_free_limit));
  declare_parameter_if_not_declared(
    nh, node_name + ".Optimization." + "weight_obstacle_of_dynamic_inflation",
    rclcpp::ParameterValue(optim.weight_obstacle_of_dynamic_inflation));
  declare_parameter_if_not_declared(
    nh, node_name + ".Optimization." + "weight_viapoint",
    rclcpp::ParameterValue(optim.weight_viapoint));
  declare_parameter_if_not_declared(
    nh, node_name + ".Optimization." + "weight_adapt_factor",
    rclcpp::ParameterValue(optim.weight_adapt_factor));

  declare_parameter_if_not_declared(
    nh, node_name + ".Optimization." + "weight_jerk_minimize",
    rclcpp::ParameterValue(optim.weight_jerk_minimize));
  declare_parameter_if_not_declared(
    nh, node_name + ".Optimization." + "weight_acc_theta_minimize",
    rclcpp::ParameterValue(optim.weight_acc_theta_minimize));
  declare_parameter_if_not_declared(
    nh, node_name + ".Optimization." + "weight_time_ascend_limit",
    rclcpp::ParameterValue(optim.weight_time_ascend_limit));
  declare_parameter_if_not_declared(
    nh, node_name + ".Optimization." + "weight_via_area_base",
    rclcpp::ParameterValue(optim.weight_via_area_base));
  declare_parameter_if_not_declared(
    nh, node_name + ".Optimization." + "weight_via_area_middle",
    rclcpp::ParameterValue(optim.weight_via_area_middle));
  declare_parameter_if_not_declared(
    nh, node_name + ".Optimization." + "weight_via_area_front",
    rclcpp::ParameterValue(optim.weight_via_area_front));
  declare_parameter_if_not_declared(
    nh, node_name + ".Optimization." + "weight_start_points_velocity",
    rclcpp::ParameterValue(optim.weight_start_points_velocity));
  declare_parameter_if_not_declared(
    nh, node_name + ".Optimization." + "weight_start_points_direction",
    rclcpp::ParameterValue(optim.weight_start_points_direction));
  declare_parameter_if_not_declared(
    nh, node_name + ".Optimization." + "weight_start_points_interval_distance",
    rclcpp::ParameterValue(optim.weight_start_points_interval_distance));
  declare_parameter_if_not_declared(
    nh, node_name + ".Optimization." + "weight_final_goal_distance",
    rclcpp::ParameterValue(optim.weight_final_goal_distance));
  declare_parameter_if_not_declared(
    nh, node_name + ".Optimization." + "weight_final_goal_yaw",
    rclcpp::ParameterValue(optim.weight_final_goal_yaw));
  declare_parameter_if_not_declared(
    nh, node_name + ".Optimization." + "weight_final_goal_time",
    rclcpp::ParameterValue(optim.weight_final_goal_time));

  declare_parameter_if_not_declared(
    nh, node_name + ".Optimization." + "divergence_detection_enable",
    rclcpp::ParameterValue(optim.divergence_detection_enable));
  declare_parameter_if_not_declared(
    nh, node_name + ".Optimization." + "divergence_detection_max_chi_squared",
    rclcpp::ParameterValue(optim.divergence_detection_max_chi_squared));

  // CollisionFreeCorridor
  declare_parameter_if_not_declared(
    nh, node_name + ".CollisionFreeCorridor." + "is_avoiding_unknown",
    rclcpp::ParameterValue(collision_free_corridor.is_avoiding_unknown));
  declare_parameter_if_not_declared(
    nh, node_name + ".CollisionFreeCorridor." + "is_avoiding_car",
    rclcpp::ParameterValue(collision_free_corridor.is_avoiding_car));
  declare_parameter_if_not_declared(
    nh, node_name + ".CollisionFreeCorridor." + "is_avoiding_truck",
    rclcpp::ParameterValue(collision_free_corridor.is_avoiding_truck));
  declare_parameter_if_not_declared(
    nh, node_name + ".CollisionFreeCorridor." + "is_avoiding_bus",
    rclcpp::ParameterValue(collision_free_corridor.is_avoiding_bus));
  declare_parameter_if_not_declared(
    nh, node_name + ".CollisionFreeCorridor." + "is_avoiding_bicycle",
    rclcpp::ParameterValue(collision_free_corridor.is_avoiding_bicycle));
  declare_parameter_if_not_declared(
    nh, node_name + ".CollisionFreeCorridor." + "is_avoiding_motorbike",
    rclcpp::ParameterValue(collision_free_corridor.is_avoiding_motorbike));
  declare_parameter_if_not_declared(
    nh, node_name + ".CollisionFreeCorridor." + "is_avoiding_pedestrian",
    rclcpp::ParameterValue(collision_free_corridor.is_avoiding_pedestrian));
  declare_parameter_if_not_declared(
    nh, node_name + ".CollisionFreeCorridor." + "is_avoiding_animal",
    rclcpp::ParameterValue(collision_free_corridor.is_avoiding_animal));

  declare_parameter_if_not_declared(
    nh, node_name + ".CollisionFreeCorridor." + "soft_clearance_from_road",
    rclcpp::ParameterValue(collision_free_corridor.soft_clearance_from_road));
  declare_parameter_if_not_declared(
    nh, node_name + ".CollisionFreeCorridor." + "extra_desired_clearance_from_road",
    rclcpp::ParameterValue(collision_free_corridor.extra_desired_clearance_from_road));
  declare_parameter_if_not_declared(
    nh, node_name + ".CollisionFreeCorridor." + "soft_clearance_from_object",
    rclcpp::ParameterValue(collision_free_corridor.soft_clearance_from_object));
  declare_parameter_if_not_declared(
    nh, node_name + ".CollisionFreeCorridor." + "extra_desired_clearance_from_object",
    rclcpp::ParameterValue(collision_free_corridor.extra_desired_clearance_from_object));
  declare_parameter_if_not_declared(
    nh, node_name + ".CollisionFreeCorridor." + "max_bound_search_width",
    rclcpp::ParameterValue(collision_free_corridor.max_bound_search_width));
}

void STEBConfig::loadRosParamFromNodeHandle(rclcpp::Node * nh)
{
  // Trajectory
  nh->get_parameter_or(
    node_name + ".Trajectory." + "steb_autosize", trajectory.steb_autosize,
    trajectory.steb_autosize);
  nh->get_parameter_or(
    node_name + ".Trajectory." + "auto_resize_resolution", trajectory.auto_resize_resolution,
    trajectory.auto_resize_resolution);
  nh->get_parameter_or(
    node_name + ".Trajectory." + "auto_resize_resolution_hysteresis",
    trajectory.auto_resize_resolution_hysteresis, trajectory.auto_resize_resolution_hysteresis);
  nh->get_parameter_or(
    node_name + ".Trajectory." + "min_samples", trajectory.min_samples, trajectory.min_samples);
  nh->get_parameter_or(
    node_name + ".Trajectory." + "max_samples", trajectory.max_samples, trajectory.max_samples);
  nh->get_parameter_or(
    node_name + ".Trajectory." + "max_global_plan_lookahead_dist",
    trajectory.max_global_plan_lookahead_dist, trajectory.max_global_plan_lookahead_dist);
  nh->get_parameter_or(
    node_name + ".Trajectory." + "current_velocity_for_debug",
    trajectory.current_velocity_for_debug, trajectory.current_velocity_for_debug);
  nh->get_parameter_or(
    node_name + ".Trajectory." + "global_plan_velocity_set", trajectory.global_plan_velocity_set,
    trajectory.global_plan_velocity_set);
  nh->get_parameter_or(
    node_name + ".Trajectory." + "exact_arc_length", trajectory.exact_arc_length,
    trajectory.exact_arc_length);
  nh->get_parameter_or(
    node_name + ".Trajectory." + "force_reinit_new_goal_dist",
    trajectory.force_reinit_new_goal_dist, trajectory.force_reinit_new_goal_dist);
  nh->get_parameter_or(
    node_name + ".Trajectory." + "force_reinit_new_goal_angular",
    trajectory.force_reinit_new_goal_angular, trajectory.force_reinit_new_goal_angular);

  // Vehicle
  nh->get_parameter_or(
    node_name + ".VehicleParam." + "width", vehicle_param.width, vehicle_param.width);
  nh->get_parameter_or(
    node_name + ".VehicleParam." + "length", vehicle_param.length, vehicle_param.length);
  nh->get_parameter_or(
    node_name + ".VehicleParam." + "height", vehicle_param.height, vehicle_param.height);
  nh->get_parameter_or(
    node_name + ".VehicleParam." + "front_suspension", vehicle_param.front_suspension,
    vehicle_param.front_suspension);
  nh->get_parameter_or(
    node_name + ".VehicleParam." + "rear_suspension", vehicle_param.rear_suspension,
    vehicle_param.rear_suspension);

  nh->get_parameter_or(
    node_name + ".VehicleParam." + "max_steering_angle", vehicle_param.max_steering_angle,
    vehicle_param.max_steering_angle);
  nh->get_parameter_or(
    node_name + ".VehicleParam." + "min_turning_radius", vehicle_param.min_turning_radius,
    vehicle_param.min_turning_radius);
  nh->get_parameter_or(
    node_name + ".VehicleParam." + "wheelbase", vehicle_param.wheel_base, vehicle_param.wheel_base);
  nh->get_parameter_or(
    node_name + ".VehicleParam." + "rear_axle_2_center", vehicle_param.rear_axle_2_center,
    vehicle_param.rear_axle_2_center);

  nh->get_parameter_or(
    node_name + ".VehicleParam." + "max_vel_x", vehicle_param.max_vel_x, vehicle_param.max_vel_x);
  nh->get_parameter_or(
    node_name + ".VehicleParam." + "max_vel_x_backwards", vehicle_param.max_vel_x_backwards,
    vehicle_param.max_vel_x_backwards);
  nh->get_parameter_or(
    node_name + ".VehicleParam." + "max_vel_theta", vehicle_param.max_vel_theta,
    vehicle_param.max_vel_theta);
  nh->get_parameter_or(
    node_name + ".VehicleParam." + "max_longitudinal_acc", vehicle_param.max_longitudinal_acc,
    vehicle_param.max_longitudinal_acc);

  nh->get_parameter_or(
    node_name + ".VehicleParam." + "max_lateral_acc", vehicle_param.max_lateral_acc,
    vehicle_param.max_lateral_acc);
  nh->get_parameter_or(
    node_name + ".VehicleParam." + "max_theta_acc", vehicle_param.max_theta_acc,
    vehicle_param.max_theta_acc);
  nh->get_parameter_or(
    node_name + ".VehicleParam." + "circle_radius", vehicle_param.circle_radius,
    vehicle_param.circle_radius);

  // Obstacles
  nh->get_parameter_or(
    node_name + ".Obstacles." + "min_obstacle_dist", obstacles.min_obstacle_dist,
    obstacles.min_obstacle_dist);
  nh->get_parameter_or(
    node_name + ".Obstacles." + "inflation_dist", obstacles.inflation_dist,
    obstacles.inflation_dist);
  nh->get_parameter_or(
    node_name + ".Obstacles." + "dynamic_obstacle_inflation_dist",
    obstacles.dynamic_obstacle_inflation_dist, obstacles.dynamic_obstacle_inflation_dist);
  nh->get_parameter_or(
    node_name + ".Obstacles." + "include_dynamic_obstacles", obstacles.include_dynamic_obstacles,
    obstacles.include_dynamic_obstacles);
  nh->get_parameter_or(
    node_name + ".Obstacles." + "obstacle_poses_affected", obstacles.obstacle_poses_affected,
    obstacles.obstacle_poses_affected);
  nh->get_parameter_or(
    node_name + ".Obstacles." + "obstacle_association_force_inclusion_factor",
    obstacles.obstacle_association_force_inclusion_factor,
    obstacles.obstacle_association_force_inclusion_factor);

  // Optimization
  nh->get_parameter_or(
    node_name + ".Optimization." + "no_inner_iterations", optim.no_inner_iterations,
    optim.no_inner_iterations);
  nh->get_parameter_or(
    node_name + ".Optimization." + "no_outer_iterations", optim.no_outer_iterations,
    optim.no_outer_iterations);
  nh->get_parameter_or(
    node_name + ".Optimization." + "optimization_activate", optim.optimization_activate,
    optim.optimization_activate);
  nh->get_parameter_or(
    node_name + ".Optimization." + "optimization_verbose", optim.optimization_verbose,
    optim.optimization_verbose);
  nh->get_parameter_or(
    node_name + ".Optimization." + "penalty_epsilon", optim.penalty_epsilon, optim.penalty_epsilon);
  nh->get_parameter_or(
    node_name + ".Optimization." + "weight_vel_x_limit", optim.weight_vel_x_limit,
    optim.weight_vel_x_limit);
  nh->get_parameter_or(
    node_name + ".Optimization." + "weight_vel_theta_limit", optim.weight_vel_theta_limit,
    optim.weight_vel_theta_limit);
  nh->get_parameter_or(
    node_name + ".Optimization." + "weight_acc_x_limit", optim.weight_acc_x_limit,
    optim.weight_acc_x_limit);
  nh->get_parameter_or(
    node_name + ".Optimization." + "weight_acc_theta_limit", optim.weight_acc_theta_limit,
    optim.weight_acc_theta_limit);
  nh->get_parameter_or(
    node_name + ".Optimization." + "weight_kinematics_nh", optim.weight_kinematics_nh,
    optim.weight_kinematics_nh);
  nh->get_parameter_or(
    node_name + ".Optimization." + "weight_kinematics_turning_radius_limit",
    optim.weight_kinematics_turning_radius_limit, optim.weight_kinematics_turning_radius_limit);
  nh->get_parameter_or(
    node_name + ".Optimization." + "weight_time_minimize", optim.weight_time_minimize,
    optim.weight_time_minimize);
  nh->get_parameter_or(
    node_name + ".Optimization." + "weight_shortest_path", optim.weight_shortest_path,
    optim.weight_shortest_path);
  nh->get_parameter_or(
    node_name + ".Optimization." + "weight_obstacle_free_limit", optim.weight_obstacle_free_limit,
    optim.weight_obstacle_free_limit);
  nh->get_parameter_or(
    node_name + ".Optimization." + "weight_obstacle_of_dynamic_inflation",
    optim.weight_obstacle_of_dynamic_inflation, optim.weight_obstacle_of_dynamic_inflation);
  nh->get_parameter_or(
    node_name + ".Optimization." + "weight_viapoint", optim.weight_viapoint, optim.weight_viapoint);
  nh->get_parameter_or(
    node_name + ".Optimization." + "weight_adapt_factor", optim.weight_adapt_factor,
    optim.weight_adapt_factor);

  nh->get_parameter_or(
    node_name + ".Optimization." + "weight_jerk_minimize", optim.weight_jerk_minimize,
    optim.weight_jerk_minimize);
  nh->get_parameter_or(
    node_name + ".Optimization." + "weight_acc_theta_minimize", optim.weight_acc_theta_minimize,
    optim.weight_acc_theta_minimize);
  nh->get_parameter_or(
    node_name + ".Optimization." + "weight_time_ascend_limit", optim.weight_time_ascend_limit,
    optim.weight_time_ascend_limit);
  nh->get_parameter_or(
    node_name + ".Optimization." + "weight_via_area_base", optim.weight_via_area_base,
    optim.weight_via_area_base);
  nh->get_parameter_or(
    node_name + ".Optimization." + "weight_via_area_middle", optim.weight_via_area_middle,
    optim.weight_via_area_middle);
  nh->get_parameter_or(
    node_name + ".Optimization." + "weight_via_area_front", optim.weight_via_area_front,
    optim.weight_via_area_front);
  nh->get_parameter_or(
    node_name + ".Optimization." + "weight_start_points_velocity",
    optim.weight_start_points_velocity, optim.weight_start_points_velocity);
  nh->get_parameter_or(
    node_name + ".Optimization." + "weight_start_points_direction",
    optim.weight_start_points_direction, optim.weight_start_points_direction);
  nh->get_parameter_or(
    node_name + ".Optimization." + "weight_start_points_interval_distance",
    optim.weight_start_points_interval_distance, optim.weight_start_points_interval_distance);
  nh->get_parameter_or(
    node_name + ".Optimization." + "weight_final_goal_distance", optim.weight_final_goal_distance,
    optim.weight_final_goal_distance);
  nh->get_parameter_or(
    node_name + ".Optimization." + "weight_final_goal_yaw", optim.weight_final_goal_yaw,
    optim.weight_final_goal_yaw);
  nh->get_parameter_or(
    node_name + ".Optimization." + "weight_final_goal_time", optim.weight_final_goal_time,
    optim.weight_final_goal_time);

  nh->get_parameter_or(
    node_name + ".Optimization." + "divergence_detection_enable", optim.divergence_detection_enable,
    optim.divergence_detection_enable);
  nh->get_parameter_or(
    node_name + ".Optimization." + "divergence_detection_max_chi_squared",
    optim.divergence_detection_max_chi_squared, optim.divergence_detection_max_chi_squared);

  // CollisionFreeCorridor
  nh->get_parameter_or(
    node_name + ".CollisionFreeCorridor." + "is_avoiding_unknown",
    collision_free_corridor.is_avoiding_unknown, collision_free_corridor.is_avoiding_unknown);
  nh->get_parameter_or(
    node_name + ".CollisionFreeCorridor." + "is_avoiding_car",
    collision_free_corridor.is_avoiding_car, collision_free_corridor.is_avoiding_car);
  nh->get_parameter_or(
    node_name + ".CollisionFreeCorridor." + "is_avoiding_truck",
    collision_free_corridor.is_avoiding_truck, collision_free_corridor.is_avoiding_truck);
  nh->get_parameter_or(
    node_name + ".CollisionFreeCorridor." + "is_avoiding_bus",
    collision_free_corridor.is_avoiding_bus, collision_free_corridor.is_avoiding_bus);
  nh->get_parameter_or(
    node_name + ".CollisionFreeCorridor." + "is_avoiding_bicycle",
    collision_free_corridor.is_avoiding_bicycle, collision_free_corridor.is_avoiding_bicycle);
  nh->get_parameter_or(
    node_name + ".CollisionFreeCorridor." + "is_avoiding_motorbike",
    collision_free_corridor.is_avoiding_motorbike, collision_free_corridor.is_avoiding_motorbike);
  nh->get_parameter_or(
    node_name + ".CollisionFreeCorridor." + "is_avoiding_pedestrian",
    collision_free_corridor.is_avoiding_pedestrian, collision_free_corridor.is_avoiding_pedestrian);
  nh->get_parameter_or(
    node_name + ".CollisionFreeCorridor." + "is_avoiding_animal",
    collision_free_corridor.is_avoiding_animal, collision_free_corridor.is_avoiding_animal);

  nh->get_parameter_or(
    node_name + ".CollisionFreeCorridor." + "soft_clearance_from_road",
    collision_free_corridor.soft_clearance_from_road,
    collision_free_corridor.soft_clearance_from_road);
  nh->get_parameter_or(
    node_name + ".CollisionFreeCorridor." + "extra_desired_clearance_from_road",
    collision_free_corridor.extra_desired_clearance_from_road,
    collision_free_corridor.extra_desired_clearance_from_road);
  nh->get_parameter_or(
    node_name + ".CollisionFreeCorridor." + "soft_clearance_from_object",
    collision_free_corridor.soft_clearance_from_object,
    collision_free_corridor.soft_clearance_from_object);
  nh->get_parameter_or(
    node_name + ".CollisionFreeCorridor." + "extra_desired_clearance_from_object",
    collision_free_corridor.extra_desired_clearance_from_object,
    collision_free_corridor.extra_desired_clearance_from_object);
  nh->get_parameter_or(
    node_name + ".CollisionFreeCorridor." + "max_bound_search_width",
    collision_free_corridor.max_bound_search_width, collision_free_corridor.max_bound_search_width);

  checkParameters();
  checkDeprecated(nh);
}

void STEBConfig::on_parameter_event_callback(
  const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
  std::lock_guard<std::mutex> l(config_mutex_);
  std::cout << "\n\n\n\n\n\n     dynamic parameter     \n\n\n\n\n\n " << std::endl;
  for (auto & changed_parameter : event->changed_parameters) {
    const auto & type = changed_parameter.value.type;
    const auto & name = changed_parameter.name;
    const auto & value = changed_parameter.value;
    std::cout << "-type: " << changed_parameter.value.type << std::endl;
    std::cout << "-name: " << name << std::endl;
    std::cout << "-value: " << value.double_value << std::endl;
    if (type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE) {
      // Trajectory
      if (name == node_name + ".Trajectory.steb_autosize") {
        trajectory.steb_autosize = value.double_value;
      } else if (name == node_name + ".Trajectory.auto_resize_resolution") {
        trajectory.auto_resize_resolution = value.double_value;
      } else if (name == node_name + ".Trajectory.auto_resize_resolution_hysteresis") {
        trajectory.auto_resize_resolution_hysteresis = value.double_value;
      } else if (name == node_name + ".Trajectory.max_global_plan_lookahead_dist") {
        trajectory.max_global_plan_lookahead_dist = value.double_value;
      } else if (name == node_name + ".Trajectory.global_plan_velocity_set") {
        trajectory.global_plan_velocity_set = value.double_value;
      } else if (name == node_name + ".Trajectory.force_reinit_new_goal_dist") {
        trajectory.force_reinit_new_goal_dist = value.double_value;
      } else if (name == node_name + ".Trajectory.force_reinit_new_goal_angular") {
        trajectory.force_reinit_new_goal_angular = value.double_value;
      } else if (name == node_name + ".Trajectory.current_velocity_for_debug") {
        trajectory.current_velocity_for_debug = value.double_value;
      }

      // Vehicle
      else if (name == node_name + ".Vehicle_param.max_vel_x") {
        vehicle_param.max_vel_x = value.double_value;
      } else if (name == node_name + ".Vehicle_param.max_vel_x_backwards") {
        vehicle_param.max_vel_x_backwards = value.double_value;
      } else if (name == node_name + ".Vehicle_param.max_vel_theta") {
        vehicle_param.max_vel_theta = value.double_value;
      } else if (name == node_name + ".Vehicle_param.acc_lim_x") {
        vehicle_param.max_longitudinal_acc = value.double_value;
      } else if (name == node_name + ".Vehicle_param.max_lateral_acc") {
        vehicle_param.max_lateral_acc = value.double_value;
      } else if (name == node_name + ".Vehicle_param.max_theta_acc") {
        vehicle_param.max_theta_acc = value.double_value;
      } else if (name == node_name + ".Vehicle_param.min_turning_radius") {
        vehicle_param.min_turning_radius = value.double_value;
      } else if (name == node_name + ".Vehicle_param.wheel_base") {
        vehicle_param.wheel_base = value.double_value;
      }
      // Obstacles
      else if (name == node_name + ".Obstacles.min_obstacle_dist") {
        obstacles.min_obstacle_dist = value.double_value;
      } else if (name == node_name + ".Obstacles.inflation_dist") {
        obstacles.inflation_dist = value.double_value;
      } else if (name == node_name + ".Obstacles.dynamic_obstacle_inflation_dist") {
        obstacles.dynamic_obstacle_inflation_dist = value.double_value;
      } else if (name == node_name + ".Obstacles.obstacle_association_force_inclusion_factor") {
        obstacles.obstacle_association_force_inclusion_factor = value.double_value;
      }
      // Optimization
      else if (name == node_name + ".Optimization.penalty_epsilon") {
        optim.penalty_epsilon = value.double_value;
      } else if (name == node_name + ".Optimization.weight_vel_x_limit") {
        optim.weight_vel_x_limit = value.double_value;
      } else if (name == node_name + ".Optimization.weight_vel_theta_limit") {
        optim.weight_vel_theta_limit = value.double_value;
      } else if (name == node_name + ".Optimization.weight_acc_x_limit") {
        optim.weight_acc_x_limit = value.double_value;
      } else if (name == node_name + ".Optimization.weight_acc_theta_limit") {
        optim.weight_acc_theta_limit = value.double_value;
      } else if (name == node_name + ".Optimization.weight_kinematics_nh") {
        optim.weight_kinematics_nh = value.double_value;
      } else if (name == node_name + ".Optimization.weight_kinematics_turning_radius_limit") {
        optim.weight_kinematics_turning_radius_limit = value.double_value;
      } else if (name == node_name + ".Optimization.weight_time_minimize") {
        optim.weight_time_minimize = value.double_value;
      } else if (name == node_name + ".Optimization.weight_shortest_path") {
        optim.weight_shortest_path = value.double_value;
      } else if (name == node_name + ".Optimization.weight_obstacle_free_limit") {
        optim.weight_obstacle_free_limit = value.double_value;
      } else if (name == node_name + ".Optimization.weight_obstacle_of_dynamic_inflation") {
        optim.weight_obstacle_of_dynamic_inflation = value.double_value;
      } else if (name == node_name + ".Optimization.weight_viapoint") {
        optim.weight_viapoint = value.double_value;
        std::cout << "optim.weight_viapoint: " << optim.weight_viapoint << std::endl;
      } else if (name == node_name + ".Optimization.weight_adapt_factor") {
        optim.weight_adapt_factor = value.double_value;
      }

      else if (name == node_name + ".Optimization.weight_jerk_minimize") {
        optim.weight_jerk_minimize = value.double_value;
      } else if (name == node_name + ".Optimization.weight_acc_theta_minimize") {
        optim.weight_acc_theta_minimize = value.double_value;
      } else if (name == node_name + ".Optimization.weight_time_ascend_limit") {
        optim.weight_time_ascend_limit = value.double_value;
      } else if (name == node_name + ".Optimization.weight_via_area_base") {
        optim.weight_via_area_base = value.double_value;
      } else if (name == node_name + ".Optimization.weight_via_area_middle") {
        optim.weight_via_area_middle = value.double_value;
      } else if (name == node_name + ".Optimization.weight_via_area_front") {
        optim.weight_via_area_front = value.double_value;
      } else if (name == node_name + ".Optimization.weight_start_points_velocity") {
        optim.weight_start_points_velocity = value.double_value;
      } else if (name == node_name + ".Optimization.weight_start_points_direction") {
        optim.weight_start_points_direction = value.double_value;
      } else if (name == node_name + ".Optimization.weight_start_points_interval_distance") {
        optim.weight_start_points_interval_distance = value.double_value;
      } else if (name == node_name + ".Optimization.weight_final_goal_distance") {
        optim.weight_final_goal_distance = value.double_value;
      } else if (name == node_name + ".Optimization.weight_final_goal_yaw") {
        optim.weight_final_goal_yaw = value.double_value;
      } else if (name == node_name + ".Optimization.weight_final_goal_time") {
        optim.weight_final_goal_time = value.double_value;
      } else if (name == node_name + ".Optimization.divergence_detection_max_chi_squared") {
        optim.divergence_detection_max_chi_squared = value.double_value;
      }
    }

    else if (type == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER) {
      // Trajectory
      if (name == node_name + ".Trajectory.min_samples") {
        trajectory.min_samples = value.integer_value;
      } else if (name == node_name + ".Trajectory.max_samples") {
        trajectory.max_samples = value.integer_value;
      }

      // Obstacles
      else if (name == node_name + ".Obstacles.obstacle_poses_affected") {
        obstacles.obstacle_poses_affected = value.integer_value;
      }
      // Optimization
      else if (name == node_name + ".Optimization.no_inner_iterations") {
        optim.no_inner_iterations = value.integer_value;
      } else if (name == node_name + ".Optimization.no_outer_iterations") {
        optim.no_outer_iterations = value.integer_value;
      }

    }

    else if (type == rcl_interfaces::msg::ParameterType::PARAMETER_BOOL) {
      // Trajectory
      if (name == node_name + ".trajectory.exact_arc_length") {
        trajectory.exact_arc_length = value.bool_value;
      } else if (name == node_name + ".trajectory.free_goal_vel") {
        trajectory.free_goal_vel = value.bool_value;
      }
      // Obstacles
      else if (name == node_name + ".obstacles.include_dynamic_obstacles") {
        obstacles.include_dynamic_obstacles = value.bool_value;
      }
      // Optimization
      else if (name == node_name + ".Optimization.optimization_activate") {
        optim.optimization_activate = value.bool_value;
      } else if (name == node_name + ".Optimization.optimization_verbose") {
        optim.optimization_verbose = value.bool_value;
      } else if (name == node_name + ".Optimization.divergence_detection_enable") {
        optim.divergence_detection_enable = value.bool_value;
      }
    }

    else if (type == rcl_interfaces::msg::ParameterType::PARAMETER_STRING) {
    }
  }
  checkParameters();
}

void STEBConfig::checkParameters() const
{
  // rclcpp::Logger logger_{rclcpp::get_logger("TEBLocalPlanner")};
  //  positive backward velocity?
  if (vehicle_param.max_vel_x_backwards <= 0)
    RCLCPP_WARN(
      logger_,
      "STEBPlanner() Param Warning: Do not choose max_vel_x_backwards to be <=0. Disable backwards "
      "driving by increasing the optimization weight for penalyzing backwards driving.");

  // bounds smaller than penalty epsilon
  if (vehicle_param.max_vel_x <= optim.penalty_epsilon)
    RCLCPP_WARN(
      logger_,
      "STEBPlanner() Param Warning: max_vel_x <= penalty_epsilon. The resulting bound is negative. "
      "Undefined behavior... Change at least one of them!");

  if (vehicle_param.max_vel_x_backwards <= optim.penalty_epsilon)
    RCLCPP_WARN(
      logger_,
      "STEBPlanner() Param Warning: max_vel_x_backwards <= penalty_epsilon. The resulting bound is "
      "negative. Undefined behavior... Change at least one of them!");

  if (vehicle_param.max_vel_theta <= optim.penalty_epsilon)
    RCLCPP_WARN(
      logger_,
      "STEBPlanner() Param Warning: max_vel_theta <= penalty_epsilon. The resulting bound is "
      "negative. Undefined behavior... Change at least one of them!");

  if (vehicle_param.max_longitudinal_acc <= optim.penalty_epsilon)
    RCLCPP_WARN(
      logger_,
      "STEBPlanner() Param Warning: max_longitudinal_acc <= penalty_epsilon. The resulting bound "
      "is negative. Undefined behavior... Change at least one of them!");

  if (vehicle_param.max_theta_acc <= optim.penalty_epsilon)
    RCLCPP_WARN(
      logger_,
      "STEBPlanner() Param Warning: max_theta_acc <= penalty_epsilon. The resulting bound is "
      "negative. Undefined behavior... Change at least one of them!");

  // auto_resize_resolution and dt_hyst
  if (trajectory.auto_resize_resolution <= trajectory.auto_resize_resolution_hysteresis)
    RCLCPP_WARN(
      logger_,
      "STEBPlanner() Param Warning: auto_resize_resolution <= auto_resize_resolution_hysteresis. "
      "The hysteresis is not allowed to be greater or equal!. Undefined behavior... Change at "
      "least one of them!");

  // min number of samples
  if (trajectory.min_samples < 3)
    RCLCPP_WARN(
      logger_,
      "STEBPlanner() Param Warning: parameter min_samples is smaller than 3! Sorry, I haven't "
      "enough degrees of freedom to plan a trajectory for you. Please increase ...");

  // carlike
  if (vehicle_param.wheel_base == 0)
    RCLCPP_WARN(
      logger_,
      "STEBPlanner() Param Warning: parameter wheel_base is non-zero but wheelbase is set to zero: "
      "undesired behavior.");

  if (vehicle_param.min_turning_radius == 0)
    RCLCPP_WARN(
      logger_,
      "STEBPlanner() Param Warning: parameter min_turning_radius is non-zero but "
      "min_turning_radius is set to zero: undesired behavior. You are mixing a carlike and a "
      "diffdrive robot");

  // positive weight_adapt_factor
  if (optim.weight_adapt_factor < 1.0)
    RCLCPP_WARN(
      logger_, "STEBPlanner() Param Warning: parameter weight_adapt_factor shoud be >= 1.0");

  if (optim.weight_time_minimize <= 0)
    RCLCPP_WARN(
      logger_,
      "STEBPlanner() Param Warning: parameter weight_time_minimize shoud be > 0 (even if "
      "weight_shortest_path is in use)");
}

void STEBConfig::checkDeprecated(rclcpp::Node * nh) const
{
  rclcpp::Parameter dummy;

  if (
    nh->get_parameter(node_name + "." + "line_obstacle_poses_affected", dummy) ||
    nh->get_parameter(node_name + "." + "polygon_obstacle_poses_affected", dummy))
    RCLCPP_WARN(
      logger_,
      "STEBPlanner() Param Warning: 'line_obstacle_poses_affected' and "
      "'polygon_obstacle_poses_affected' are deprecated. They share now the common parameter "
      "'obstacle_poses_affected'.");

  if (
    nh->get_parameter(node_name + "." + "weight_point_obstacle", dummy) ||
    nh->get_parameter(node_name + "." + "weight_line_obstacle", dummy) ||
    nh->get_parameter(node_name + "." + "weight_poly_obstacle", dummy))
    RCLCPP_WARN(
      logger_,
      "STEBPlanner() Param Warning: 'weight_point_obstacle', 'weight_line_obstacle' and "
      "'weight_poly_obstacle' are deprecated. They are replaced by the single param "
      "'weight_obstacle_free_limit'.");

  if (nh->get_parameter(node_name + "." + "costmap_obstacles_front_only", dummy))
    RCLCPP_WARN(
      logger_,
      "STEBPlanner() Param Warning: 'costmap_obstacles_front_only' is deprecated. It is replaced "
      "by 'costmap_obstacles_behind_robot_dist' to define the actual area taken into account.");

  if (nh->get_parameter(node_name + "." + "costmap_emergency_stop_dist", dummy))
    RCLCPP_WARN(
      logger_,
      "STEBPlanner() Param Warning: 'costmap_emergency_stop_dist' is deprecated. You can safely "
      "remove it from your parameter config.");

  if (nh->get_parameter(node_name + "." + "alternative_time_cost", dummy))
    RCLCPP_WARN(
      logger_,
      "STEBPlanner() Param Warning: 'alternative_time_cost' is deprecated. It has been replaced by "
      "'selection_alternative_time_cost'.");

  if (nh->get_parameter(node_name + "." + "global_plan_via_point_sep", dummy))
    RCLCPP_WARN(
      logger_,
      "STEBPlanner() Param Warning: 'global_plan_via_point_sep' is deprecated. It has been "
      "replaced by 'global_plan_viapoint_sep' due to consistency reasons.");
}

}  // namespace steb_planner
