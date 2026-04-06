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

#ifndef STEB_CONFIG_H_
#define STEB_CONFIG_H_

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <boost/smart_ptr/make_shared.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "steb_planner/steb_optimal/vehicle_footprint_model.h"
// #include "steb_planner/vehicle_parameter.h"

// Definitions
#define USE_ANALYTIC_JACOBI // if available for a specific edge, use analytic jacobi
#define UNUSED(x) (void)(x)

namespace steb_planner
{
// STEBConfig
class STEBConfig
{
public:
 using STEBConfigPtr = std::unique_ptr<STEBConfig>;

struct VehicleParam {
  // size
  double width = 1.9;
  double length = 4.9;
  double height = 1.45;
  double front_suspension = 0.90;
  double rear_suspension = 1.10;
  // kino
  double max_steering_angle = 0.70; // [rad]
  double min_turning_radius = 5.0;  // Minimum turning radius of a car;
  double wheel_base = 3.0;
  double rear_axle_2_center = 1.5;  // length between geometry center and rear axle
  // dynamic
  double max_vel_x = 10.0;           // Maximum translational velocity of the car
  double max_vel_x_backwards = 0.2;  // Maximum translational velocity of the robot for driving backwards
  double max_vel_theta = 0.5;        // Maximum angular velocity of the car
  double max_longitudinal_acc = 2.0;
  double max_lateral_acc = 2.0;
  double max_theta_acc = 0.5;   // Maximum angular acceleration of the car
  // cllision
  double circle_radius = 1.45;  // The radius of linearized circular contours of the collision models
};

 struct Trajectory
 {
   bool steb_autosize = true;         // Enable automatic resizing of the trajectory
   bool steb_autosize_fast_mode = true; 
   double auto_resize_resolution = 1.4; // Desired spatio-temporal resolution of the trajectory
   double auto_resize_resolution_hysteresis = 0.6; // Hysteresis for automatic resizing depending on the current spatio-temporal resolution: usually 10% of dt_ref
   int min_samples = 5;   // Minimum number of samples (should be always greater than 2)
   int max_samples = 100; // Maximum number of samples; 
   double max_global_plan_lookahead_dist = 50; // Specify maximum cumulative Euclidean distances of the global plan taken into account for optimization 
   double global_plan_velocity_set = 3;        // Max speed set for the global plan
   double current_velocity_for_debug = 0.0;
   bool exact_arc_length = true;               // If true, the planner uses the exact arc length in velocity, acceleration and turning rate computations [-> increased cpu time], otherwise the euclidean approximation is used.
   double force_reinit_new_goal_dist = 10;     // Reinitialize the trajectory if a previous goal is updated with a seperation of more than the specified value in meters 
   double force_reinit_new_goal_angular = 0.5 * M_PI; // Reinitialize the trajectory if a previous goal is updated with an angular difference of more than the specified value in radians
   bool free_goal_vel;    // Allow the car's velocity to be nonzero for planning purposes
   bool final_trajectory_bezier_smooth = true; 
 }; 

 struct Obstacles
 {
   double max_static_obstacle_velocity = 0.2; // divide static and dynamic obstacle by this velocity
   double min_obstacle_dist = 4;   // Minimum desired separation from obstacles
   double min_obstacle_time = 6.0; // Minimum time separation from obstacles for edge construction
   double inflation_dist = 3.5;    // Inflated minimum desired separation from obstacles
   bool include_dynamic_obstacles = false; 
   int obstacle_poses_affected = 25;  
   double obstacle_association_force_inclusion_factor = 1.5; 
   double dynamic_obstacle_inflation_dist = 4.0;
 };

 struct CollisionFreeCorridor
{
  bool is_avoiding_unknown = false;
  bool is_avoiding_car = true;
  bool is_avoiding_truck = true;
  bool is_avoiding_bus = true;
  bool is_avoiding_bicycle = true;
  bool is_avoiding_motorbike = true;
  bool is_avoiding_pedestrian = true;
  bool is_avoiding_animal = false;

  double soft_clearance_from_road = 0.3;
  double extra_desired_clearance_from_road = 0.0;
  double soft_clearance_from_object = 1.0;
  double extra_desired_clearance_from_object = 0.0;

  double max_bound_search_width = 5.0;
  std::vector<double> bounds_search_step_widths = {0.45, 0.15, 0.05};
};

 
 struct Optimization
 {
   int no_inner_iterations = 15;   // Number of solver iterations called in each outerloop iteration
   int no_outer_iterations = 4;    // Each outerloop iteration automatically resizes the trajectory and invokes the internal optimizer with no_inner_iterations

   bool optimization_activate = true; // Enable the optimization
   bool optimization_verbose = true;  // Print verbose information

   double penalty_epsilon = 0.05;     // Add a small safety margin to penalty functions for hard-constraint approximations

   double weight_vel_x_limit = 1.0;      // Optimization weight for satisfying the maximum allowed translational velocity
   double weight_vel_theta_limit = 1.0; // Optimization weight for satisfying the maximum allowed angular velocity
   double weight_acc_x_limit = 10.0;      // Optimization weight for satisfying the maximum allowed translational acceleration
   double weight_acc_theta_limit = 10.0; // Optimization weight for satisfying the maximum allowed angular acceleration
   double weight_acc_theta_minimize = 10.0;
   double weight_jerk_minimize = 10.0;

   double weight_kinematics_nh = 100.0;    // Optimization weight for satisfying the non-holonomic kinematics
   double weight_kinematics_turning_radius_limit = 1.0; // Optimization weight for enforcing a minimum turning radius 
   double weight_time_minimize = 0.001;              // Optimization weight for contracting the trajectory w.r.t. transition time
   double weight_time_ascend_limit = 100.0;
   double weight_shortest_path = 1.0;            // Optimization weight for contracting the trajectory w.r.t. path length
   double weight_obstacle_free_limit = 2.0;       // Optimization weight for satisfying a minimum separation from obstacles
   double weight_obstacle_of_dynamic_inflation = 0.01; // Optimization weight for the inflation penalty of dynamic obstacles (should be small)
   double weight_viapoint = 0.001;                      // Optimization weight for minimizing the distance to via-points
   double weight_via_area_base = 10;
   double weight_via_area_middle = 1.0;
   double weight_via_area_front = 1.0;
   double weight_start_points_velocity = 0.0;
   double weight_start_points_direction = 0.0;
   double weight_start_points_interval_distance = 0.0;
   double weight_final_goal_distance = 0.1;
   double weight_final_goal_yaw = 0.1;
   double weight_final_goal_time = 1.1;

// Some special weights (currently 'weight_obstacle') are repeatedly scaled by this factor in each outer STEB iteration (weight_new = weight_old*factor); 
// Increasing weights iteratively instead of setting a huge value a-priori leads to better numerical conditions of the underlying optimization problem.
   double weight_adapt_factor = 1.2; 
   double obstacle_cost_exponent = 1.0; // Exponent for nonlinear obstacle cost (cost = linear_cost * obstacle_cost_exponent). Set to 1 to disable nonlinear cost (default)
 
   bool divergence_detection_enable = true;       // True to enable divergence detection.
   int divergence_detection_max_chi_squared = 10; // Maximum acceptable Mahalanobis distance above which it is assumed that the optimization diverged.
 }; 


 // Declares static ROS2 parameter
 void declareParameters(rclcpp::Node* nh);

 // Declares static ROS2 parameter with given type if it was not already declared
 void declare_parameter_if_not_declared(
     rclcpp::Node* node,
     const std::string & param_name,
     const rclcpp::ParameterValue & default_value,
     const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor =
         rcl_interfaces::msg::ParameterDescriptor())
 {
   if (!node->has_parameter(param_name)) {
     node->declare_parameter(param_name, default_value, parameter_descriptor);
   }
 }

 // Load parmeters from the ros param server.
 void loadRosParamFromNodeHandle(rclcpp::Node* nh);

 // Paremeter event callback
 void on_parameter_event_callback(
     const rcl_interfaces::msg::ParameterEvent::SharedPtr event);

 // Check parameters and print warnings in case of discrepancies
 void checkParameters() const;

 // Check if some deprecated parameters are found and print warnings
 void checkDeprecated(rclcpp::Node* nh) const;

 // Return the internal config mutex
 std::mutex& configMutex() {return config_mutex_;}


 // parameters
std::string node_name = "STEB";

VehicleParam vehicle_param;
VehicleFootprintModelPtr  ego_vehicle_model = 
boost::make_shared<ThreeCirclesVehicleFootprint>(vehicle_param.wheel_base, 
                                                   vehicle_param.wheel_base*0.5, 
                                                   vehicle_param.circle_radius);
Trajectory trajectory;
Obstacles obstacles;
CollisionFreeCorridor collision_free_corridor;
Optimization optim;

std::mutex config_mutex_; //  Mutex for config accesses and changes

private:
  rclcpp::Logger logger_{rclcpp::get_logger("STEBPlanner")};
 
};

} // namespace steb_planner

#endif
