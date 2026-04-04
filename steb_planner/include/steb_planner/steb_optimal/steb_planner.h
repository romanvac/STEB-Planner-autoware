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

#ifndef OPTIMAL_PLANNER_H_
#define OPTIMAL_PLANNER_H_

#include <math.h>
#include <limits.h>
#include <map>
#include <memory>
#include <limits>



// g2o lib stuff
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"

// ros
#include <rclcpp/node.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
// messages
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2/time.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// g2o custom edges and vertices for the TEB planner
#include "steb_planner/g2o_types/edge_velocity.h"
#include "steb_planner/g2o_types/edge_acceleration.h"
#include "steb_planner/g2o_types/edge_kinematics.h"
#include "steb_planner/g2o_types/edge_time_optimal.h"
#include "steb_planner/g2o_types/edge_shortest_path.h"
#include "steb_planner/g2o_types/edge_obstacle.h"
#include "steb_planner/g2o_types/edge_via_point.h"

// teb stuff
#include "steb_planner/misc.h"
#include "steb_planner/std_cout.h"
#include "steb_planner/steb_obstacle/via_area.h"
#include "steb_planner/steb_obstacle/collision_free_corridor.h"
#include "steb_planner/steb_optimal/steb_config.h"
#include "steb_planner/steb_optimal/spatio_temporal_elastic_band.h"
#include "steb_planner/steb_optimal/vehicle_footprint_model.h"



namespace steb_planner
{

typedef std::vector< Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > Point2dContainer;
//! Typedef for a container storing via-points (x, y, theta, T)
typedef std::vector< Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > TrajectoryPointsContainer;

//! Typedef for the block solver utilized for optimization
typedef g2o::BlockSolverX STEBBlockSolver;

//! Typedef for the linear solver utilized for optimization
typedef g2o::LinearSolverCSparse<STEBBlockSolver::PoseMatrixType> STEBLinearSolver;



class STEBPlanner
{
public:

  STEBPlanner();

  STEBPlanner(rclcpp::Node* node, const STEBConfig* cfg,
              const nav_msgs::msg::Odometry* odometry = NULL, 
              const steb_planner::ObstContainer* obstacles = NULL,
              const TrajectoryPointsContainer* via_points = NULL,
              const nav_msgs::msg::OccupancyGrid* = NULL);

 ~STEBPlanner();

 // Plan a trajectory based on an initial reference plan.
 bool plan(const tf2::Transform &map_to_ego, const TrajectoryPointsContainer& global_plan);

 // Optimize a previously initialized trajectory (actual STEB optimization loop).
 bool optimizeSTEB(int iterations_innerloop, int iterations_outerloop, bool compute_cost_afterwards = true);

 // data input
 void setPridictedObject(autoware_auto_perception_msgs::msg::PredictedObjects & objects) {objects_ = objects;}

 void setPath(autoware_auto_planning_msgs::msg::Path & path) {path_ = path;}

 // void setOdometry(const nav_msgs::msg::Odometry* odometry) {odometry_ = odometry;}

 void setRvizPubTime(const rclcpp::Time & time) {rviz_pub_time_ = time;}

 // Assign a new set of obstacles
 void setObstVector(ObstContainer* obst_vector) {obstacles_ = obst_vector;}

 // Assign a new set of via-points
 void setViaPoints(const TrajectoryPointsContainer* via_points) {via_points_ = via_points;}


 // Access the internal obstacle container.
 const ObstContainer& getObstVector() const {return *obstacles_;}

 // Access the internal via-point container.
 const TrajectoryPointsContainer& getViaPoints() const {return *via_points_;}


 // Reset the planner by clearing the internal graph and trajectory.
 void clearPlanner()
 {
   clearGraph();
   steb_.clearTimedElasticBand();
 }



 // Register the vertices and edges defined for the TEB to the g2o::Factory.
 static void registerG2OTypes();

 // Initialize and configure the g2o sparse optimizer.
 std::shared_ptr<g2o::SparseOptimizer> initOptimizer();

 // Access the internal SpatioTemporalElasticBand trajectory.
 SpatioTemporalElasticBand& teb() {return steb_;};

 // Access the internal SpatioTemporalElasticBand trajectory (read-only).
 const SpatioTemporalElasticBand& teb() const {return steb_;};

 // Access the internal g2o optimizer.
 std::shared_ptr<g2o::SparseOptimizer> optimizer() {return optimizer_;};

 // Access the internal g2o optimizer (read-only).
 std::shared_ptr<const g2o::SparseOptimizer> optimizer() const {return optimizer_;};

 // Check if last optimization was successful
 bool isOptimized() const {return optimized_;};

 // Check if the planner has diverged.
 bool hasDiverged() const;

 // Compute the cost vector of a given optimization problen (hyper-graph must exist).
 void computeCurrentCostSTEB();


 // Access the cost vector.
 double getCurrentCost() const {return cost_;}


 // Extract the velocity from consecutive poses and a time difference (including strafing velocity for holonomic robots)
 inline void extractVelocity(const PoseSE2& pose1, const PoseSE2& pose2, double dt, double& vx, double& vy, double& omega) const;


 // Return the complete trajectory including poses, velocity profiles and temporal information
 void getFullTrajectory(TrajectoryPointsContainer& trajectory);

 // 计算贝塞尔曲线点
 inline Eigen::Vector3d bezierPoint(const std::vector<Eigen::Vector3d>& controlPoints, double t)
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

 // 生成贝塞尔曲线的插值点
 inline std::vector<Eigen::Vector3d> generateBezierCurve(const std::vector<Eigen::Vector3d>& controlPoints, int numPoints) {
   std::vector<Eigen::Vector3d> curvePoints;

   for (int i = 0; i <= numPoints; ++i) {
     double t = static_cast<double>(i) / numPoints;
     curvePoints.push_back(bezierPoint(controlPoints, t));
   }

   return curvePoints;
 }


 steb_planner::CVMaps getCVMaps(){return CVMaps_;}

//protected:
  private:

 /* Hyper-Graph creation and optimization */
 //@{

 // Build the hyper-graph representing the STEB optimization problem.
 bool buildGraph(double weight_multiplier=1.0, int iteration_time = -1);

 // Optimize the previously constructed hyper-graph to deform / optimize the STEB.
 bool optimizeGraph(int no_iterations, bool clear_after=true);

 // Clear an existing internal hyper-graph.
 void clearGraph();

 // Add all relevant vertices to the hyper-graph as optimizable variables.
 void AddSTEBVertices();

  // Add all edges (local cost functions) for minimizing the path length
 void AddEdgesShortestPath();

 /// Add all edges (local cost functions) for limiting the translational and angular velocity.
 void AddEdgesVelocity();

  // Add all edges (local cost functions) for limiting the translational and angular acceleration.
 void AddEdgesAcceleration();

 void AddEdgesInitVelocity();

 void AddEdgesFinalGoal();

 // Add all edges (local cost functions) for minimizing the transition time (resp. minimize time differences)
 void AddEdgesTimeOptimal();

 // Add all edges (local cost functions) related to keeping a distance from dynamic obstacles
 void AddEdgesObstacles(double weight_multiplier=1.0);

 void AddEdgesSTEBDynamicObstacles(double weight_multiplier=1.0);

  // Add all edges (local cost functions) related to keeping a distance from dynamic (moving) obstacles.
 void AddEdgesDynamicObstacles(double weight_multiplier=1.0);

 // Add all edges (local cost functions) for satisfying kinematic constraints of a car
 void AddEdgesKinematicsCarlike();


 // Add all edges (local cost functions) related to minimizing the distance to via-points
 // and edges keep a distance from static obstacle
 void AddEdgesSTEBViaPoints(double weight_multiplier=1.0);

 inline int findNearestViaPoints(Eigen::Vector3d & pose, int start_index = 0)
 {
   int nearest_index = -1;
   double min_dist_sq = std::numeric_limits<double>::max();
   Eigen::Vector2d current_position{pose.x(), pose.y()};
   for (int j = start_index; j < static_cast<int>(via_area_.size()); ++j)
   {
     Eigen::Vector2d vp_it_2d{via_area_[j].get()->getViaPoint().x(),
                              via_area_[j].get()->getViaPoint().y()};
     double dist_sq = (current_position - vp_it_2d).squaredNorm();
     if (dist_sq < min_dist_sq)
     {
       min_dist_sq = dist_sq;
       nearest_index = j;
     }
   }
   return nearest_index;
 }

 inline std::vector<Eigen::Vector3d> findCenterOfCollisionCircle(Eigen::Vector3d & pose_1, Eigen::Vector3d & pose_2)
 {
   std::vector<Eigen::Vector3d> center_vector;

   const Eigen::Vector2d deltaS{pose_2.x() - pose_1.x() ,
                                pose_2.y() - pose_1.y() };
   const double theta = std::atan2(deltaS.y(), deltaS.x());

   tf2::Transform current_pose_tf;
   current_pose_tf.setOrigin(tf2::Vector3(pose_1.x(), pose_1.y(), 0.0));
   tf2::Quaternion quaternion;
   quaternion.setRPY(0.0, 0.0, theta);
   current_pose_tf.setRotation(quaternion);

   tf2::Transform middle_center;
   middle_center.setIdentity();
   middle_center.setOrigin(tf2::Vector3(steb_cfg_->vehicle_param.rear_axle_2_center, 0.0, 0.0));
   middle_center = current_pose_tf * middle_center;
   Eigen::Vector3d middle_center_eigen {middle_center.getOrigin().x(), middle_center.getOrigin().y(), pose_1.z()};

   tf2::Transform front_center;
   front_center.setIdentity();
   front_center.setOrigin(tf2::Vector3(steb_cfg_->vehicle_param.wheel_base, 0.0, 0.0));
   front_center = current_pose_tf * front_center;
   Eigen::Vector3d front_center_eigen {front_center.getOrigin().x(), front_center.getOrigin().y(), pose_1.z()};

   center_vector.push_back(middle_center_eigen);
   center_vector.push_back(front_center_eigen);

   return center_vector;
 }

 void publishDebugMarker();
 void publishViaAreaDebugMarker();

 rclcpp::Node* node_;
 rclcpp::Time rviz_pub_time_;
 rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_viz_pub_;
 rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr via_area_debug_rviz_pub_;


 const STEBConfig* steb_cfg_; //!< Config class that stores and manages all related parameters
 const ObstContainer* obstacles_; //!< Store obstacles that are relevant for planning
 autoware_auto_perception_msgs::msg::PredictedObjects objects_;
 autoware_auto_planning_msgs::msg::Path path_;
 const nav_msgs::msg::Odometry* odometry_;
 const nav_msgs::msg::OccupancyGrid* occupancy_grid_ptr_;
 steb_planner::CVMaps CVMaps_;
 std::vector<ObstContainer> obstacles_per_vertex_; //!< Store the obstacles associated with the n-1 initial vertices
 const TrajectoryPointsContainer* via_points_; //!< Store via points for planning
 ViaAreaContainer via_area_; // @hs

 SpatioTemporalElasticBand steb_; //!< Actual trajectory object
 std::shared_ptr<g2o::SparseOptimizer> optimizer_; //!< g2o optimizer for trajectory optimization
 double cost_; //!< Store cost value of the current hyper-graph
 bool initialized_; //!< Keeps track about the correct initialization of this class
 bool optimized_; //!< This variable is \c true as long as the last optimization has been completed successful




public:
 EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//! Abbrev. for shared instances of the STEBOptimalPlanner
typedef std::shared_ptr<STEBPlanner> STEBPlannerPtr;
//! Abbrev. for shared const STEBOptimalPlanner pointers
typedef std::shared_ptr<const STEBPlanner> STEBPlannerConstPtr;
//! Abbrev. for containers storing multiple teb optimal planners
typedef std::vector< STEBPlannerPtr > STEBPlannerContainer;

} // namespace steb_planner

#endif /* OPTIMAL_PLANNER_H_ */
