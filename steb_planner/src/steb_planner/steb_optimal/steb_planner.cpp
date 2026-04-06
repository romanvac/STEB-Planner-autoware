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

#include "steb_planner/steb_optimal/steb_planner.h"
#include "steb_planner/g2o_types/edge_obstacle.h"
#include "steb_planner/steb_obstacle/collision_free_corridor.h"
#include <cstddef>
#include <steb_planner/tic_toc.h>

namespace steb_planner {

// ============== Implementation ===================

STEBPlanner::STEBPlanner()
    : node_(nullptr), steb_cfg_(NULL), obstacles_(NULL), occupancy_grid_ptr_(NULL), via_points_(NULL), 
      cost_(HUGE_VAL), initialized_(false), optimized_(false) {}

STEBPlanner::STEBPlanner(rclcpp::Node* node, const STEBConfig *cfg,
                         const nav_msgs::msg::Odometry* odometry,
                         const steb_planner::ObstContainer *obstacles,
                         const TrajectoryPointsContainer *via_points,
                         const nav_msgs::msg::OccupancyGrid *occupancy_grid_ptr)
{
  node_ = node;
  odometry_ = odometry;
  debug_viz_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/steb_planner/vox_to_obs", 1);
  via_area_debug_rviz_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/steb_planner/via_area", 1);

  // init optimizer (set solver and block ordering settings)
  optimizer_ = initOptimizer();

  steb_cfg_ = cfg;
  obstacles_ = obstacles;
  via_points_ = via_points;
  occupancy_grid_ptr_ = occupancy_grid_ptr;
  cost_ = HUGE_VAL;

  initialized_ = true;
}

STEBPlanner::~STEBPlanner() {
  clearGraph();
  // free dynamically allocated memory
  // if (optimizer_)
  //  g2o::Factory::destroy();
  // g2o::OptimizationAlgorithmFactory::destroy();
  // g2o::HyperGraphActionLibrary::destroy();
}

template <typename T>
void register_type(g2o::Factory *factory, const std::string name) {
  std::unique_ptr<g2o::HyperGraphElementCreator<T>> ptr_(
      new g2o::HyperGraphElementCreator<T>());
  std::shared_ptr<g2o::HyperGraphElementCreator<T>> shared_(std::move(ptr_));

  factory->registerType(name, shared_);
}

/*
 * registers custom vertices and edges in g2o framework
 */
void STEBPlanner::registerG2OTypes() {
  g2o::Factory *factory = g2o::Factory::instance();
  register_type<VertexST>(factory, "VERTEX_POSE");
  register_type<EdgeTimeOptimal>(factory, "EDGE_TIME_OPTIMAL");
  register_type<EdgeShortestPath>(factory, "EDGE_SHORTEST_PATH");
  register_type<EdgeVelocity>(factory, "EDGE_VELOCITY");
  register_type<EdgeAcceleration>(factory, "EDGE_ACCELERATION");
  register_type<EdgeKinematicsCarlike>(factory, "EDGE_KINEMATICS_CARLIKE");
  register_type<EdgeDynamicObstacleSpatio>(factory, "EDGE_DYNAMIC_OBSTACLE_SPATIO");
  register_type<EdgeDynamicObstacleTemporal>(factory, "EDGE_DYNAMIC_OBSTACLE_TEMPORAL");
  register_type<EdgeDynamicObstacleST>(factory, "EDGE_DYNAMIC_OBSTACLE_ST");
  register_type<EdgeViaPoint>(factory, "EDGE_VIA_POINT");
  
  return;
}

/*
 * initialize g2o optimizer. Set solver settings here.
 * Return: pointer to new SparseOptimizer Object.
 */
std::shared_ptr<g2o::SparseOptimizer> STEBPlanner::initOptimizer() {
  // Call register_g2o_types once, even for multiple STEBPlanner instances
  // (thread-safe)
  static std::once_flag flag;
  std::call_once(flag, [this]() { this->registerG2OTypes(); });

  // allocating the optimizer
  std::shared_ptr<g2o::SparseOptimizer> optimizer = std::make_shared<g2o::SparseOptimizer>();
  auto linearSolver = std::make_unique<STEBLinearSolver>(); // see typedef in optimization.h
  linearSolver->setBlockOrdering(true);

  auto blockSolver = std::make_unique<STEBBlockSolver>(std::move(linearSolver));
  g2o::OptimizationAlgorithmLevenberg *solver =
      new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));

  optimizer->setAlgorithm(solver);

  optimizer->initMultiThreading(); // required for >Eigen 3.1

  return optimizer;
}



bool STEBPlanner::plan(const tf2::Transform &map_to_geo, const TrajectoryPointsContainer &global_plan)
{
  STEB_ASSERT_MSG(initialized_, "Call initialize() first.");

  // add a fixed vertex for considering the initial state of ego
  constexpr double handle_length = -0.5;
  Eigen::Vector3d handle_start{handle_length, 0.0, std::max(-std::fabs(handle_length / odometry_->twist.twist.linear.x), -5.0)};
  // Eigen::Vector3d handle_start{handle_length, 0.0, -2.0};

  constexpr bool need_warm_start = false;
  if (!need_warm_start)
  {
    steb_.clearTimedElasticBand();
    steb_.initTrajectoryToGoal(map_to_geo, global_plan, handle_start);

    steb_.printSTEB("init STEB");
  }
  else
  {
    if (!steb_.isInit())
    {
      steb_.initTrajectoryToGoal(map_to_geo, global_plan, handle_start);

      steb_.printSTEB("first init STEB");
    }
    else // warm start
    {
      bool is_time_descend = false;
      if (steb_.sizePoses() > 2)
      {
        for (int j=0; j <steb_.sizePoses()-1; ++j)
        {
          if ( (steb_.getPoseReadOnly(j+1).z() - steb_.getPoseReadOnly(j).z()) < 0.0 )
          {
            is_time_descend = true;
          }
        }
      }

      Eigen::Vector3d start_(global_plan.front().x(), global_plan.front().y(), global_plan.front().w());
      Eigen::Vector3d goal_(global_plan.back().x(), global_plan.back().y(), global_plan.back().w());
      double path_goal_dist = 0; 

      if (steb_.sizePoses() > 2 && path_goal_dist < steb_cfg_->trajectory.force_reinit_new_goal_dist && !is_time_descend) // actual warm start!
      {
        steb_.updateAndPruneTEB(map_to_geo, handle_start, start_, goal_, 
                                global_plan, steb_cfg_->trajectory.min_samples); // update TEB

        steb_.printSTEB("updated STEB");
      }
      else // goal too far away -> reinit
      {
        RCLCPP_DEBUG(rclcpp::get_logger("steb_planner"),
                     "New goal: distance to existing goal is higher than the "
                     "specified threshold. Reinitalizing trajectories.");
        steb_.clearTimedElasticBand();
        steb_.initTrajectoryToGoal(map_to_geo, global_plan, handle_start);

        steb_.printSTEB("reinit STEB");
      }
    }
  }

  // std::cout << "-------------------- steb size : " << steb_.sizePoses() << std::endl;

  //  if (occupancy_grid_ptr_)
  //  {

  //  std::cout << "-------------------- occ_map : " << steb_.sizePoses() << std::endl;
  //  std::cout << occupancy_grid_ptr_->info.height << ", "
  //            << occupancy_grid_ptr_->info.width << ", "
  //            << occupancy_grid_ptr_->data.size() << std::endl;
  //  std::cout << occupancy_grid_ptr_->info.origin.position.x << ", "
  //            << occupancy_grid_ptr_->info.origin.position.y << ", "
  //            << tf2::getYaw(occupancy_grid_ptr_->info.origin.orientation) << std::endl;

  autoware_auto_perception_msgs::msg::PredictedObjects objects_in_ego = objects_;
  for (auto & object : objects_in_ego.objects)
  {
      tf2::Transform tf_obj_pose;
      tf2::fromMsg(object.kinematics.initial_pose_with_covariance.pose, tf_obj_pose);
      // map_to_geo = base_link→map, поэтому inverse() = map→base_link
      tf2::Transform tf_obj_in_ego = map_to_geo.inverse() * tf_obj_pose;
      tf2::toMsg(tf_obj_in_ego, object.kinematics.initial_pose_with_covariance.pose);
  }

  steb_planner::CollisionFreeCorridor collision_free_corridor(steb_cfg_, occupancy_grid_ptr_, objects_in_ego, path_);
  CVMaps_ = collision_free_corridor.getCVMaps();
  //  }

  static TicToc bound_calc_time_cost;
  bound_calc_time_cost.tic();

  via_area_.clear();
  via_area_.reserve((int)global_plan.size());
  for (size_t i = 0; i < global_plan.size(); ++i)
  {
    Eigen::Vector3d via_pose{global_plan[i].x(), global_plan[i].y(), global_plan[i].z()};
    steb_planner::ViaAreaPtr new_via_area(new steb_planner::ViaArea(via_pose, global_plan[i].w()));
    auto bound = collision_free_corridor.calcBound(via_pose);
    if (!bound.empty())
    {
      // Найти bounds, чья середина ближайшая к нулю (к самой via_point)
      auto best = bound.begin();
      for (auto it = bound.begin(); it != bound.end(); ++it) {
        if (it->lower_bound <= 0.0 && it->upper_bound >= 0.0) {
          best = it;
          break;
        }
      }
      // new_via_area->setLeftBound(best->upper_bound);
      // new_via_area->setRightBound(best->lower_bound);

      new_via_area->setLeftBound(bound.back().upper_bound);
      new_via_area->setRightBound(bound.back().lower_bound);
      // print
      for (int j = 0; j < static_cast<int>(bound.size()); ++j)
      {
        std::cout << "Bound i: " << i << "|"
                  << j << " -( "
                  << bound.at(j).upper_bound << ", "
                  << bound.at(j).lower_bound << " )" << std::endl;
      }
    }
    via_area_.push_back(new_via_area);
  }

  auto bound_calc = bound_calc_time_cost.toc();
  std::cout << "********** bound calc cost: " << bound_calc << " ms, " << std::endl;

  // now optimize
  return optimizeSTEB(steb_cfg_->optim.no_inner_iterations,
                     steb_cfg_->optim.no_outer_iterations);
}


bool STEBPlanner::optimizeSTEB(int iterations_innerloop,
                               int iterations_outerloop,
                               bool compute_cost_afterwards)
{
  if (steb_cfg_->optim.optimization_activate == false)
    return false;

  bool success = false;
  optimized_ = false;

  double weight_multiplier = 1.0;
  for (int iteration_time = 0; iteration_time < iterations_outerloop; ++iteration_time) {
    if (steb_cfg_->trajectory.steb_autosize) {
      steb_.autoResize(steb_cfg_->trajectory.auto_resize_resolution, 
                       steb_cfg_->trajectory.auto_resize_resolution_hysteresis,
                       steb_cfg_->trajectory.min_samples, 
                       steb_cfg_->trajectory.max_samples, steb_cfg_->trajectory.steb_autosize_fast_mode);
                       
      steb_.printSTEB("auto resized STEB");                 
    }

    static steb_planner::TicToc time_cost;
    time_cost.tic();

    success = buildGraph(weight_multiplier, iteration_time);
    auto time_ = time_cost.toc();
    std::cout << "********** build graph time cost: " << time_ << " ms, " << std::endl;

    publishDebugMarker();
    publishViaAreaDebugMarker();

    if (!success)
    {
      clearGraph();
      return false;
    }

    time_cost.tic();
    success = optimizeGraph(iterations_innerloop, false);
    time_ = time_cost.toc();
    std::cout <<OUT_GREEN<< "********** optimize graph time cost: " << time_ << " ms, " <<OUT_RESET<< std::endl;
    // steb_.printSTEB(std::string "STEB_OptimizedPoints_iterations_outerloop");

    if (!success)
    {
      clearGraph();
      return false;
    }
    optimized_ = true;

    time_cost.tic();
//    if (compute_cost_afterwards && i == iterations_outerloop - 1) // compute cost vec only in the last iteration
    if (compute_cost_afterwards)
    {
      computeCurrentCostSTEB();
      time_ = time_cost.toc();
      std::cout << "********** compute cost time cost: " << time_ << " ms, "
                <<OUT_RED<<"("<<iteration_time<<") cost is: " << cost_ << OUT_RESET << std::endl;
    }

    clearGraph();
    weight_multiplier *= steb_cfg_->optim.weight_adapt_factor;
  }

  return true;
}

bool STEBPlanner::buildGraph(double weight_multiplier,  int iteration_time) {

  if (!optimizer_->edges().empty() || !optimizer_->vertices().empty())
  {
    RCLCPP_WARN(rclcpp::get_logger("steb_planner"), "Cannot build graph, because it is not empty. Call graphClear()!");
    return false;
  }

  optimizer_->setComputeBatchStatistics(steb_cfg_->optim.divergence_detection_enable);

  std::cout << "--------------- objective function weight: ---------------- " << std::endl;
  std::cout <<"steb_cfg_->optim.weight_shortest_path :              " << steb_cfg_->optim.weight_shortest_path << std::endl;

  std::cout <<"steb_cfg_->optim.weight_kinematics_nh :              " << steb_cfg_->optim.weight_kinematics_nh << std::endl;
  std::cout <<"steb_cfg_->optim.weight_kinematics_turning_radius :  " << steb_cfg_->optim.weight_kinematics_turning_radius_limit << std::endl;

  std::cout <<"steb_cfg_->optim.weight_optimaltime :                " << steb_cfg_->optim.weight_time_minimize << std::endl;
  std::cout <<"steb_cfg_->optim.weight_ascend_time :                " << steb_cfg_->optim.weight_time_ascend_limit << std::endl;

  std::cout <<"steb_cfg_->optim.weight_max_vel_x :                  " << steb_cfg_->optim.weight_vel_x_limit << std::endl;
  std::cout <<"steb_cfg_->optim.weight_max_vel_theta :              " << steb_cfg_->optim.weight_vel_theta_limit << std::endl;

  std::cout <<"steb_cfg_->optim.weight_init_velocity :              " << steb_cfg_->optim.weight_start_points_velocity << std::endl;
  std::cout <<"steb_cfg_->optim.weight_final_goal_distance :        " << steb_cfg_->optim.weight_final_goal_distance << std::endl;
  std::cout <<"steb_cfg_->optim.weight_final_goal_yaw :             " << steb_cfg_->optim.weight_final_goal_yaw << std::endl;
  std::cout <<"steb_cfg_->optim.weight_final_goal_time :            " << steb_cfg_->optim.weight_final_goal_time << std::endl;

  std::cout <<"steb_cfg_->optim.weight_acc_lim_x :                  " << steb_cfg_->optim.weight_acc_x_limit << std::endl;
  std::cout <<"steb_cfg_->optim.weight_acc_lim_theta :              " << steb_cfg_->optim.weight_acc_theta_limit << std::endl;
  std::cout <<"steb_cfg_->optim.weight_minimum_jerk :               " << steb_cfg_->optim.weight_jerk_minimize << std::endl;
  std::cout <<"steb_cfg_->optim.weight_minimum_angle_velocity :     " << steb_cfg_->optim.weight_acc_theta_minimize << std::endl;

  std::cout <<"steb_cfg_->optim.weight_viapoint :                   " << steb_cfg_->optim.weight_viapoint << std::endl;
  std::cout <<"steb_cfg_->optim.weight_via_area_base :              " << steb_cfg_->optim.weight_via_area_base << std::endl;
  std::cout <<"steb_cfg_->optim.weight_via_area_middle :            " << steb_cfg_->optim.weight_via_area_middle << std::endl;
  std::cout <<"steb_cfg_->optim.weight_via_area_front :             " << steb_cfg_->optim.weight_via_area_front << std::endl;

  if (iteration_time == 0)
  {

  }

  // add TEB vertices
  AddSTEBVertices();

  AddEdgesShortestPath();

  AddEdgesKinematicsCarlike();

  AddEdgesTimeOptimal();

  AddEdgesVelocity();

  AddEdgesAcceleration();

//  AddEdgesInitVelocity();

  AddEdgesFinalGoal();

  // @hs: for static obstalce
//  AddEdgesSTEBViaPoints(weight_multiplier);
  AddEdgesSTEBViaPoints();

  //  for dynamic obstacle
  AddEdgesSTEBDynamicObstacles(weight_multiplier);

  return true;
}

bool STEBPlanner::optimizeGraph(int no_iterations, bool clear_after) {
  if (steb_cfg_->vehicle_param.max_vel_x < 0.01)
  {
    RCLCPP_WARN(rclcpp::get_logger("steb_planner"),
                "optimizeGraph(): Robot Max Velocity is smaller than 0.01m/s. Optimizing aborted...");
    if (clear_after)
      clearGraph();
    return false;
  }

  if (!steb_.isInit() || steb_.sizePoses() < steb_cfg_->trajectory.min_samples) {
    RCLCPP_WARN(rclcpp::get_logger("steb_planner"), "optimizeGraph(): STEB is empty or has too less elements. Skipping optimization.");
    if (clear_after)
      clearGraph();
    return false;
  }

  optimizer_->setVerbose(steb_cfg_->optim.optimization_verbose);
  optimizer_->initializeOptimization();

  int iter = optimizer_->optimize(no_iterations);

  // Save Hessian for visualization
  //  g2o::OptimizationAlgorithmLevenberg* lm =
  //  dynamic_cast<g2o::OptimizationAlgorithmLevenberg*> (optimizer_->solver());
  //  lm->solver()->saveHessian("~/Hessian.txt");

  if (!iter) {
    RCLCPP_ERROR(rclcpp::get_logger("steb_planner"), "optimizeGraph(): Optimization failed! iter=%i", iter);
    return false;
  }

  if (clear_after)
    clearGraph();

  return true;
}

void STEBPlanner::clearGraph() {
  // clear optimizer states
  if (optimizer_)
  {
    // we will delete all edges but keep the vertices.
    // before doing so, we will delete the link from the vertices to the edges.
    auto &vertices = optimizer_->vertices();
    for (auto &v : vertices)
      v.second->edges().clear();

    optimizer_->vertices().clear(); // necessary, because optimizer->clear deletes pointer-targets (therefore it deletes TEB states!)
    optimizer_->clear();
  }
}

void STEBPlanner::AddSTEBVertices() {
  // add vertices to graph
  RCLCPP_DEBUG_EXPRESSION(rclcpp::get_logger("steb_planner"), 
                          steb_cfg_->optim.optimization_verbose, "Adding STEB vertices ...");

  unsigned int id_counter = 0; // used for vertices ids
  obstacles_per_vertex_.resize(steb_.sizePoses());
  auto iter_obstacle = obstacles_per_vertex_.begin();
  for (int i = 0; i < steb_.sizePoses(); ++i)
  {
    steb_.getPoseVertex(i)->setId(id_counter++);
    optimizer_->addVertex(steb_.getPoseVertex(i));

    iter_obstacle->clear();
    (iter_obstacle++)->reserve(obstacles_->size());
  }
}

void STEBPlanner::AddEdgesSTEBDynamicObstacles(double weight_multiplier) {

  if (steb_cfg_->optim.weight_obstacle_free_limit == 0 || weight_multiplier == 0 || obstacles_ == nullptr)
    return; // if weight equals zero skip adding edges!

  auto iter_obstacle = obstacles_per_vertex_.begin();
  for (int i = 1; i < steb_.sizePoses()-1; ++i) {
    // iterate obstacles
    for (const ObstaclePtr &obst : *obstacles_)
    {
//      if (std::fabs(steb_.getPose(i).z() - obst->getEmergenceTime()) > 2.0) {
//        continue;
//      }
      const Eigen::Vector2d deriction = {steb_.getPose(i+1).x() - steb_.getPose(i).x(),
                                         steb_.getPose(i+1).y() - steb_.getPose(i).y()};
      const double theta = std::atan2(deriction.y(), deriction.x());

      const Eigen::Vector2d distance_vector = {obst->getCentroid().x() - steb_.getPose(i).x(),
                                               obst->getCentroid().y() - steb_.getPose(i).y()};
      double dist1 = distance_vector.norm();

      const Eigen::Vector2d obstacle_deriction = obst->getEndPoint() - obst->getStarPoint();
      const double theta2 = std::atan2(obstacle_deriction.y(), obstacle_deriction.x());

      const double theta_diff = std::cos( g2o::normalize_theta(std::abs(theta - theta2)) );

      const double time_diff = std::fabs(obst->getEmergenceTime() - steb_.getPose(i).z());
      UNUSED(theta_diff);
      // force considering obstacle if really close to the current pose
//      if (dist1 < steb_cfg_->obstacles.min_obstacle_dist * steb_cfg_->obstacles.obstacle_association_force_inclusion_factor
//          && theta_diff > - 0.9 && time_diff < 8.0)
      if (dist1 < steb_cfg_->obstacles.min_obstacle_dist * steb_cfg_->obstacles.obstacle_association_force_inclusion_factor
          && time_diff < steb_cfg_->obstacles.min_obstacle_time)
      {
        iter_obstacle->push_back(obst);
        continue;
      }
    }

    // create obstacle edges
    for (const auto& obst : *iter_obstacle)
    {
      Eigen::Matrix<double, 2, 2> information_dynamic_obstacle;
      information_dynamic_obstacle(0, 1) = information_dynamic_obstacle(1, 0) = 0;
      information_dynamic_obstacle(0, 0) = steb_cfg_->optim.weight_obstacle_free_limit * weight_multiplier;
      information_dynamic_obstacle(1, 1) = steb_cfg_->optim.weight_obstacle_of_dynamic_inflation;

      EdgeDynamicObstacleST *dist_bandpt_obst = new EdgeDynamicObstacleST;
      dist_bandpt_obst->setVertex(0, steb_.getPoseVertex(i));
      dist_bandpt_obst->setVertex(1, steb_.getPoseVertex(i + 1));
      dist_bandpt_obst->setInformation(information_dynamic_obstacle);
      dist_bandpt_obst->setSTEBConfig(*steb_cfg_);
      dist_bandpt_obst->setParameters(obst.get());
      optimizer_->addEdge(dist_bandpt_obst);

      // EdgeDynamicObstacleTemporal *dist_bandpt_obst = new EdgeDynamicObstacleTemporal;
      // dist_bandpt_obst->setVertex(0, steb_.getPoseVertex(i));
      // dist_bandpt_obst->setInformation(information_dynamic_obstacle);
      // dist_bandpt_obst->setSTEBConfig(*steb_cfg_);
      // dist_bandpt_obst->setParameters(obst.get());
      // optimizer_->addEdge(dist_bandpt_obst);

//      std::cout << "vertex < " << i << " > of "<<steb_.sizePoses() <<" : ( "
//                << steb_.getPose(i).x() << ", "
//                << steb_.getPose(i).y() << ", "
//                << steb_.getPose(i).z() << " ) "
//                << " have obstacle: ( "
//                << obst.get()->getCentroid().x() << ", "
//                << obst.get()->getCentroid().y() << ", "
//                << obst.get()->getEmergenceTime()<< " )" << std::endl;
    }

    ++iter_obstacle;
  }
}

void STEBPlanner::AddEdgesSTEBViaPoints(double weight_multiplier)
{
  if (steb_cfg_->optim.weight_viapoint == 0 || via_area_.empty() || via_points_->empty())
    return; // if weight equals zero skip adding edges!

  // int start_pose_idx = 0;
  if (steb_.sizePoses() < steb_cfg_->trajectory.min_samples) 
    return;

  for (int i = 0; i < steb_.sizePoses()-1; ++i)
  {
//    Eigen::Vector3d vp_it_3d;
    int nearest_index = findNearestViaPoints(steb_.getPose(i));
    if (nearest_index == -1) {
      continue;
    }

    std::vector<Eigen::Vector3d> center_vector = findCenterOfCollisionCircle(steb_.getPose(i), steb_.getPose(i+1));
    int middle_center_nearest_index = findNearestViaPoints(center_vector.front(), nearest_index);
    int front_center_nearest_index = findNearestViaPoints(center_vector.back(), nearest_index);
    if (middle_center_nearest_index == -1 || front_center_nearest_index == -1) {
      continue;
    }
    std::vector<int> nearest_index_vector = {nearest_index, middle_center_nearest_index, front_center_nearest_index};

//    std::cout << "vertex < " << i << " > : ( " << steb_.getPose(i).x() << ", "
//              << steb_.getPose(i).y() << ", " << steb_.getPose(i).z() << " ) "
//              << " colse to via point: ( " << vp_it_3d.x() << ", "
//              << vp_it_3d.y() << ", " << vp_it_3d.z() << " )" << std::endl;

    Eigen::Matrix<double, 4, 4> information;
    information.fill(0.0);
    information(0, 0) = steb_cfg_->optim.weight_viapoint;
    information(1, 1) = steb_cfg_->optim.weight_via_area_base * weight_multiplier;
    information(2, 2) = steb_cfg_->optim.weight_via_area_middle;
    information(3, 3) = steb_cfg_->optim.weight_via_area_front;
//    information(4, 4) = 0;

    EdgeViaPoint *edge_viapoint = new EdgeViaPoint;
    edge_viapoint->setVertex(0, steb_.getPoseVertex(i));
    edge_viapoint->setVertex(1, steb_.getPoseVertex(i+1));
    edge_viapoint->setInformation(information);
    edge_viapoint->setNearestViaPointIndex(nearest_index_vector);
    edge_viapoint->setSTEBConfig(*steb_cfg_);
    edge_viapoint->setParameters(&via_area_);
    optimizer_->addEdge(edge_viapoint);
  }
}

void STEBPlanner::AddEdgesVelocity() {

  if (steb_cfg_->optim.weight_vel_x_limit == 0 &&
      steb_cfg_->optim.weight_vel_theta_limit == 0)
    return; // if weight equals zero skip adding edges!

  int n = steb_.sizePoses();
  Eigen::Matrix<double, 3, 3> information;
  information.fill(0);
  information(0, 0) = steb_cfg_->optim.weight_vel_x_limit;
  information(1, 1) = steb_cfg_->optim.weight_vel_theta_limit;
  information(2, 2) = steb_cfg_->optim.weight_vel_theta_limit;

  for (int i = 0; i < n - 2; ++i) {
    EdgeVelocity *velocity_edge = new EdgeVelocity;
    velocity_edge->setVertex(0, steb_.getPoseVertex(i));
    velocity_edge->setVertex(1, steb_.getPoseVertex(i + 1));
    velocity_edge->setVertex(2, steb_.getPoseVertex(i + 2));
    velocity_edge->setInformation(information);
    velocity_edge->setSTEBConfig(*steb_cfg_);
    optimizer_->addEdge(velocity_edge);
  }
}

void STEBPlanner::AddEdgesInitVelocity()
{
  if (steb_.sizePoses() < steb_cfg_->trajectory.min_samples)
    return ;
  Eigen::Matrix<double, 2, 2> information;
  information.fill(0);
  information(0, 0) = steb_cfg_->optim.weight_start_points_velocity;
  information(1, 1) = steb_cfg_->optim.weight_start_points_direction;

  for (int i = 0; i < 2; ++i) {
    EdgeInitVelocity *inti_velocity_edge = new EdgeInitVelocity;
    inti_velocity_edge->setVertex(0, steb_.getPoseVertex(i));
    inti_velocity_edge->setVertex(1, steb_.getPoseVertex(i + 1));
    inti_velocity_edge->setInformation(information);
    inti_velocity_edge->setSTEBConfig(*steb_cfg_);
    inti_velocity_edge->setParameters(odometry_);
    optimizer_->addEdge(inti_velocity_edge);
  }
}

void STEBPlanner::AddEdgesFinalGoal()
{
  if (steb_.sizePoses() < steb_cfg_->trajectory.min_samples)
    return ;

  Eigen::Matrix<double, 3, 3> information;
  information.fill(0);
  information(0, 0) = steb_cfg_->optim.weight_final_goal_distance;
  information(1, 1) = steb_cfg_->optim.weight_final_goal_yaw;
  information(2, 2) = steb_cfg_->optim.weight_final_goal_time;

  EdgeGoalPoint *final_goal_edge = new EdgeGoalPoint;
  final_goal_edge->setVertex(0, steb_.getPoseVertex(steb_.sizePoses()-2));
  final_goal_edge->setVertex(1, steb_.getPoseVertex(steb_.sizePoses()-1));
  final_goal_edge->setInformation(information);
  final_goal_edge->setSTEBConfig(*steb_cfg_);
  final_goal_edge->setParameters(&via_points_->back());
  optimizer_->addEdge(final_goal_edge);

}

void STEBPlanner::AddEdgesAcceleration() {
  if (steb_cfg_->optim.weight_acc_x_limit == 0 &&
      steb_cfg_->optim.weight_acc_theta_limit == 0)
    return; // if weight equals zero skip adding edges!

  int n = steb_.sizePoses();

  Eigen::Matrix<double, 4, 4> information;
  information.fill(0);
  information(0, 0) = steb_cfg_->optim.weight_acc_x_limit;
  information(1, 1) = steb_cfg_->optim.weight_acc_theta_limit;
  information(2, 2) = steb_cfg_->optim.weight_jerk_minimize;
  information(3, 3) = steb_cfg_->optim.weight_acc_theta_minimize;

  for (int i = 0; i < n - 3; ++i) {
    EdgeAcceleration *acceleration_edge = new EdgeAcceleration;
    acceleration_edge->setVertex(0, steb_.getPoseVertex(i));
    acceleration_edge->setVertex(1, steb_.getPoseVertex(i + 1));
    acceleration_edge->setVertex(2, steb_.getPoseVertex(i + 2));
    acceleration_edge->setVertex(3, steb_.getPoseVertex(i + 3));
    acceleration_edge->setInformation(information);
    acceleration_edge->setSTEBConfig(*steb_cfg_);
    optimizer_->addEdge(acceleration_edge);

  }
 
}

void STEBPlanner::AddEdgesTimeOptimal() {
  if (steb_cfg_->optim.weight_time_minimize == 0 && steb_cfg_->optim.weight_time_ascend_limit == 0)
    return; // if weight equals zero skip adding edges!

  Eigen::Matrix<double, 2, 2> information;
  information.fill(0);
  information(0, 0) = steb_cfg_->optim.weight_time_minimize;
  information(1, 1) = steb_cfg_->optim.weight_time_ascend_limit;

  for (int i = 0; i < steb_.sizePoses() - 1; ++i) {
    EdgeTimeOptimal *timeoptimal_edge = new EdgeTimeOptimal;
    timeoptimal_edge->setVertex(0, steb_.getPoseVertex(i));
    timeoptimal_edge->setVertex(1, steb_.getPoseVertex(i + 1));
    timeoptimal_edge->setInformation(information);
    timeoptimal_edge->setSTEBConfig(*steb_cfg_);
    optimizer_->addEdge(timeoptimal_edge);
  }
}

void STEBPlanner::AddEdgesShortestPath()
{
  if (steb_cfg_->optim.weight_shortest_path == 0)
    return; // if weight equals zero skip adding edges!

  Eigen::Matrix<double, 1, 1> information;
  information.fill(steb_cfg_->optim.weight_shortest_path);

//  std::cout << "AddEdgesShortestPath, teb size: " << steb_.sizePoses()<<std::endl;
  for (int i = 0; i < steb_.sizePoses() - 1; ++i)
  {
    EdgeShortestPath *shortest_path_edge = new EdgeShortestPath;
    shortest_path_edge->setVertex(0, steb_.getPoseVertex(i));
    shortest_path_edge->setVertex(1, steb_.getPoseVertex(i + 1));
    shortest_path_edge->setInformation(information);
    shortest_path_edge->setSTEBConfig(*steb_cfg_);
    optimizer_->addEdge(shortest_path_edge);
  }
}



void STEBPlanner::AddEdgesKinematicsCarlike() {
  if (steb_cfg_->optim.weight_kinematics_nh == 0 &&
      steb_cfg_->optim.weight_kinematics_turning_radius_limit == 0)
    return; // if weight equals zero skip adding edges!
  
  // create edge for satisfiying kinematic constraints
  Eigen::Matrix<double, 2, 2> information_kinematics;
  information_kinematics.fill(0.0);
  information_kinematics(0, 0) = steb_cfg_->optim.weight_kinematics_nh;
  information_kinematics(1, 1) = steb_cfg_->optim.weight_kinematics_turning_radius_limit;

  for (int i = 0; i < steb_.sizePoses() - 2; i++) 
  {
    EdgeKinematicsCarlike *kinematics_edge = new EdgeKinematicsCarlike;
    kinematics_edge->setVertex(0, steb_.getPoseVertex(i));
    kinematics_edge->setVertex(1, steb_.getPoseVertex(i + 1));
    kinematics_edge->setVertex(2, steb_.getPoseVertex(i + 2));
    kinematics_edge->setInformation(information_kinematics);
    kinematics_edge->setSTEBConfig(*steb_cfg_);
    optimizer_->addEdge(kinematics_edge);
  }
}




bool STEBPlanner::hasDiverged() const {
  // Early returns if divergence detection is not active
  if (!steb_cfg_->optim.divergence_detection_enable)
    return false;

  auto stats_vector = optimizer_->batchStatistics();

  // No statistics yet
  if (stats_vector.empty())
    return false;

  // Grab the statistics of the final iteration
  const auto last_iter_stats = stats_vector.back();

  return last_iter_stats.chi2 >
         steb_cfg_->optim.divergence_detection_max_chi_squared;
}



void STEBPlanner::computeCurrentCostSTEB()
{

  if (!optimizer_->edges().empty() && !optimizer_->vertices().empty())
  {
    optimizer_->computeInitialGuess();
    cost_ = 0;

    for (std::vector<g2o::OptimizableGraph::Edge *>::const_iterator it = optimizer_->activeEdges().begin();
         it != optimizer_->activeEdges().end(); it++)
    {
      double cur_cost = (*it)->chi2();
      cost_ += cur_cost;

      if (false)
      {
        if (dynamic_cast<EdgeViaPoint *>(*it) != nullptr)
        {
          std::cout <<OUT_BLUE<< "2. EdgeViaPoint cost:              " << cur_cost <<OUT_RESET<< std::endl;
        }
        else if (dynamic_cast<EdgeKinematicsCarlike *>(*it) != nullptr)
        {
          std::cout <<OUT_BLUE<< "3. EdgeKinematicsSTEBCarlike cost: " << cur_cost <<OUT_RESET<< std::endl;
        }
        else if (dynamic_cast<EdgeTimeOptimal *>(*it) != nullptr)
        {
          std::cout <<OUT_BLUE<< "4. STEBEdgeTimeOptimal cost:       " << cur_cost <<OUT_RESET<< std::endl;
        }
        else if (dynamic_cast<EdgeVelocity *>(*it) != nullptr)
        {
          std::cout <<OUT_BLUE<< "5. EdgeSTEBVelocity cost:          " << cur_cost <<OUT_RESET<< std::endl;
        }
        else if (dynamic_cast<EdgeAcceleration *>(*it) != nullptr)
        {
          std::cout <<OUT_BLUE<< "6. EdgeSTEBAcceleration cost:      " << cur_cost <<OUT_RESET<< std::endl;
        }
        else if (dynamic_cast<EdgeShortestPath *>(*it) != nullptr )
        {
          std::cout <<OUT_BLUE<< "7. EdgeShortestPath cost:          " << cur_cost <<OUT_RESET<< std::endl;
        }
      }
    }
  }


}

void STEBPlanner::extractVelocity(const PoseSE2 &pose1,
                                        const PoseSE2 &pose2, double dt,
                                        double &vx, double &vy,
                                        double &omega) const {
  if (dt == 0) {
    vx = 0;
    vy = 0;
    omega = 0;
    return;
  }

  Eigen::Vector2d deltaS = pose2.position() - pose1.position();

  Eigen::Vector2d conf1dir(cos(pose1.theta()), sin(pose1.theta()));
  // translational velocity
  double dir = deltaS.dot(conf1dir);
  vx = (double)g2o::sign(dir) * deltaS.norm() / dt;
  vy = 0;
  
  // rotational velocity
  double orientdiff = g2o::normalize_theta(pose2.theta() - pose1.theta());
  omega = orientdiff / dt;
}


void STEBPlanner::getFullTrajectory(TrajectoryPointsContainer &trajectory)
{
  int n = steb_.sizePoses();
  if (n == 0)
    return;

  if (steb_cfg_->trajectory.final_trajectory_bezier_smooth)
  {
    // smoothed by bezier curve 
    int numPoints = n; 

    std::vector<Eigen::Vector3d> controlPoints;
    for (int i = 0; i < n; ++i)
    {
      controlPoints.emplace_back(Eigen::Vector3d{steb_.getPose(i).x(), steb_.getPose(i).y(), steb_.getPose(i).z()});
    }

    std::vector<Eigen::Vector3d> bezierCurve = generateBezierCurve(controlPoints, numPoints);

    trajectory.clear();
    trajectory.resize(bezierCurve.size());

    for (size_t i = 0; i < bezierCurve.size(); ++i) {
      Eigen::Vector4d &point = trajectory[i];
      point = {bezierCurve.at(i).x(), bezierCurve.at(i).y(), 0.0, bezierCurve.at(i).z()};
      // std::cout << "Point: (" << point.x() << ", " << point.y() << ", " << point.w()<< ")\n";
    }
  }
  else
  {
    // double curr_time = 0;
    trajectory.clear();
    trajectory.resize(n);
    for (int i = 0; i < n; ++i)
    {
      Eigen::Vector4d &point = trajectory[i];
      point = {steb_.getPose(i).x(), steb_.getPose(i).y(), 0.0, steb_.getPose(i).z()};
    }
  }
}

void STEBPlanner::publishDebugMarker()
{
  visualization_msgs::msg::MarkerArray debug_markers;
  visualization_msgs::msg::Marker vertex_to_obs_marker;
  vertex_to_obs_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  vertex_to_obs_marker.action = visualization_msgs::msg::Marker::ADD;
  vertex_to_obs_marker.header.frame_id = "base_link";
  vertex_to_obs_marker.header.stamp = rviz_pub_time_;
  vertex_to_obs_marker.color.a = 0.8;
  vertex_to_obs_marker.color.r = 1.0;
  vertex_to_obs_marker.color.g = 0.5;
  vertex_to_obs_marker.color.b = 0.0;
  vertex_to_obs_marker.scale.x = 0.05;
  vertex_to_obs_marker.scale.y = 0.05;
  vertex_to_obs_marker.scale.z = 0.05;
  vertex_to_obs_marker.id = 10;

  // steb points
  visualization_msgs::msg::Marker via_point_marker;
  via_point_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  via_point_marker.action = visualization_msgs::msg::Marker::ADD;
  via_point_marker.header.frame_id = "base_link";
  via_point_marker.header.stamp = vertex_to_obs_marker.header.stamp;
  via_point_marker.color.a = 0.8;
  via_point_marker.color.r = 1.0;
  via_point_marker.color.g = 0.0;
  via_point_marker.color.b = 0.6;
  via_point_marker.scale.x = 0.2;
  via_point_marker.scale.y = 0.2;
  via_point_marker.scale.z = 0.2;
  via_point_marker.id = 11;


  std::cout << ">>>>> obstacles_per_vertex_ size : " << obstacles_per_vertex_.size() <<std::endl;
  for (int i=0; i<(int)obstacles_per_vertex_.size(); ++i)
  {
    if (!obstacles_per_vertex_.at(i).empty())
    {
      geometry_msgs::msg::Point vertex_point;
      vertex_point.x = steb_.getPosesReadOnly().at(i)->x();
      vertex_point.y = steb_.getPosesReadOnly().at(i)->y();
      vertex_point.z = steb_.getPosesReadOnly().at(i)->t();
      via_point_marker.points.push_back(vertex_point);
      for (auto& obstacle : obstacles_per_vertex_.at(i))
      {
        vertex_to_obs_marker.points.push_back(vertex_point);
        geometry_msgs::msg::Point obstacle_point;
        obstacle_point.x = obstacle->getCentroid().x();
        obstacle_point.y = obstacle->getCentroid().y();
        obstacle_point.z = obstacle->getEmergenceTime();
        vertex_to_obs_marker.points.push_back(obstacle_point);
      }
    }
  }
  debug_markers.markers.push_back(vertex_to_obs_marker);
  debug_markers.markers.push_back(via_point_marker);


  // steb points resized
  visualization_msgs::msg::Marker via_point_resized_marker;
  via_point_resized_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  via_point_resized_marker.action = visualization_msgs::msg::Marker::ADD;
  via_point_resized_marker.header.frame_id = "base_link";
  via_point_resized_marker.header.stamp = vertex_to_obs_marker.header.stamp;
  via_point_resized_marker.color.a = 0.5;
  via_point_resized_marker.color.r = 1.0;
  via_point_resized_marker.color.g = 0.0;
  via_point_resized_marker.color.b = 0.6;
  via_point_resized_marker.scale.x = 0.2;
  via_point_resized_marker.scale.y = 0.2;
  via_point_resized_marker.scale.z = 0.2;
  via_point_resized_marker.id = 13;

  for (auto& pose : steb_.getPosesReadOnly())
  {
    geometry_msgs::msg::Point resized_pose;
    resized_pose.x = pose->x();
    resized_pose.y = pose->y();
    resized_pose.z = pose->t();
    via_point_resized_marker.points.push_back(resized_pose);
  }
  debug_markers.markers.push_back(via_point_resized_marker);

  debug_viz_pub_->publish(debug_markers);
}

void STEBPlanner::publishViaAreaDebugMarker()
{
  visualization_msgs::msg::MarkerArray debug_markers;
  // via_area
  visualization_msgs::msg::Marker via_area_marker;
  via_area_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  via_area_marker.action = visualization_msgs::msg::Marker::ADD;
  via_area_marker.header.frame_id = "base_link";
  via_area_marker.header.stamp = rviz_pub_time_;
  via_area_marker.color.a = 0.4;
  via_area_marker.color.r = 1.0;
  via_area_marker.color.g = 0.0;
  via_area_marker.color.b = 0.0;
  via_area_marker.scale.x = 0.1;
  via_area_marker.scale.y = 0.1;
  via_area_marker.scale.z = 0.1;
  via_area_marker.id = 12;

  std::vector<geometry_msgs::msg::Point> right_bounds;
  std::vector<geometry_msgs::msg::Point> left_bounds;
  for (int k=0; k<(int)via_area_.size(); ++k)
  {
    tf2::Transform tf_base_link_to_via_point;
    tf_base_link_to_via_point.setOrigin(tf2::Vector3(via_area_.at(k)->getViaPoint().x(),
                                                     via_area_.at(k)->getViaPoint().y(), 0.0));
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, via_area_.at(k)->getViaPoint().z());
    tf_base_link_to_via_point.setRotation(quaternion);

    // right bound
    {
      tf2::Transform tf_via_point_to_right_bound = tf2::Transform::getIdentity();
      tf_via_point_to_right_bound.setOrigin(tf2::Vector3(0.0, via_area_.at(k)->getRightBound(), 0.0));
      tf2::Transform tf_base_link_to_right_bound = tf_base_link_to_via_point * tf_via_point_to_right_bound;

      geometry_msgs::msg::Point right_bound_point;
      right_bound_point.x = tf_base_link_to_right_bound.getOrigin().x();
      right_bound_point.y = tf_base_link_to_right_bound.getOrigin().y();
      right_bound_point.z = tf_base_link_to_right_bound.getOrigin().z();
      right_bounds.push_back(right_bound_point);
    }

    // left bound
    {
      tf2::Transform tf_via_point_to_left_bound = tf2::Transform::getIdentity();
      tf_via_point_to_left_bound.setOrigin(tf2::Vector3(0.0, via_area_.at(k)->getLeftBound(), 0.0));
      tf2::Transform tf_base_link_to_left_bound = tf_base_link_to_via_point * tf_via_point_to_left_bound;

      geometry_msgs::msg::Point left_bound_point;
      left_bound_point.x = tf_base_link_to_left_bound.getOrigin().x();
      left_bound_point.y = tf_base_link_to_left_bound.getOrigin().y();
      left_bound_point.z = tf_base_link_to_left_bound.getOrigin().z();
      left_bounds.push_back(left_bound_point);
    }
  }

  for (int i=0; i<(int)right_bounds.size(); ++i)
  {
    via_area_marker.points.push_back(right_bounds.at(i));
  }
  for (int j=(int)left_bounds.size()-1; j>=0; --j)
  {
    via_area_marker.points.push_back(left_bounds.at(j));
  }
  via_area_marker.points.push_back(right_bounds.front());
  debug_markers.markers.push_back(via_area_marker);

  via_area_debug_rviz_pub_->publish(debug_markers);
}

} // namespace steb_planner
