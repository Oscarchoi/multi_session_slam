// This software is licensed under the BSD 2-Clause License.
// Copyright (c) 2020, Ryohei Sasaki. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted under the following conditions:
//
// 1. Redistributions of source code must retain the copyright notice,
//    license conditions, and disclaimer.
// 2. Redistributions in binary form must reproduce the above information in
//    the documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.
// For full license details, see the LICENSE file in the project root.

#include "multi_session_slam/graph_slam/graph_slam.h"

#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pclomp/gicp_omp.h>
#include <pclomp/ndt_omp.h>
#include <pclomp/voxel_grid_covariance_omp.h>
#include <pclomp/gicp_omp_impl.hpp>
#include <pclomp/ndt_omp_impl.hpp>
#include <pclomp/voxel_grid_covariance_omp_impl.hpp>

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/types/slam3d/edge_se3_pointxyz.h"
#include "g2o/types/slam3d/parameter_se3_offset.h"
#include "g2o/types/slam3d/se3quat.h"
#include "g2o/types/slam3d/vertex_pointxyz.h"
#include "g2o/types/slam3d/vertex_se3.h"

namespace multi_session_slam {

GraphSlam::GraphSlam(rclcpp::Node* node) : node_(node) {
  node_->get_parameter("registration_method", registration_method_);
  node_->get_parameter("threshold_loop_closure_score",
                       threshold_loop_closure_score_);
  node_->get_parameter("loop_closure_search_num", loop_closure_search_num_);
  node_->get_parameter("range_of_searching_loop_closure",
                       range_of_searching_loop_closure_);
  node_->get_parameter("num_adjacent_pose_constraints",
                       num_adjacent_pose_constraints_);
  node_->get_parameter("voxel_leaf_size", voxel_leaf_size_);
  node_->get_parameter("debug", is_debug_);

  double ndt_resolution;
  int ndt_num_threads;
  node_->get_parameter("ndt_resolution", ndt_resolution);
  node_->get_parameter("ndt_num_threads", ndt_num_threads);

  grid_filter_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_,
                           voxel_leaf_size_);

  if (registration_method_ == "NDT") {
    pclomp::NormalDistributionsTransform<PointType, PointType>::Ptr ndt(
        new pclomp::NormalDistributionsTransform<PointType, PointType>());
    ndt->setMaximumIterations(100);
    ndt->setResolution(ndt_resolution);
    ndt->setTransformationEpsilon(0.01);
    ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    if (ndt_num_threads > 0) {
      ndt->setNumThreads(ndt_num_threads);
    }
    registration_ = ndt;
  } else {
    pclomp::GeneralizedIterativeClosestPoint<PointType, PointType>::Ptr gicp(
        new pclomp::GeneralizedIterativeClosestPoint<PointType, PointType>());
    gicp->setMaxCorrespondenceDistance(30);
    gicp->setMaximumIterations(100);
    gicp->setTransformationEpsilon(1e-8);
    gicp->setEuclideanFitnessEpsilon(1e-6);
    gicp->setRANSACIterations(0);
    registration_ = gicp;
  }
}

GraphSlam::~GraphSlam() {}

void GraphSlam::RegisterPointCloud(const PointCloudType::Ptr& input_cloud,
                                   Eigen::Matrix4f pose) {
  cloud_array_.push_back(input_cloud);
  pose_array_.push_back(pose);
}

GraphSlam::PointCloudType::Ptr GraphSlam::GenerateMapFromClouds() {
  bool found = SearchLoopClosure();
  if (!found) {
    RCLCPP_ERROR(node_->get_logger(),
                 "No loop closure candidate found. Generate map anyway.");
  }
  return DoPoseAdjustment();
}

bool GraphSlam::SearchLoopClosure() {
  if (cloud_array_.size() == 0) {
    RCLCPP_ERROR(node_->get_logger(), "No cloud received.");
    return false;
  }
  if (cloud_array_.size() != pose_array_.size()) {
    RCLCPP_ERROR(node_->get_logger(), "Invalid array size: %ld vs %ld",
                 cloud_array_.size(), pose_array_.size());
    return false;
  }

  // FIXME(wy.choi): Since we check for loop closure at the end, we need to
  // iterate through double loops to find all possible loop closure candidates.
  size_t cloud_count = cloud_array_.size();
  auto& latest_cloud = cloud_array_[cloud_count - 1];
  auto& latest_pose = pose_array_[cloud_count - 1];
  auto latest_position = latest_pose.block<3, 1>(0, 3);

  double min_fitness_score = std::numeric_limits<double>::max();
  registration_->setInputSource(latest_cloud);

  bool has_candidate = false;
  size_t min_index = 0;
  double min_distance = std::numeric_limits<double>::max();

  // find the smallest index capable of generating loop closure
  for (int idx = 0; idx < cloud_count - 1; idx++) {
    auto& pose = pose_array_[idx];
    auto position = pose.block<3, 1>(0, 3);

    double distance = (position - latest_position).norm();
    if (distance < range_of_searching_loop_closure_) {
      has_candidate = true;
      if (distance < min_distance) {
        min_distance = distance;
        min_index = idx;
      }
    }
  }

  if (!has_candidate) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Has no candidate to generate loop closure");
    return false;
  }

  PointCloudType::Ptr loop_closure_target_cloud(new PointCloudType);
  for (int idx = min_index - loop_closure_search_num_;
       idx < min_index + loop_closure_search_num_; idx++) {
    if (idx < 0 || idx >= cloud_count) {
      continue;
    }
    *loop_closure_target_cloud += *(cloud_array_[idx]);
  }

  PointCloudType::Ptr filtered_cloud(new PointCloudType);
  grid_filter_.setInputCloud(loop_closure_target_cloud);
  grid_filter_.filter(*filtered_cloud);
  registration_->setInputTarget(filtered_cloud);
  PointCloudType::Ptr output_cloud(new PointCloudType);
  registration_->align(*output_cloud);

  // check whether the loop closure is good to be generated
  double fitness_score = registration_->getFitnessScore();
  if (fitness_score > threshold_loop_closure_score_) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Fitness score is higher(worse) than the threshold: %f",
                 fitness_score);
    return false;
  }

  LoopEdge loop_edge;
  loop_edge.ids = {min_index, cloud_count - 1};

  Eigen::Isometry3d from =
      Eigen::Isometry3d(pose_array_[min_index].cast<double>());
  Eigen::Isometry3d to =
      Eigen::Isometry3d(registration_->getFinalTransformation().cast<double>() *
                        latest_pose.matrix().cast<double>());
  loop_edge.relative_pose = Eigen::Isometry3d(from.inverse() * to);
  loop_edges_.push_back(loop_edge);

  return true;
}

GraphSlam::PointCloudType::Ptr GraphSlam::DoPoseAdjustment() {
  g2o::SparseOptimizer optimizer;
  optimizer.setVerbose(false);
  std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linear_solver =
      g2o::make_unique<
          g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
  g2o::OptimizationAlgorithmLevenberg* solver =
      new g2o::OptimizationAlgorithmLevenberg(
          g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linear_solver)));

  optimizer.setAlgorithm(solver);

  size_t cloud_count = cloud_array_.size();
  Eigen::Matrix<double, 6, 6> info_mat =
      Eigen::Matrix<double, 6, 6>::Identity();

  // add adjacent node edges
  size_t adjacent_edge_count = 0;
  for (int idx = 0; idx < cloud_count; idx++) {
    Eigen::Isometry3d pose(pose_array_[idx].cast<double>());
    g2o::VertexSE3* vertex_se3 = new g2o::VertexSE3();
    vertex_se3->setId(idx);
    vertex_se3->setEstimate(pose);
    if (idx == 0) {
      vertex_se3->setFixed(true);
    }
    optimizer.addVertex(vertex_se3);

    if (idx > num_adjacent_pose_constraints_) {
      for (int j = 0; j < num_adjacent_pose_constraints_; j++) {
        Eigen::Isometry3d pre_pose(
            pose_array_[idx - num_adjacent_pose_constraints_ + j]
                .cast<double>());
        Eigen::Isometry3d relative_pose = pre_pose.inverse() * pose;
        g2o::EdgeSE3* edge_se3 = new g2o::EdgeSE3();
        edge_se3->setMeasurement(relative_pose);
        edge_se3->setInformation(info_mat);
        edge_se3->vertices()[0] =
            optimizer.vertex(idx - num_adjacent_pose_constraints_ + j);
        edge_se3->vertices()[1] = optimizer.vertex(idx);
        optimizer.addEdge(edge_se3);
        adjacent_edge_count++;
      }
    }
  }
  RCLCPP_INFO(node_->get_logger(), "Found %d adjacent edges.",
              adjacent_edge_count);

  // add loop edge
  for (auto loop_edge : loop_edges_) {
    g2o::EdgeSE3* edge_se3 = new g2o::EdgeSE3();
    edge_se3->setMeasurement(loop_edge.relative_pose);
    edge_se3->setInformation(info_mat);
    edge_se3->vertices()[0] = optimizer.vertex(loop_edge.ids.first);
    edge_se3->vertices()[1] = optimizer.vertex(loop_edge.ids.second);
    optimizer.addEdge(edge_se3);
  }
  RCLCPP_INFO(node_->get_logger(), "Found %d loop edges.", loop_edges_.size());

  RCLCPP_INFO(node_->get_logger(),
              "Start optimize loop closure and adjust pointclouds...");
  optimizer.initializeOptimization();
  optimizer.optimize(10);

  PointCloudType::Ptr output_cloud(new PointCloudType);
  for (int idx = 0; idx < cloud_count; ++idx) {
    g2o::VertexSE3* vertex_se3 =
        static_cast<g2o::VertexSE3*>(optimizer.vertex(idx));
    Eigen::Affine3d se3 = vertex_se3->estimate();

    Eigen::Matrix4f corrected_transform =
        se3.matrix().cast<float>() * pose_array_[idx].inverse();

    PointCloudType transformed_cloud;
    pcl::transformPointCloud(*cloud_array_[idx], transformed_cloud,
                             corrected_transform);
    *output_cloud += transformed_cloud;
  }

  // filter output clouds
  PointCloudType::Ptr filtered_output_cloud(new PointCloudType);
  grid_filter_.setInputCloud(output_cloud);
  grid_filter_.filter(*filtered_output_cloud);

  return filtered_output_cloud;
}

}  // namespace multi_session_slam