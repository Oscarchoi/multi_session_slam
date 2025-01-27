#ifndef MULTI_SESSION_SLAM_GRAPH_SLAM_GRAPH_SLAM_H_
#define MULTI_SESSION_SLAM_GRAPH_SLAM_GRAPH_SLAM_H_

#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/registration.h>

namespace multi_session_slam {

class GraphSlam {
 public:
  using PointType = pcl::PointXYZI;
  using PointCloudType = pcl::PointCloud<PointType>;

  struct LoopEdge {
    std::pair<int, int> ids;
    Eigen::Isometry3d relative_pose;
  };

  GraphSlam(rclcpp::Node* node);
  ~GraphSlam();

  void RegisterPointCloud(const PointCloudType::Ptr& input_cloud,
                          Eigen::Matrix4f pose);
  void Finish();

 private:
  bool SearchLoopClosure();

  PointCloudType::Ptr DoPoseAdjustment();

  pcl::Registration<PointType, PointType>::Ptr registration_;
  std::vector<PointCloudType::Ptr> cloud_array_;
  std::vector<Eigen::Matrix4f> pose_array_;
  std::vector<LoopEdge> loop_edges_;
  pcl::VoxelGrid<PointType> grid_filter_;

  // parameters
  std::string registration_method_;
  double voxel_leaf_size_;
  double threshold_loop_closure_score_;
  double range_of_searching_loop_closure_;
  double loop_closure_search_num_;
  int num_adjacent_pose_constraints_;
  bool is_debug_ = false;

  rclcpp::Node* node_;  // not owned
};

}  // namespace multi_session_slam

#endif  // MULTI_SESSION_SLAM_GRAPH_SLAM_GRAPH_SLAM_H_