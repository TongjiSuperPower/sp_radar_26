#ifndef ICP_NODE_HPP
#define ICP_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Geometry>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <string>
#include <atomic>
#include <mutex>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

/// ————————————————————————————————————————————————
#include <small_gicp/pcl/pcl_point.hpp>
#include <small_gicp/pcl/pcl_point_traits.hpp>
#include <small_gicp/pcl/pcl_registration.hpp>
#include <small_gicp/util/downsampling_omp.hpp>
#include <small_gicp/benchmark/read_points.hpp>
#include "tf2_eigen/tf2_eigen.hpp"
using namespace small_gicp;
/// ————————————————————————————————————————————————

#include "radar_msgs/msg/game_status.hpp"

class ICPNode : public rclcpp::Node
{
public:
  ICPNode();

private:
  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void gameStatusCallback(const radar_msgs::msg::GameStatus::SharedPtr msg);
  Eigen::Matrix4f genTransformation(const std::vector<float> &params);
  void printEularTvector(const Eigen::Isometry3d &result);
  void update_best_result(Eigen::Isometry3d result_t);
  void icp();
  void small_gicp();
  void init_map2lidar_();
  void broadcast_transform();
  void update_map2lidar(Eigen::Isometry3d icp_result);
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> filter_points_in_map(std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud);
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> filter_points_in_lidar(std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud);

  std::string pcd_target_path_;
  std::string topic_pointcloud_;
  std::string topic_game_status_;
  bool use_best_result_;
  bool use_small_gicp_;
  float voxel_grid_size_;
  double max_score_;
  double max_error_;
  int num_neighbors_;
  int num_threads_;
  double max_dist_sq_;
  std::vector<float> init_params_;

  double icp_score_;
  double icp_error_;
  double best_score_;
  
  rclcpp::TimerBase::SharedPtr timer_broadcast_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_pointcloud_;
  rclcpp::Subscription<radar_msgs::msg::GameStatus>::SharedPtr subscription_game_status_;
  radar_msgs::msg::GameStatus game_status_ref_;
  uint8_t game_type_;
  uint8_t game_process_;
  uint16_t stage_remain_time_;
  
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_target_{new pcl::PointCloud<pcl::PointXYZI>};
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_source_{new pcl::PointCloud<pcl::PointXYZI>};
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_target_filtered_{new pcl::PointCloud<pcl::PointXYZI>}; // 场地点云
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_source_filtered_{new pcl::PointCloud<pcl::PointXYZI>}; // 雷达点云
  std::shared_ptr<small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>> register_;
  pcl::PointCloud<pcl::PointCovariance>::Ptr target_;
  pcl::PointCloud<pcl::PointCovariance>::Ptr source_;
  std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> target_tree_;
  std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> source_tree_;
  Eigen::Isometry3d result_t_;
  Eigen::Isometry3d previous_result_t_;
  Eigen::Isometry3d best_result_t_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  geometry_msgs::msg::TransformStamped map2lidar_;

  Eigen::Matrix4f transformation_;
  bool filter_flag_, listen_game_status_;
};

#endif // ICP_NODE_HPP
