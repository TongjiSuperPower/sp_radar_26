#include "relocalization/icp.hpp"
using std::placeholders::_1;

ICPNode::ICPNode() : Node("icp_node")
{
  // 初始化配置文件及重定位参数
  std::string config_file;
  config_file = "src/main/relocalization/config/config.yaml";
  const auto config = YAML::LoadFile(config_file);
  pcd_target_path_ = config["pcd_target_path"].as<std::string>();
  topic_pointcloud_ = config["topic_pointcloud"].as<std::string>();
  topic_game_status_ = config["topic_game_status"].as<std::string>();
  use_best_result_ = config["use_best_result"].as<bool>();
  use_small_gicp_ = config["use_small_gicp"].as<bool>();
  voxel_grid_size_ = config["voxel_grid_size"].as<float>();
  max_score_ = config["max_score"].as<double>();
  max_error_ = config["max_error"].as<double>();
  num_neighbors_ = config["num_neighbors"].as<int>();
  num_threads_ = config["num_threads"].as<int>();
  max_dist_sq_ = config["max_dist_sq"].as<double>();
  init_params_ = config["init_params"].as<std::vector<float>>();
  init_map2lidar_();

  best_score_ = INT_MAX;
  
  // 加载目标点云
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_target_path_, *cloud_target_) == -1)
  {
    RCLCPP_ERROR(this->get_logger(), "Couldn't read target PCD file.");
    return;
  }
  // 目标点云预处理
  if (use_small_gicp_)
  { 
    target_ = small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZI>, pcl::PointCloud<pcl::PointCovariance>>(*cloud_target_, voxel_grid_size_);
    small_gicp::estimate_covariances_omp(*target_, num_neighbors_, num_threads_);
    target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(target_, small_gicp::KdTreeBuilderOMP(num_threads_));
    register_ = std::make_shared<small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>();
  }
  else
  {
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
    voxel_grid.setLeafSize(voxel_grid_size_, voxel_grid_size_, voxel_grid_size_);
    voxel_grid.setInputCloud(cloud_target_);
    voxel_grid.filter(*cloud_target_filtered_);
  }

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  timer_broadcast_ =
      this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ICPNode::broadcast_transform, this));
  subscription_pointcloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      topic_pointcloud_, 10, std::bind(&ICPNode::topic_callback, this, _1));
  subscription_game_status_ = this->create_subscription<radar_msgs::msg::GameStatus>(
      topic_game_status_, 10, std::bind(&ICPNode::gameStatusCallback, this, _1));
}

void ICPNode::gameStatusCallback(const radar_msgs::msg::GameStatus::SharedPtr msg)
{
  game_status_ref_ = *msg;
  game_type_ = game_status_ref_.game_type;        // 1Byte bit[0:3]
  game_process_ = game_status_ref_.game_progress; // 1Byte bit[4:7]
  stage_remain_time_ = game_status_ref_.stage_remain_time;
  RCLCPP_INFO(this->get_logger(), "Game Type: %d, Game Progress: %d, Stage Remain Time: %d",
              game_type_, game_process_, stage_remain_time_);
}

void ICPNode::topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // if(game_process_ == 0)
  //   std::cout << "help" << std::endl;
  //   return; // 比赛未开始
  // if(game_process_ == 1 && stage_remain_time_ >= 10)
  //   return; // 准备阶段剩余时间大于等于10s
  RCLCPP_INFO(this->get_logger(), "Received point cloud message");
    pcl::fromROSMsg(*msg, *cloud_source_);
    // cloud_source_= filter_points_in_lidar(cloud_source_);  // 收敛后会有概率波动
    // cloud_source_ = filter_points_in_map(cloud_source_);
    
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
    voxel_grid.setLeafSize(voxel_grid_size_, voxel_grid_size_, voxel_grid_size_);
    voxel_grid.setInputCloud(cloud_source_);
    voxel_grid.filter(*cloud_source_filtered_);

    auto start = std::chrono::steady_clock::now();
    if (use_small_gicp_)
      small_gicp();
    else
      icp();
    auto end = std::chrono::steady_clock::now();
    auto diff = end - start;
    RCLCPP_INFO(this->get_logger(), "GICP time: %f ms", std::chrono::duration<double, std::milli>(diff).count());
    if (use_best_result_)
    {
      update_best_result(result_t_);
      update_map2lidar(best_result_t_);
    }
    else if (icp_score_ < max_score_)
      update_map2lidar(result_t_);
    printEularTvector(result_t_);
}

Eigen::Matrix4f ICPNode::genTransformation(const std::vector<float> &params)
{
  Eigen::Vector3f r(params[0], params[1], params[2]);
  Eigen::Vector3f t(params[3], params[4], params[5]);
  Eigen::AngleAxisf init_rotation_x(r.x(), Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf init_rotation_y(r.y(), Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf init_rotation_z(r.z(), Eigen::Vector3f::UnitZ());
  Eigen::Translation3f init_translation(t.x(), t.y(), t.z());
  return (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();
}

void ICPNode::printEularTvector(const Eigen::Isometry3d &result)
{
  Eigen::Vector3d translation = result.translation();
  Eigen::Vector3d eulerAngles = result.rotation().eulerAngles(2, 1, 0) * (180.0 / M_PI);
  std::cout << "rotate: " << std::endl
            << eulerAngles << std::endl;
  std::cout << "p: " << std::endl
            << translation << std::endl;
}
void ICPNode::update_best_result(Eigen::Isometry3d result_t)
{
  if (use_small_gicp_)
    if (icp_error_ < best_score_)
      best_score_ = icp_error_;
  else
    if (icp_score_ < best_score_)
      best_score_ = icp_score_;
  best_result_t_ = result_t;
}

void ICPNode::icp()
{ 
  pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
  icp.setTransformationEpsilon(1e-10);
  icp.setMaxCorrespondenceDistance(2.0);
  icp.setMaximumIterations(50);
  icp.setInputSource(cloud_source_filtered_); 
  icp.setInputTarget(cloud_target_filtered_); // icp结果是从地图坐标系到雷达坐标系的齐次变换矩阵
  pcl::PointCloud<pcl::PointXYZI> final_cloud;
  Eigen::Matrix4f guess = previous_result_t_.matrix().cast<float>();
  icp.align(final_cloud, guess);
  if (icp.hasConverged())
  {
    icp_score_ = icp.getFitnessScore();
    RCLCPP_INFO(this->get_logger(), "ICP has converged with score: %f", icp.getFitnessScore());
  }
  else
    RCLCPP_WARN(this->get_logger(), "ICP did not converge.");
  transformation_ = icp.getFinalTransformation();
  // Eigen::Matrix4f transformation转化为Eigen::Isometry3d
  result_t_.matrix() = transformation_.cast<double>();
  previous_result_t_ = result_t_;
}

void ICPNode::small_gicp()
{
  source_ = small_gicp::voxelgrid_sampling_omp<
      pcl::PointCloud<pcl::PointXYZI>, pcl::PointCloud<pcl::PointCovariance>>(
      *cloud_source_filtered_, voxel_grid_size_);
  small_gicp::estimate_covariances_omp(*source_, num_neighbors_, num_threads_);
  source_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
      source_, small_gicp::KdTreeBuilderOMP(num_threads_));
  register_->reduction.num_threads = num_threads_;
  register_->rejector.max_dist_sq = max_dist_sq_;
  struct small_gicp::RegistrationResult result = register_->align(*target_, *source_, *target_tree_, previous_result_t_);
  if (result.converged)
  {
    RCLCPP_INFO(this->get_logger(), "GICP converge.");
    result_t_ = result.T_target_source;
    previous_result_t_ = result.T_target_source;
    icp_error_ = result.error;
    RCLCPP_INFO(this->get_logger(), "GICP has converged with error: %f", result.error);
  }
  else
    RCLCPP_WARN(this->get_logger(), "GICP did not converge and error is: %f", result.error);
}

void ICPNode::init_map2lidar_()
{
  map2lidar_.header.stamp = this->get_clock()->now();
  map2lidar_.header.frame_id = "map";
  map2lidar_.child_frame_id = "lidar_frame";
  map2lidar_.transform.translation.x = init_params_[0];
  map2lidar_.transform.translation.y = init_params_[1];
  map2lidar_.transform.translation.z = init_params_[2];
  tf2::Quaternion q;
  q.setRPY(init_params_[5] * M_PI / 180.0, init_params_[4] * M_PI / 180.0, init_params_[3] * M_PI / 180.0);
  map2lidar_.transform.rotation.x = q.x();
  map2lidar_.transform.rotation.y = q.y();
  map2lidar_.transform.rotation.z = q.z();
  map2lidar_.transform.rotation.w = q.w();
  previous_result_t_ = tf2::transformToEigen(map2lidar_.transform);
  result_t_ = tf2::transformToEigen(map2lidar_.transform);
  transformation_ = previous_result_t_.matrix().cast<float>();
}

void ICPNode::broadcast_transform()
{
  // 获取rosbag中的时间戳
  map2lidar_.header.stamp = this->get_clock()->now();
  tf_broadcaster_->sendTransform(map2lidar_);
}

void ICPNode::update_map2lidar(Eigen::Isometry3d icp_result)
{
  auto icp_result_lidar2map = icp_result;

  map2lidar_.transform.translation.x = icp_result_lidar2map.translation().x();
  map2lidar_.transform.translation.y = icp_result_lidar2map.translation().y();
  map2lidar_.transform.translation.z = icp_result_lidar2map.translation().z();
  Eigen::Quaterniond q(icp_result_lidar2map.rotation());
  map2lidar_.transform.rotation.x = q.x();
  map2lidar_.transform.rotation.y = q.y();
  map2lidar_.transform.rotation.z = q.z();
  map2lidar_.transform.rotation.w = q.w();
}

std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> ICPNode::filter_points_in_map(std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::transformPointCloud(*cloud, *transformed_cloud, transformation_);

  for (int i = 0; i < transformed_cloud->points.size(); i++) {
    const auto& point_in_lidar = cloud->points[i];
    const auto& point_in_map = transformed_cloud->points[i];
    if (pow(point_in_lidar.x, 2) + pow(point_in_lidar.y, 2) + pow(point_in_lidar.z, 2) < pow(1, 2))
      continue;
    else if (point_in_map.x > 0.25 && point_in_map.x < 27.25 && point_in_map.y > 0.25 && point_in_map.y < 14.75 && point_in_map.z < 1.5) {
      filtered_cloud->points.push_back(cloud->points[i]);
    }
  }
  RCLCPP_INFO(this->get_logger(), "size of input point cloud : %ld", cloud->points.size());
  RCLCPP_INFO(this->get_logger(), "size of filtered point cloud : %ld", filtered_cloud->points.size());

  return filtered_cloud;
}

std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> ICPNode::filter_points_in_lidar(std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>), transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::transformPointCloud(*cloud, *transformed_cloud, transformation_);

  auto in_map = [](pcl::PointXYZI point){
    int flag = 1;
    if (point.z > 2.0) {
      flag = 0;
    }
    else if (point.x / 30 + point.y / 10 > 1.0) {
      flag = 0;
    }
    else if (point.x / 30 + point.y / 10 < -0.2) {
      flag = 0;
    }
    return flag;
  };

  for (int i = 0; i < cloud->points.size(); i++) {
    auto& point = cloud->points[i];
    if (pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2) < pow(1, 2))
      continue;

    else if (pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2) > pow(25, 2))
      continue;

    else if (in_map(point)) {
      filtered_cloud->points.push_back(point);
    }
  }

  RCLCPP_INFO(this->get_logger(), "size of input point cloud : %ld", cloud->points.size());
  RCLCPP_INFO(this->get_logger(), "size of filtered point cloud : %ld", filtered_cloud->points.size());

  return filtered_cloud;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ICPNode>());
  rclcpp::shutdown();
  return 0;
}