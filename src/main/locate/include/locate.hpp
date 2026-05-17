#ifndef POINT_CLOUD_SUBSCRIBER_HPP_
#define POINT_CLOUD_SUBSCRIBER_HPP_

#include <exception>
#include <queue>
#include <deque> // 默认底层容器

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h> // <--- 确保包含这个头文件
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <radar_msgs/msg/map_robot_data.hpp>
#include <radar_msgs/msg/car_bbox.hpp>
#include <radar_msgs/msg/car.hpp>
#include <radar_msgs/msg/cars.hpp>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include "fmt/core.h"

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

#include "tools/max_size_queue.hpp"

// const int camera_width = 3072, camera_height = 2048;
const int record_point_cloud_frame = 3;

class PointcloudLocater : public rclcpp::Node
{
public:
    // 构造函数
    PointcloudLocater();

private:
    // 点云回调函数
    void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void bbox_callback(const radar_msgs::msg::CarBbox::SharedPtr msg);
    // 点云转换函数
    void transform_point_cloud(
        const sensor_msgs::msg::PointCloud2::SharedPtr &msg,
        const geometry_msgs::msg::TransformStamped &transform,
        pcl::PointCloud<pcl::PointXYZ> &transformed_cloud);

    std::vector<std::pair<pcl::PointXYZ, int>> pointclouds_to_image(const pcl::PointCloud<pcl::PointXYZ> &cloud, cv::Mat &img);
    void locate(std::vector<std::pair<pcl::PointXYZ, int>> points);

    geometry_msgs::msg::TransformStamped tum_to_transform_stamped(std::vector<double> TUM);
    geometry_msgs::msg::TransformStamped inverse_transform(
        const geometry_msgs::msg::TransformStamped& transform);

    cv::Mat pointcloud_img_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Subscription<radar_msgs::msg::CarBbox>::SharedPtr bbox_subscription_;
    rclcpp::Publisher<radar_msgs::msg::Cars>::SharedPtr car_publisher_;
    cv::Mat camera_matrix_, distort_coeffs_;
    std::queue<std::vector<double>> features_queue_;
    double max_distance_ = 0;
    radar_msgs::msg::CarBbox::SharedPtr bbox_msg_;
    std::vector<std::vector<cv::Point3f>> car_points_;
    double camera_time_;
    double lidar_time_;
    std::mutex mtx_;

    int debug_flag_;
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    geometry_msgs::msg::TransformStamped transform_L2C_, transform_C2L_, transform_L2M_;

    tools::MaxSizeQueue<pcl::PointCloud<pcl::PointXYZ>> point_queue_;


};

#endif // POINT_CLOUD_SUBSCRIBER_HPP_
