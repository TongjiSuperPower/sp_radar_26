#ifndef VIDEO_POINTCLOUD_PROJECTION_HPP_
#define VIDEO_POINTCLOUD_PROJECTION_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/buffer.h> // <--- 确保包含这个头文件
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <cv_bridge/cv_bridge.h>

#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "camera_io/io/camera.hpp"
#include "camera_io/tools/exiter.hpp"
#include "camera_io/tools/logger.hpp"
#include "camera_io/tools/math_tools.hpp"


class PointCloudSubscriber : public rclcpp::Node
{
public:
    PointCloudSubscriber();

private:
    void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
    void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    void transform_point_cloud(
        const sensor_msgs::msg::PointCloud2::SharedPtr &msg,
        const geometry_msgs::msg::TransformStamped &transform,
        pcl::PointCloud<pcl::PointXYZI> &transformed_cloud);

    void pointclouds_to_image(const pcl::PointCloud<pcl::PointXYZI> &cloud, cv::Mat &img);

    cv::Mat camera_matrix_, distort_coeffs_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr img_subscription_;
    std::shared_ptr<io::Camera> camera_;
    cv::Mat img_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_subscription_;
    std::queue<std::vector<cv::Point2f>> point_queue_;
    std::queue<std::vector<double>> features_queue_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    double max_distance_ = 0;
};

#endif // POINT_CLOUD_SUBSCRIBER_HPP_
