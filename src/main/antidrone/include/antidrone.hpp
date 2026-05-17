#ifndef ANTIDRONE_HPP
#define ANTIDRONE_HPP

#include <iostream>
#include <chrono>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Transform.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include "../tools/solver.hpp"
#include "../io/gimbal/gimbal.hpp"

class Antidrone : public rclcpp::Node

{
public:
    Antidrone();

private:
    void Callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void transform_point_cloud(
        const sensor_msgs::msg::PointCloud2::SharedPtr &msg,
        const geometry_msgs::msg::TransformStamped &transform,
        pcl::PointCloud<pcl::PointXYZ> &transformed_cloud);
    geometry_msgs::msg::TransformStamped tum_to_transform_stamped(std::vector<double> TUM);
    geometry_msgs::msg::TransformStamped inverse_transform(const geometry_msgs::msg::TransformStamped &transform);
    io::Gimbal gimbal_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    geometry_msgs::msg::TransformStamped transform_C2L_;
    geometry_msgs::msg::TransformStamped transform_L2C_;
    geometry_msgs::msg::TransformStamped transform_G2W_;
    geometry_msgs::msg::TransformStamped transform_W2G_;
    geometry_msgs::msg::TransformStamped transform_L2W_;
    cv::Mat T_lidar2world_;
    cv::Mat T_camera2gimbal_;
    cv::Mat T_gimbal2world_;
    cv::Mat camera_matrix_;
    std::vector<double> dist_coeffs_;
    double real_spacing_;
};

#endif // ANTIDRONE_HPP