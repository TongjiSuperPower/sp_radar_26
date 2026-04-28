#ifndef ANTIDRONE_HPP
#define ANTIDRONE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <yaml-cpp/yaml.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>
#include <cmath>

class Antidrone : public rclcpp::Node
{
public:
    Antidrone();
    ~Antidrone(){}
private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

    void Callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    geometry_msgs::msg::TransformStamped transform_L2G_, transform_G2L_;
    geometry_msgs::msg::TransformStamped tum_to_transform_stamped(std::vector<double> TUM);
    geometry_msgs::msg::TransformStamped inverse_transform(
        const geometry_msgs::msg::TransformStamped& transform);

    void transform_point_cloud(
        const sensor_msgs::msg::PointCloud2::SharedPtr &msg,
        const geometry_msgs::msg::TransformStamped &transform,
        pcl::PointCloud<pcl::PointXYZ> &transformed_cloud);
};

#endif
