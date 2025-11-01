#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

#include <exception>

#include <rclcpp/rclcpp.hpp>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("./src/calibration/coordinate_transform/pcd/voxel.pcd", *cloud);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr scaled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    scaled_cloud->points.reserve(cloud->points.size());
    for (const auto& pt : cloud->points) {
        pcl::PointXYZ scaled_pt;
        scaled_pt.x = pt.x / 1000.0f;
        scaled_pt.y = pt.y / 1000.0f;
        scaled_pt.z = pt.z / 1000.0f;
        scaled_cloud->points.push_back(scaled_pt);
    }
    scaled_cloud->width = scaled_cloud->points.size();
    scaled_cloud->height = 1;
    scaled_cloud->is_dense = cloud->is_dense;

    pcl::io::savePCDFile("./src/calibration/coordinate_transform/pcd/voxel_scaled.pcd", *scaled_cloud);
    return 0;

}