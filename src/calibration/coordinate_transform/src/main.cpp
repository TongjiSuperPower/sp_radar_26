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

#include <iostream>

Eigen::Affine3f tvec_and_rvec_to_tranform_stamped(std::vector<double> TUM)
{
    // 位置 (x,y,z)
    Eigen::Vector3f position(TUM[0], TUM[1], TUM[2]);

    // 欧拉角 (roll, pitch, yaw) - 假设是按ZYX顺序
    Eigen::Vector3f euler_angles(TUM[3], TUM[4], TUM[5]);
    
    // 将欧拉角转换为旋转矩阵
    Eigen::AngleAxisf rollAngle(euler_angles.x(), Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitchAngle(euler_angles.y(), Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yawAngle(euler_angles.z(), Eigen::Vector3f::UnitZ());
    
    // 组合旋转（注意乘法顺序取决于你想要的旋转顺序）
    Eigen::Quaternionf orientation =  rollAngle * pitchAngle * yawAngle;
    
    // 或者直接使用Matrix3f:
    // Eigen::Matrix3f rotation = (yawAngle * pitchAngle * rollAngle).matrix();

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translate(position);
    transform.rotate(orientation);

    std::cout << "position x: " << TUM[0] << std::endl;

    return transform;
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    const auto config = YAML::LoadFile("./src/calibration/coordinate_transform/config/config.yaml");
    auto t_r = config["tvec_and_rvec_YPR"].as<std::vector<double>>(); // TUM: from camera to lidar
    std::string input = config["input"].as<std::string>();
    std::string output = config["output"].as<std::string>();
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(input, *cloud);
    
    auto transform = tvec_and_rvec_to_tranform_stamped(t_r);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform);
    

    transformed_cloud->width = transformed_cloud->points.size();
    transformed_cloud->height = 1; // 无序点云可以将 height 设置为 1

    pcl::io::savePCDFile(output, *transformed_cloud);

    return 0;

}