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

// Eigen::Affine3f tum_to_transform_stamped(std::vector<double> TUM)
// {
//     // 位置 (x,y,z)
//     Eigen::Vector3f position(TUM[0], TUM[1], TUM[2]);

//     // 方向四元数 (w,x,y,z)
//     Eigen::Quaternionf orientation(TUM[3], TUM[4], TUM[5], TUM[6]);
    

//     Eigen::Affine3f transform = Eigen::Affine3f::Identity();
//     transform.translate(position);
//     transform.rotate(orientation);

//     return transform;
// }

Eigen::Affine3f tum_to_transform_stamped(std::vector<double> TUM)
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
    
    // auto transform = tum_to_transform_stamped(t_r);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::transformPointCloud(*cloud, *transformed_cloud, transform);
    
    for (auto& point : cloud->points)
    {
        if (point.z > -0.2) {
            transformed_cloud->points.push_back(point);
        } 
    }
    transformed_cloud->width = transformed_cloud->points.size();
    transformed_cloud->height = 1; // 无序点云可以将 height 设置为 1

    pcl::io::savePCDFile(output, *transformed_cloud);

    return 0;

}