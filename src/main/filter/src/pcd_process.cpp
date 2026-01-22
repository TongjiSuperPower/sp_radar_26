#include <rclcpp/rclcpp.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <vector>

class PointCloudInterpolation : public rclcpp::Node
{
public:
    PointCloudInterpolation()
        : Node("pointcloud_interpolation")
    {
        RCLCPP_INFO(this->get_logger(), "PointCloudInterpolation node started.");
        processPointCloud();
    }

private:
    void processPointCloud()
    {
        // 原始点云和插值后的点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr dense_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr voxelized_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        // 读取PCD文件
        std::string input_filename = "./src/main/map_scan/pcd/before_process.pcd";
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_filename, *input_cloud) == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load PCD file: %s", input_filename.c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Loaded point cloud with %ld points.", input_cloud->size());


        for (auto& point: input_cloud->points) {
            // 过滤掉z值小于0的点
            if (fabs(point.z) > 50 || fabs(point.x) > 60 || fabs(point.y) > 60){
                point.x = point.y = point.z = 0;
            }
        }

        // // 插值处理
        // interpolatePointCloud(input_cloud, dense_cloud);

        // // 保存插值后的点云
        // std::string output_filename = "./src/main/map_scan/pcd/process_1.pcd";
        // if (pcl::io::savePCDFileBinary(output_filename, *dense_cloud) == 0)
        // {
        //     RCLCPP_INFO(this->get_logger(), "Saved dense point cloud to %s with %ld points.", output_filename.c_str(), dense_cloud->size());
        // }
        // else
        // {
        //     RCLCPP_ERROR(this->get_logger(), "Failed to save point cloud to %s", output_filename.c_str());
        // }
        // *input_cloud = *dense_cloud; // 更新输入点云为插值后的点云

        // 对插值后的点云进行体素化处理
        voxelizePointCloud(input_cloud, voxelized_cloud);

        // 保存体素化后的点云
        std::string voxelized_filename = "./src/main/map_scan/pcd/process_2.pcd";
        if (pcl::io::savePCDFileBinary(voxelized_filename, *voxelized_cloud) == 0)
        {
            RCLCPP_INFO(this->get_logger(), "Saved voxelized point cloud to %s with %ld points.", voxelized_filename.c_str(), voxelized_cloud->size());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to save voxelized point cloud to %s", voxelized_filename.c_str());
        }
        // rclcpp::shutdown();
    }
    void voxelizePointCloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &dense_cloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr &voxelized_cloud)
    {
        const int divide = 10; // 体素化的分辨率
        for (int i = 0; i < divide; i++) {
            auto points = dense_cloud->points[i * dense_cloud->points.size() / divide];
        }
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
        voxel_grid.setInputCloud(dense_cloud);

        // 设置体素大小，这个参数控制体素化的精度
        float leaf_size = 0.1f; // 设定体素大小为 10cm
        voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);

        // 执行体素化
        voxel_grid.filter(*voxelized_cloud);
        RCLCPP_INFO(this->get_logger(), "Voxelization complete. Dense points: %ld, Voxelized points: %ld",
                    dense_cloud->size(), voxelized_cloud->size());
    }
    void interpolatePointCloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr &dense_cloud)
    {
        // KD树搜索近邻点
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(input_cloud);

        // 插值的邻域大小
        int K = 20; // 每个点寻找20个最近邻点

        // 遍历输入点云的每个点
        for (size_t i = 0; i < input_cloud->points.size(); ++i)
        {
            const pcl::PointXYZ &current_point = input_cloud->points[i];
            std::vector<int> point_indices(K);     // 最近K个点的索引
            std::vector<float> point_distances(K); // 距离值

            // 查找当前点最近的K个点
            if (kdtree.nearestKSearch(current_point, K, point_indices, point_distances) > 0)
            {
                // 将当前点添加到结果点云
                dense_cloud->points.push_back(current_point);

                // 对邻域点进行插值
                for (int j = 1; j < K; ++j)
                {
                    const pcl::PointXYZ &neighbor_point = input_cloud->points[point_indices[j]];
                    double k = abs((neighbor_point.z - current_point.z) / std::sqrt(std::pow(neighbor_point.x - current_point.x, 2) + std::pow(neighbor_point.y - current_point.y, 2)));
                    if (k > 0.3 && k < 6)
                        continue;
                    pcl::PointXYZ interpolated_point;
                    interpolated_point.x = (current_point.x + neighbor_point.x) / 2.0;
                    interpolated_point.y = (current_point.y + neighbor_point.y) / 2.0;
                    interpolated_point.z = (current_point.z + neighbor_point.z) / 2.0;
                    dense_cloud->points.push_back(interpolated_point);
                    interpolated_point.x = (current_point.x / 4.0 + 3 * neighbor_point.x / 4.0);
                    interpolated_point.y = (current_point.y / 4.0 + 3 * neighbor_point.y / 4.0);
                    interpolated_point.z = (current_point.z / 4.0 + 3 * neighbor_point.z / 4.0);
                    dense_cloud->points.push_back(interpolated_point);
                    interpolated_point.x = (3 * current_point.x / 4.0 + neighbor_point.x / 4.0);
                    interpolated_point.y = (3 * current_point.y / 4.0 + neighbor_point.y / 4.0);
                    interpolated_point.z = (3 * current_point.z / 4.0 + neighbor_point.z / 4.0);
                    dense_cloud->points.push_back(interpolated_point);
                }
            }
        }

        // 设置点云的宽度和高度
        dense_cloud->width = dense_cloud->points.size();
        dense_cloud->height = 1;
        dense_cloud->is_dense = true;

        RCLCPP_INFO(this->get_logger(), "Interpolation complete. Original points: %ld, Dense points: %ld",
                    input_cloud->size(), dense_cloud->size());
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::make_shared<PointCloudInterpolation>();
    
    // rclcpp::shutdown();
    return 0;
}
