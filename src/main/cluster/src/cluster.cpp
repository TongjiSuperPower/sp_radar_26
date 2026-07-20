#include "cluster.h"
#include <pcl/PCLPointCloud2.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <rclcpp/duration.hpp>
#include <pcl/segmentation/extract_clusters.h>

Cluster::Cluster(): Node("cluster_node")
{
    RCLCPP_INFO(this->get_logger(), "cluster_node start");

    sub_ = this->create_subscription<radar_msgs::msg::CarsAndDrones>(
        "/livox/filtered_lidar", 10,
        std::bind(&Cluster::callback, this, std::placeholders::_1));
    pub_ = this->create_publisher<radar_msgs::msg::CarsAndDrones>("/livox/clustered_lidar", 10);
}


void Cluster::callback(const radar_msgs::msg::CarsAndDrones::SharedPtr msg)
{
    auto result = std::make_unique<radar_msgs::msg::CarsAndDrones>();

    cluster_cloud(msg->cars_cloud, points_list_, result->cars_cloud);
    cluster_cloud(msg->drones_cloud, points_list_drone_, result->drones_cloud);

    pub_->publish(std::move(result));
}


void Cluster::cluster_cloud(const sensor_msgs::msg::PointCloud2 &in_cloud,
                            std::list<pcl::PointCloud<pcl::PointXYZ>::Ptr> &points_list,
                            sensor_msgs::msg::PointCloud2 &out_cloud)
{
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>),
                                        cloud_across_frame(new pcl::PointCloud<pcl::PointXYZ>),
                                        cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(in_cloud, *cloud);
    if (cloud->empty()) { return; }

    points_list.push_back(cloud);
    if (points_list.size() > accumulate_frame)
        points_list.pop_front();
    for (auto& points: points_list) {
        *cloud_across_frame += *points;
    }

    cloud_projected = project(cloud_across_frame);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_projected);

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.25);
    ec.setMinClusterSize(5);
    ec.setMaxClusterSize(1000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_projected);
    std::vector<pcl::PointIndices> cluster_indices;
    ec.extract(cluster_indices);

    pcl::PointCloud<pcl::PointXYZ>::Ptr out_pcl(new pcl::PointCloud<pcl::PointXYZ>);

    for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (auto pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            cloud_cluster->points.push_back(cloud_across_frame->points[*pit]);
        }
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        pcl::PointXYZ move_point;
        for (auto point: cloud_cluster->points)
        {
            move_point.x += point.x;
            move_point.y += point.y;
            move_point.z += point.z;
        }
        move_point.x /= cloud_cluster->points.size();
        move_point.y /= cloud_cluster->points.size();
        move_point.z /= cloud_cluster->points.size();
        if (move_point.x < 30 && abs(move_point.y) < 20) {
            out_pcl->points.push_back(move_point);
        }
    }

    pcl::toROSMsg(*out_pcl, out_cloud);
    out_cloud.header = in_cloud.header;

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    RCLCPP_INFO(this->get_logger(), "Clustered %zu points in %f ms",
                out_pcl->points.size(),
                std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() / 1000.0);
}


pcl::PointCloud<pcl::PointXYZ>::Ptr Cluster::project(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_xyz) {
    
    // 创建输出点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
    
    // 设置点云属性
    cloud_projected->width = cloud_xyz->width;
    cloud_projected->height = cloud_xyz->height;
    cloud_projected->is_dense = cloud_xyz->is_dense;
    cloud_projected->points.resize(cloud_xyz->size());
    
    // 转换每个点
    for (size_t i = 0; i < cloud_xyz->size(); ++i) {
        cloud_projected->points[i].x = cloud_xyz->points[i].x;
        cloud_projected->points[i].y = cloud_xyz->points[i].y;
        cloud_projected->points[i].z = 0.0;
    }
    
    return cloud_projected;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Cluster>());
    rclcpp::shutdown();
    return 0;
}
