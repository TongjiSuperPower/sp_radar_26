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
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/livox/filtered_lidar", 10, std::bind(&Cluster::callback, this, std::placeholders::_1));
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox/clustered_lidar", 10);
}


void Cluster::callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>), 
                                        cloud_across_frame(new pcl::PointCloud<pcl::PointXYZ>), 
                                        cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
    
    pcl::fromROSMsg(*msg, *cloud);
    RCLCPP_INFO(this->get_logger(), "Received a point cloud with %d points", cloud->size());
    points_list_.push_back(cloud);
    if (points_list_.size() > accumulate_frame)
        points_list_.pop_front();
    for (auto& points: points_list_) {
        *cloud_across_frame += *points;
    }
    
    cloud_projected = project(cloud_across_frame);
    if (cloud->empty()) {return;}
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_projected);
    auto time = std::chrono::system_clock::now();

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.25);
    ec.setMinClusterSize(5);
    ec.setMaxClusterSize(1000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_projected);
    std::vector<pcl::PointIndices> cluster_indices;
    ec.extract(cluster_indices);
    // std::cout<<(std::chrono::system_clock::now()-time).count()<<"ms"<<std::endl;
    
    pcl::PointCloud<pcl::PointXYZ> *out_cloud(new pcl::PointCloud<pcl::PointXYZ>); 
    for(auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for(auto pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            cloud_cluster->points.push_back(cloud_across_frame->points[*pit]);
        }        
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        pcl::PointXYZ move_point;
        for(auto point:cloud_cluster->points)
        {
            move_point.x += point.x;
            move_point.y += point.y;
            move_point.z += point.z;
        }
        move_point.x /= cloud_cluster->points.size();
        move_point.y /= cloud_cluster->points.size();
        move_point.z /= cloud_cluster->points.size();
        if (move_point.x < 30 && abs(move_point.y) < 20) {
            out_cloud->points.push_back(move_point);      
        }  
    }
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*out_cloud, output);
    output.header.frame_id = "livox_frame";
    output.header.stamp = msg->header.stamp;
    pub_->publish(output);
    // print_cloud(output);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    RCLCPP_INFO(this->get_logger(), "Cluster callback time: %f", std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count()/1000.0);
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
