#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>
#include <iostream>
#include <cmath>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h> // <--- 确保包含这个头文件
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <Eigen/Geometry>
#include <pcl/visualization/pcl_visualizer.h>
#include <yaml-cpp/yaml.h>

// #include <geometry_msgs/msg/transform_stamped.hpp>

class PCDPointCloudFilterNode : public rclcpp::Node
{
public:
    PCDPointCloudFilterNode()
        : Node("pcd_pointcloud_filter")
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        // 加载PCD文件
        auto config = YAML::LoadFile("./src/main/map_scan/config/filter.yaml");
        use_static_scan_ = config["use_static_scan"].as<bool>();
        std::string map_pcd_file;
        if (use_static_scan_)
            map_pcd_file = config["scan_pcd_path"].as<std::string>();
        else
            map_pcd_file = config["map_pcd_path"].as<std::string>();
        pcl::io::loadPCDFile(map_pcd_file, *MAP_pcd_cloud_);
        transformAndVoxelPCD();
        // 体素化
        RCLCPP_INFO(this->get_logger(), "Loaded and voxelized PCD file.");
        // 订阅livox lidar点云话题
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "livox/lidar", 10,
            std::bind(&PCDPointCloudFilterNode::pointCloudCallback, this, std::placeholders::_1));

        // 发布过滤后的点云
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox/filtered_lidar", 10);

    }

private:
    geometry_msgs::msg::TransformStamped inverse_transform(
        const geometry_msgs::msg::TransformStamped &transform)
    {
        // 创建逆变换对象
        geometry_msgs::msg::TransformStamped inverse;

        // 交换源坐标系和目标坐标系
        inverse.header.frame_id = transform.child_frame_id;
        inverse.child_frame_id = transform.header.frame_id;
        inverse.header.stamp = transform.header.stamp;

        // 将原始变换转换为 tf2::Transform
        tf2::Transform tf_transform;
        tf2::fromMsg(transform.transform, tf_transform);

        // 计算逆变换
        tf2::Transform tf_inverse = tf_transform.inverse();

        // 将逆变换转换回 geometry_msgs
        inverse.transform = tf2::toMsg(tf_inverse);

        return inverse;
    }

    void transformAndVoxelPCD()
    {
        if (use_static_scan_)
        {
            sensor_msgs::msg::PointCloud2::SharedPtr map_msg(new sensor_msgs::msg::PointCloud2());
            pcl::toROSMsg(*MAP_pcd_cloud_, *map_msg);
            tf2::doTransform(*map_msg, *map_msg, transform_L2M_);
            pcl::fromROSMsg(*map_msg, *MAP_pcd_cloud_);
        }
        // 创建k-d树
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(MAP_pcd_cloud_);
        voxel_filter.setLeafSize(0.1f, 0.1f, 0.1f); // 设置体素大小
        voxel_filter.filter(*MAP_pcd_cloud_voxel_);
        kdtree_.setInputCloud(MAP_pcd_cloud_voxel_);
    }

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // 将接收到的PointCloud2转换为PCL格式
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
        sensor_msgs::msg::PointCloud2::SharedPtr output_msg(new sensor_msgs::msg::PointCloud2());
        auto now = get_clock()->now();

        try
        {   // TODO 检查重定位收敛时间
            transform_L2M_ = tf_buffer_->lookupTransform(
                "map",
                "lidar_frame",
                rclcpp::Time(0),
                rclcpp::Duration(1, 0));
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
        }
        transform_M2L_ = inverse_transform(transform_L2M_);
        tf2::doTransform(*msg, *msg, transform_L2M_);
        pcl::fromROSMsg(*msg, *cloud_in);
        removeOverlap(cloud_in, cloud_filtered);
        pcl::toROSMsg(*cloud_filtered, *output_msg);
        tf2::doTransform(*output_msg, *output_msg, transform_M2L_);

        output_msg->header.frame_id = "livox_frame";
        output_msg->header.stamp.sec = now.seconds();
        output_msg->header.stamp.nanosec = static_cast<uint32_t>(now.nanoseconds() % 1000000000);

        publisher_->publish(*output_msg);
    }

    // 过滤场地背景点云 cloud_in:场地坐标系下的最新一帧点云
    void removeOverlap(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered)
    {
        std::vector<int> point_idx_search;
        std::vector<float> point_dist_squared;
        float threshold_distance = 0.18f;
        int filtered_count = 0;

        auto is_point_outside_field = [] (const pcl::PointXYZ& point)  {
            return (point.x < 0.5 || point.x > 27.5 || 
                    point.y < 0.25 || point.y > 14.75 || 
                    point.z > 1.5 || point.z < 0);
        };
        auto is_point_in_base = [] (const pcl::PointXYZ& point)  {
            return (point.x > 1.6 && point.x < 3.2 && point.y > 6.5 && point.y < 8.5) ||
                    (point.x > 24.8 && point.x < 26.4 && point.y > 6.5 && point.y < 8.5);
        };
        auto is_point_in_outpost = [] (const pcl::PointXYZ& point)  {
            return (point.x > 10.5 && point.x < 11.3 && point.y > 3.3 && point.y < 3.9) ||
                    (point.x > 16.7 && point.x < 17.5 && point.y > 11.1 && point.y < 11.7);
        };
        // 由于场地模型内飞镖闸门是打开的，所以需要手动删除
        auto is_point_in_dart_door = [] (const pcl::PointXYZ& point)  { 
            return (point.x > 25.0 && point.x < 27.0 && point.y > 2.9 && point.y < 5.5) ||
                    (point.x < 3.0 && point.x > 1.0 && point.y < 12.1 && point.y > 9.5);
        };
        auto is_point_on_helipad = [] (const pcl::PointXYZ& point)  { 
            return (point.x > 25.0 && point.y <3.5) || (point.x < 3.0 && point.y > 11.5);
        };
        auto is_point_in_exchange_station = [] (const pcl::PointXYZ& point)  { 
            return (point.x < 1.5 && point.y < 1.8) || (point.x > 26.5 && point.y > 13.2);
        };


        for (const auto &point_in : cloud_in->points)
        {
            // 过滤场地外的点
            if (is_point_outside_field(point_in) || is_point_in_base(point_in) ||
                is_point_in_outpost(point_in) || is_point_in_dart_door(point_in) || 
                is_point_on_helipad(point_in) || is_point_in_exchange_station(point_in)) {
                filtered_count++;
                continue;
            }
            
            // 过滤map_scan得到的背景点云
            kdtree_.nearestKSearch(point_in, 1, point_idx_search, point_dist_squared);
            if (point_dist_squared[0] < threshold_distance * threshold_distance)
            {
                filtered_count++;
                continue;
            }
            cloud_filtered->points.push_back(point_in);
        }
        RCLCPP_INFO(this->get_logger(), "remove %d points", filtered_count);
        RCLCPP_INFO(this->get_logger(), "point cloud size: %ld", cloud_filtered->points.size());
        RCLCPP_INFO(this->get_logger(), "Overlap removal completed.");
    }


    geometry_msgs::msg::TransformStamped transform_L2M_;
    geometry_msgs::msg::TransformStamped transform_M2L_;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    pcl::PointCloud<pcl::PointXYZ>::Ptr MAP_pcd_cloud_{new pcl::PointCloud<pcl::PointXYZ>()};
    pcl::PointCloud<pcl::PointXYZ>::Ptr MAP_pcd_cloud_voxel_{new pcl::PointCloud<pcl::PointXYZ>()};
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // pcl::visualization::PCLVisualizer::Ptr visualizer;

    int use_static_scan_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PCDPointCloudFilterNode>());
    rclcpp::shutdown();
    return 0;
}
