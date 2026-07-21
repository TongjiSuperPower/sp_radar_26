#include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/point_cloud2.hpp>
#include <radar_msgs/msg/cars_and_drones.hpp>
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
        auto config = YAML::LoadFile("./src/main/filter/config/filter.yaml");
        use_static_scan_ = config["use_static_scan"].as<bool>();
        use_outside_filter_ = config["use_outside_filter"].as<bool>();
        std::string map_pcd_file;
        
        enemy_color = config["enemy_color"].as<std::string>();

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
            std::bind(&PCDPointCloudFilterNode::pointCloudCallback, this, std::placeholders::_1)); //get the message into callback

        // 发布过滤后的点云
        publisher_ = create_publisher<radar_msgs::msg::CarsAndDrones>("/livox/filtered_lidar", 10);

    }

private:
    geometry_msgs::msg::TransformStamped inverse_transform(    // this is for later where we need to do an inverse transformation
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
            sensor_msgs::msg::PointCloud2::SharedPtr map_msg(new sensor_msgs::msg::PointCloud2()); //single use
            pcl::toROSMsg(*MAP_pcd_cloud_, *map_msg); // to ros msg
            tf2::doTransform(*map_msg, *map_msg, transform_L2M_); //lidar to map frame 1 sec later
            pcl::fromROSMsg(*map_msg, *MAP_pcd_cloud_); //back to pcl format
        }
        // 创建k-d树
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter; 
        voxel_filter.setInputCloud(MAP_pcd_cloud_);
        voxel_filter.setLeafSize(0.1f, 0.1f, 0.1f); // 设置体素大小 //the filter // lots of points -> less but big points
        voxel_filter.filter(*MAP_pcd_cloud_voxel_);
        kdtree_.setInputCloud(MAP_pcd_cloud_voxel_);
    }

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // 将接收到的PointCloud2转换为PCL格式
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_car_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_drone_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        sensor_msgs::msg::PointCloud2::SharedPtr car_msg(new sensor_msgs::msg::PointCloud2());
        sensor_msgs::msg::PointCloud2::SharedPtr drone_msg(new sensor_msgs::msg::PointCloud2());

        auto now = get_clock()->now();

        if (!use_static_scan_){
            try
            {   // TODO 检查重定位收敛时间
                transform_L2M_ = tf_buffer_->lookupTransform(  //for transforming lidar to map frame
                    "map",
                    "lidar_frame",
                    rclcpp::Time(0),
                    rclcpp::Duration(1, 0));
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
            } //the points wouldnbt be correct if we use the map transofrm?
            transform_M2L_ = inverse_transform(transform_L2M_); 
            tf2::doTransform(*msg, *msg, transform_L2M_); // first we get the msg into map frame
        }

        pcl::fromROSMsg(*msg, *cloud_in); // then to point cloud format
        removeOverlap(cloud_in, filtered_car_cloud, filtered_drone_cloud); // then remove the overlap points from cloud_in and add the rest to filtered_car_cloud
        pcl::toROSMsg(*filtered_car_cloud, *car_msg);// to ros
        pcl::toROSMsg(*filtered_drone_cloud, *drone_msg);

        if (!use_static_scan_){
            tf2::doTransform(*car_msg, *car_msg, transform_M2L_);// again to livox
        }
        if (!use_static_scan_){
            tf2::doTransform(*drone_msg, *drone_msg, transform_M2L_);
        }

        car_msg->header.frame_id = "livox_frame";
        car_msg->header.stamp.sec = msg->header.stamp.sec;
        car_msg->header.stamp.nanosec = msg->header.stamp.nanosec;

        drone_msg->header.frame_id = "livox_frame";
        drone_msg->header.stamp.sec = msg->header.stamp.sec;
        drone_msg->header.stamp.nanosec = msg->header.stamp.nanosec;
        
        auto cars_and_drones_msg = std::make_unique<radar_msgs::msg::CarsAndDrones>();
        cars_and_drones_msg->cars_cloud = std::move(*car_msg);
        cars_and_drones_msg->drones_cloud = std::move(*drone_msg);
        publisher_->publish(std::move(cars_and_drones_msg));
    }

    // 过滤场地背景点云 cloud_in:场地坐标系下的最新一帧点云
    void removeOverlap(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered_car_cloud,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered_drone_cloud)
    {
        std::vector<int> point_idx_search; 
        std::vector<float> point_dist_squared; //the one we use
        float threshold_distance = 0.15f; //f for float literal
        int filtered_count = 0;

        auto is_point_outside_field = [] (const pcl::PointXYZ& point)  {
            return (point.x < 0.5 || point.x > 27.5 || 
                    point.y < 0.25 || point.y > 14.75 || 
                    point.z < 0);
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
            return (point.x > 25.0 && point.x < 27.5 && point.y > 2.9 && point.y < 5.5) ||
                    (point.x < 3.0 && point.x > 0.5 && point.y < 12.1 && point.y > 9.5);
        };
        auto is_point_on_helipad = [] (const pcl::PointXYZ& point)  { 
            return (point.x > 25.0 && point.y <3.5) || (point.x < 3.0 && point.y > 11.5);
        };
        auto is_point_in_exchange_station = [] (const pcl::PointXYZ& point)  { 
            return (point.x < 1.5 && point.y < 1.8) || (point.x > 26.5 && point.y > 13.2);
        };
        auto is_point_in_drone_red = [] (const pcl::PointXYZ& point)  {
            return (point.x < 16.4 && point.x > 0.5 && point.y > 8.85 && point.y < 13.5 && point.z > 1.4 && point.z < 3 );
        };

        auto is_point_in_drone_blue = [] (const pcl::PointXYZ& point)  {
            return (point.x < 27.5 && point.x > 28 - 16.4 && point.y > 1.5 && point.y < 15 - 8.85 && point.z > 1.4 && point.z < 3 );
        };

        auto is_point_up = [] (const pcl::PointXYZ& point)  {
            return (point.z > 3);
        };

        auto is_point_in_tech_core = [] (const pcl::PointXYZ& point) {
            return (point.y - point.x < -5.73 && point.y - point.x > -7.27 && 
                    point.y + point.x < 22.96 && point.y + point.x > 20.04);
        };

        for (const auto &point_in : cloud_in->points)
        {
            if (!use_static_scan_ && use_outside_filter_) {
                // 过滤场地外的点
                if (is_point_outside_field(point_in) || is_point_in_base(point_in) ||
                    is_point_in_outpost(point_in) || is_point_in_dart_door(point_in) || 
                    is_point_on_helipad(point_in) || is_point_in_exchange_station(point_in) || 
                    is_point_in_tech_core(point_in) || is_point_up(point_in)) {
                    filtered_count++;
                    continue;
                }
            }

            // 分类无人机点云
            if (is_point_in_drone_red(point_in) || is_point_in_drone_blue(point_in)) {
                filtered_drone_cloud->points.push_back(point_in);
                continue;
            }

            // 过滤map_scan得到的背景点云
            kdtree_.nearestKSearch(point_in, 1, point_idx_search, point_dist_squared); //if forget how to work go to pcl Ktree explains it
            if (point_dist_squared[0] < threshold_distance * threshold_distance)
            {
                filtered_count++;
                continue;
            }

            filtered_car_cloud->points.push_back(point_in);
        }
        RCLCPP_INFO(this->get_logger(), "remove %d points", filtered_count);
        RCLCPP_INFO(this->get_logger(), "point cloud size: %ld", filtered_car_cloud->points.size());
        RCLCPP_INFO(this->get_logger(), "Overlap removal completed.");
    }


    geometry_msgs::msg::TransformStamped transform_L2M_;
    geometry_msgs::msg::TransformStamped transform_M2L_;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    pcl::PointCloud<pcl::PointXYZ>::Ptr MAP_pcd_cloud_{new pcl::PointCloud<pcl::PointXYZ>()};
    pcl::PointCloud<pcl::PointXYZ>::Ptr MAP_pcd_cloud_voxel_{new pcl::PointCloud<pcl::PointXYZ>()};
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<radar_msgs::msg::CarsAndDrones>::SharedPtr publisher_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // pcl::visualization::PCLVisualizer::Ptr visualizer;

    int use_static_scan_;
    int use_outside_filter_;
    std::string enemy_color;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PCDPointCloudFilterNode>());
    rclcpp::shutdown();
    return 0;
}
