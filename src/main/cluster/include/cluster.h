#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/io/pcd_io.h"
#include <numeric>
#include <pcl/impl/point_types.hpp>
#include <rclcpp/logging.hpp>
#include <vector>
#include <chrono>
#include <pcl/kdtree/kdtree.h>
#include <list>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h> // <--- 确保包含这个头文件
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_ros/buffer.h>

int accumulate_frame = 2;   // use past two frames

void print_cloud(sensor_msgs::msg::PointCloud2 msg);


class Cluster : public rclcpp::Node
{
public:
    Cluster();
    ~Cluster(){}
    
private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void callbackdrone(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    pcl::PointCloud<pcl::PointXYZ>::Ptr project(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_xyz);

    std::list<pcl::PointCloud<pcl::PointXYZ>::Ptr> points_list_;
    std::list<pcl::PointCloud<pcl::PointXYZ>::Ptr> points_list_drone_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> accumulated_clouds_;

    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_drone_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_drone_;
    geometry_msgs::msg::TransformStamped transform_M2L, transform_L2M_;
    

};

