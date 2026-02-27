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

int accumulate_frame = 2;   // use past two frames

void print_cloud(sensor_msgs::msg::PointCloud2 msg);


class Cluster : public rclcpp::Node
{
public:
    Cluster();
    ~Cluster(){}
    
private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    pcl::PointCloud<pcl::PointXYZ>::Ptr project(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_xyz);

    std::list<pcl::PointCloud<pcl::PointXYZ>::Ptr> points_list_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> accumulated_clouds_;
};

