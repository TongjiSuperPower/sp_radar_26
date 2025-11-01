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
#include <visualization_msgs/msg/marker.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include "iterable_queue.hpp"

int accumulate_frame = 2;   // use past two frames

void print_cloud(sensor_msgs::msg::PointCloud2 msg);


class Cluster : public rclcpp::Node
{
public:
    Cluster(const rclcpp::NodeOptions& node_options);
    ~Cluster(){}
    
private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    IterableQueue<pcl::PointCloud<pcl::PointXYZ>::Ptr> queue_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> accumulated_clouds_;
};

