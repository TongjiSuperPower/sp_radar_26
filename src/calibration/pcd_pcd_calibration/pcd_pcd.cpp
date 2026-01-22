#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <memory>
#include <string>

class PCLPublisher : public rclcpp::Node
{
public:
    PCLPublisher()
    : Node("pcl_publisher")
    {
        // 声明参数
        this->declare_parameter("cloud_topic", "/livox/lidar");
        this->declare_parameter("frame_id", "map");
        this->declare_parameter("publish_rate", 0.1);
        
        // 获取参数
        std::string cloud_topic = this->get_parameter("cloud_topic").as_string();
        frame_id_ = this->get_parameter("frame_id").as_string();
        double publish_rate = this->get_parameter("publish_rate").as_double();
        
        // 创建发布者
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic, 10);
        
        // 创建定时器
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate)),
            std::bind(&PCLPublisher::timer_callback, this));
        
        // 创建示例点云
        create_sample_cloud();
        
        RCLCPP_INFO(this->get_logger(), "PCL Publisher node has been initialized");
    }

private:
    void timer_callback()
    {
        // 转换PCL点云为ROS2消息
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud_, output);
        output.header.stamp = this->now();
        output.header.frame_id = frame_id_;
        
        // 发布点云
        publisher_->publish(output);
        RCLCPP_DEBUG(this->get_logger(), "Published point cloud with %ld points", cloud_->size());
    }
    
    void create_sample_cloud()
    {
        // 创建一个示例点云 (在实际应用中，你可以从这里加载你的点云文件)
        auto cloud_no_intensity = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        pcl::io::loadPCDFile("./src/main/relocalization/pcd/RMUC_25_voxel.pcd", *cloud_no_intensity);
        for (auto point : *cloud_no_intensity) {
            pcl::PointXYZI p;
            p.x = point.x;
            p.y = point.y;
            p.z = point.z;
            p.intensity = 0.1f;
            cloud_->push_back(p);
        }

        RCLCPP_INFO(this->get_logger(), "Created sample cloud with %ld points", cloud_->size());
    }
    
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_;
    std::string frame_id_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PCLPublisher>());
    rclcpp::shutdown();
    return 0;
}