#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <yaml-cpp/yaml.h>
#include <filesystem>

class PointCloudMerger : public rclcpp::Node
{
public:
    PointCloudMerger()
        : Node("pointcloud_merger"), count_(0)
    {
        std::string config_file;
        this->declare_parameter<std::string>("config_file", "");
        this->get_parameter("config_file", config_file);
        // Load configuration from YAML file
        const auto config = YAML::LoadFile(config_file);
        pcd_target_path = config["pcd_target_path"].as<std::string>();
        topic_name = config["topic_name"].as<std::string>();
        scann_count = config["scann_count"].as<int>();

        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic_name, 10,
            std::bind(&PointCloudMerger::pointCloudCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "PointCloudMerger node started.");
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud);

        RCLCPP_INFO(this->get_logger(), "Received point cloud with %ld points.", cloud->size());
        *merged_cloud_ += *cloud; // 合并点云
        count_++;

        if (count_ == 50)
        {
            savePointCloud();
            merged_cloud_->clear(); // 清空已合并的点云
            count_ = 0;
        }
    }

    void savePointCloud()
    {
        std::string path = pcd_target_path, filename = "before_process.pcd";
        std::cout << path << std::endl;
        // 检查路径是否存在 static_map.pcd
        if (!std::filesystem::exists(path)) {
            // 创建目录（包括必要的父目录）
            if (std::filesystem::create_directories(path)) {
                std::cout << "目录创建成功: " << path << std::endl;
            } else {
                std::cerr << "目录创建失败: " << path << std::endl;
            }
        } else {
            std::cout << "目录已存在: " << path << std::endl;
        }
        if (pcl::io::savePCDFileBinary(path + filename, *merged_cloud_) == 0)
        {
            RCLCPP_INFO(this->get_logger(), "Saved merged point cloud to %s", (path + filename).c_str());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to save point cloud to %s", (path + filename).c_str());
        }
        
        rclcpp::shutdown();
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud_{new pcl::PointCloud<pcl::PointXYZ>()};
    int count_;
    std::string pcd_target_path;
    std::string topic_name;
    int scann_count;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudMerger>());
    rclcpp::shutdown();
    return 0;
}
