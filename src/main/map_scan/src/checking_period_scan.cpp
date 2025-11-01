#include <chrono>
#include <yaml-cpp/yaml.h>
#include <filesystem>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>

#include "radar_msgs/msg/game_status.hpp"


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

        game_type_ = 0;
        game_process_ = 0;
        stage_remain_time_ = 0;

        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic_name, 10,
            std::bind(&PointCloudMerger::pointCloudCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "PointCloudMerger node started.");
        game_status_sub_ = this->create_subscription<radar_msgs::msg::GameStatus>(
            "/game_status", 10,
            std::bind(&PointCloudMerger::gameStatusCallback, this, std::placeholders::_1));
    }

private:
    void gameStatusCallback(const radar_msgs::msg::GameStatus::SharedPtr msg)
    {
        game_status_ref_ = *msg;
        game_type_ = game_status_ref_.game_type;        // 1Byte bit[0:3]
        game_process_ = game_status_ref_.game_progress; // 1Byte bit[4:7]
        stage_remain_time_ = game_status_ref_.stage_remain_time;
    }

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (game_type_ != 1 || game_process_ != 1) // RMUC & 3分钟准备剩余9s
            return;
        if (stage_remain_time_ >= 10) // 剩余时间小于9秒
            return;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud);

        RCLCPP_INFO(this->get_logger(), "Received point cloud with %ld points.", cloud->size());
        *merged_cloud_ += *cloud; // 合并点云
        count_++;

        if (count_ == 75)
        {
            savePointCloud();
            merged_cloud_->clear(); // 清空已合并的点云
            count_ = 0;
        }
    }

    void savePointCloud()
    {
        std::string path = pcd_target_path;
        std::cout << path << std::endl;
        std::string filename = "before_process.pcd";
        // 检查路径是否存在 static_map.pcd
        if (!std::filesystem::exists(path))
        {
            // 创建目录（包括必要的父目录）
            if (std::filesystem::create_directories(path))
            {
                std::cout << "目录创建成功: " << path << std::endl;
            }
            else
            {
                std::cerr << "目录创建失败: " << path << std::endl;
            }
        }
        else
        {
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

    rclcpp::Subscription<radar_msgs::msg::GameStatus>::SharedPtr game_status_sub_;
    uint8_t game_type_;
    uint8_t game_process_;
    uint16_t stage_remain_time_;
    radar_msgs::msg::GameStatus game_status_ref_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<PointCloudMerger>());
    rclcpp::shutdown();
    return 0;
}
