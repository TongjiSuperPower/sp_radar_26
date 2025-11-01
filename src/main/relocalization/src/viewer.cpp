#include <chrono>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <open3d/Open3D.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Geometry>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <yaml-cpp/yaml.h>

class PointCloudVisualizer : public rclcpp::Node
{
public:
    PointCloudVisualizer()
        : Node("point_cloud_visualizer")
    {
        // 订阅坐标变换
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        // 订阅点云话题
        point_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/lidar", 10, std::bind(&PointCloudVisualizer::pointCloudCallback, this, std::placeholders::_1));
        // 读取pcd文件
        auto config = YAML::LoadFile("./src/main/relocalization/config/config.yaml");
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(config["pcd_target_path"].as<std::string>(), *cloud_target_) == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Couldn't read target PCD file.");
            return;
        }
        // 初始化 Open3D 可视化窗口
        viewer_ = std::make_shared<open3d::visualization::Visualizer>();
        viewer_->CreateVisualizerWindow("3D Viewer", 1024, 768);
        viewer_->GetRenderOption().point_size_ = 1.0;                   // 设置点云大小
        viewer_->GetRenderOption().background_color_ = {1.0, 1.0, 1.0}; // 设置背景色为白色

        setDefaultView();
    }

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // 尝试获取从map到livox_frame的变换
        geometry_msgs::msg::TransformStamped transform;
        try
        {
            transform = tf_buffer_->lookupTransform("map", "lidar_frame", tf2::TimePointZero);
            // 打印变换矩阵
            // RCLCPP_INFO(this->get_logger(), "Transform from map to livox_frame: ");
            // RCLCPP_INFO(this->get_logger(), "Translation: x=%f, y=%f, z=%f", transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z);
            // RCLCPP_INFO(this->get_logger(), "Rotation: x=%f, y=%f, z=%f, w=%f", transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "%s", ex.what());
            return;
        }
        // 将ROS的PointCloud2消息转换为PCL的点云格式
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        // cloud为livox_frame系下的点云，需要转换到map系下
        Eigen::Affine3d transform_eigen;
        // 手动设置变换矩阵
        transform_eigen.translation() = Eigen::Vector3d(transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z);
        Eigen::Quaterniond q(transform.transform.rotation.w, transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z);
        transform_eigen.linear() = q.toRotationMatrix();

        // 将点云转换到map系下
        pcl::transformPointCloud(*cloud, *cloud, transform_eigen.matrix().cast<float>());
        if (count_ < 20)
        {
            // 把cloud添加到cloud_source_
            *cloud_source_ += *cloud;
            count_++;
        }
        else
        {
            // 将 pcl 点云转换为 Open3D 格式
            open3d::geometry::PointCloud o3d_cloud;
            for (const auto &point : cloud_source_->points)
            {
                o3d_cloud.points_.emplace_back(point.x, point.y, point.z);
            }
            open3d::geometry::PointCloud o3d_cloud_target;
            for (const auto &point : cloud_target_->points)
            {
                o3d_cloud_target.points_.emplace_back(point.x, point.y, point.z);
            }
            // 设置点云颜色
            o3d_cloud.colors_.resize(o3d_cloud.points_.size());
            for (size_t i = 0; i < o3d_cloud.points_.size(); ++i)
            {
                o3d_cloud.colors_[i] = Eigen::Vector3d(1.0, 0.0, 0.0); // 设置颜色为红色
            }
            o3d_cloud_target.colors_.resize(o3d_cloud_target.points_.size());
            for (size_t i = 0; i < o3d_cloud_target.points_.size(); ++i)
            {
                o3d_cloud_target.colors_[i] = Eigen::Vector3d(0.0, 1.0, 0.0); // 设置颜色为红色
            }
            // 如果是第一次接收到点云消息，添加点云到可视化窗口
            if (!is_initialized_)
            {
                //清空
                viewer_->ClearGeometries();
                viewer_->AddGeometry(std::make_shared<open3d::geometry::PointCloud>(o3d_cloud));
                viewer_->AddGeometry(std::make_shared<open3d::geometry::PointCloud>(o3d_cloud_target));
                // is_initialized_ = true;
            }
            else
            {
                // 更新点云
                std::cout<<"update point cloud"<<std::endl;
                
            }
            count_ = 0;
            cloud_source_->clear();
        }
        // 更新可视化窗口
        viewer_->PollEvents();
        viewer_->UpdateRender();
    }

    void spin()
    {
        // 用于处理 ROS 2 回调的方式
        rclcpp::Rate rate(100);
        while (rclcpp::ok())
        {
            rclcpp::spin_some(this->shared_from_this());
            viewer_->PollEvents();
            viewer_->UpdateRender();
            rate.sleep();
        }
    }
    void setDefaultView()
    {
        auto view_control = viewer_->GetViewControl();

        // 设置相机的位置、目标和上方向
        view_control.SetFront(Eigen::Vector3d(0.5, -1.0, -0.5)); // 设置相机的斜视方向
        view_control.SetLookat(Eigen::Vector3d(0.0, 0.0, 0.0));  // 观察点
        view_control.SetUp(Eigen::Vector3d(0.0, 0.0, 1.0));      // 上方向
        view_control.SetZoom(0.8);                               // 设置缩放因子
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscriber_;
    std::shared_ptr<open3d::visualization::Visualizer> viewer_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target_{new pcl::PointCloud<pcl::PointXYZ>};
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_{new pcl::PointCloud<pcl::PointXYZ>};
    bool is_initialized_ = false; // 用于检查是否第一次添加点云
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    bool show_ = false;
    int count_ = 0;
};

int main(int argc, char *argv[])
{
    // 初始化ROS 2节点
    rclcpp::init(argc, argv);

    // 创建并运行节点
    auto node = std::make_shared<PointCloudVisualizer>();
    node->spin();

    // 清理并退出
    rclcpp::shutdown();
    return 0;
}
