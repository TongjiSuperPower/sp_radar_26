#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <radar_msgs/msg/cars_and_drones.hpp>

class PointcloudDebugger : public rclcpp::Node
{
public:
    PointcloudDebugger()
        : Node("pointcloud_debugger")
    {
        // 订阅 filter 输出的 CarsAndDrones
        filtered_sub_ = this->create_subscription<radar_msgs::msg::CarsAndDrones>(
            "/livox/filtered_lidar", 10,
            std::bind(&PointcloudDebugger::filtered_callback, this, std::placeholders::_1));

        // 订阅 cluster 输出的 CarsAndDrones
        clustered_sub_ = this->create_subscription<radar_msgs::msg::CarsAndDrones>(
            "/livox/clustered_lidar", 10,
            std::bind(&PointcloudDebugger::clustered_callback, this, std::placeholders::_1));

        // 四个 debug 发布者
        pub_filtered_cars_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/debug/filtered_cars", 10);
        pub_filtered_drones_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/debug/filtered_drones", 10);
        pub_clustered_cars_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/debug/clustered_cars", 10);
        pub_clustered_drones_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/debug/clustered_drones", 10);

        RCLCPP_INFO(this->get_logger(), "pointcloud_debugger start");
    }

private:
    void filtered_callback(const radar_msgs::msg::CarsAndDrones::SharedPtr msg)
    {
        pub_filtered_cars_->publish(msg->cars_cloud);
        pub_filtered_drones_->publish(msg->drones_cloud);
    }

    void clustered_callback(const radar_msgs::msg::CarsAndDrones::SharedPtr msg)
    {
        pub_clustered_cars_->publish(msg->cars_cloud);
        pub_clustered_drones_->publish(msg->drones_cloud);
    }

    rclcpp::Subscription<radar_msgs::msg::CarsAndDrones>::SharedPtr filtered_sub_;
    rclcpp::Subscription<radar_msgs::msg::CarsAndDrones>::SharedPtr clustered_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_filtered_cars_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_filtered_drones_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_clustered_cars_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_clustered_drones_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointcloudDebugger>());
    rclcpp::shutdown();
    return 0;
}