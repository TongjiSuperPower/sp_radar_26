#include "antidrone.hpp"

Antidrone::Antidrone() : Node("anti_drone")
{
    RCLCPP_INFO(this->get_logger(), "antidrone node start");

    std::string config_file = "./src/main/antidrone/config/antidrone.yaml";
    // get_parameter("config_file", config_file);
    const auto yaml_config = YAML::LoadFile(config_file);
    std::vector<double> TUM_L2G = yaml_config["TUM_lidar2gimball"].as<std::vector<double>>(); 
    transform_G2L_ = tum_to_transform_stamped(TUM_L2G);
    transform_L2G_ = inverse_transform(transform_G2L_); 

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/livox/clustered_lidar_drone", 10, std::bind(&Antidrone::Callback, this, std::placeholders::_1));


}

void Antidrone::Callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud; // points in camera
    transform_point_cloud(msg, transform_L2G_, transformed_cloud);
    for (auto &point : transformed_cloud.points)
    {
        double yaw = atan(point.y/point.x);
        double pitch = atan(point.z/sqrt(point.y * point.y + point.x * point.x));
        std::cout << "yaw and pitch: " << yaw << ", " << pitch << std::endl;
    }
}

void Antidrone::transform_point_cloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr &msg,
    const geometry_msgs::msg::TransformStamped &transform,
    pcl::PointCloud<pcl::PointXYZ> &transformed_cloud)
{   
    pcl::PointCloud<pcl::PointXYZ> cloud; 
    pcl::fromROSMsg(*msg, cloud);     
    transformed_cloud.header = cloud.header;

    for (auto &point : cloud.points)
    {
        geometry_msgs::msg::PointStamped point_in_lidar;
        point_in_lidar.header.frame_id = "lidar_frame";
        point_in_lidar.point.x = point.x;
        point_in_lidar.point.y = point.y;
        point_in_lidar.point.z = point.z;        
        geometry_msgs::msg::PointStamped point_in_gimball;
        try
        {
            tf2::doTransform(point_in_lidar, point_in_gimball, transform);
            // 转换后的点坐标
            pcl::PointXYZ transformed_point(point_in_gimball.point.x, point_in_gimball.point.y, point_in_gimball.point.z);
            transformed_cloud.points.push_back(transformed_point);
        }

        catch (tf2::TransformException &ex)
        {
            //RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
        }
    }
    //RCLCPP_INFO(this->get_logger(), "Transformed %zu points", transformed_cloud.points.size());
}

geometry_msgs::msg::TransformStamped Antidrone::tum_to_transform_stamped(std::vector<double> TUM)
{
    geometry_msgs::msg::TransformStamped transform;
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();


    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
    auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration - seconds);

    // 设置时间戳
    transform.header.stamp.sec = seconds.count();
    transform.header.stamp.nanosec = nanoseconds.count();

    // 设置坐标系ID，从雷达坐标系到相机坐标系
    transform.header.frame_id = "lidar_frame";
    transform.child_frame_id = "gimball_frame";

    // 设置平移
    transform.transform.translation.x = TUM[0];
    transform.transform.translation.y = TUM[1];
    transform.transform.translation.z = TUM[2];

    // 设置旋转（四元数）
    transform.transform.rotation.x = TUM[3];
    transform.transform.rotation.y = TUM[4];
    transform.transform.rotation.z = TUM[5];
    transform.transform.rotation.w = TUM[6];

    return transform;
}

geometry_msgs::msg::TransformStamped Antidrone::inverse_transform(
    const geometry_msgs::msg::TransformStamped &transform)
{
    // 创建逆变换对象
    geometry_msgs::msg::TransformStamped inverse;

    // 交换源坐标系和目标坐标系
    inverse.header.frame_id = transform.child_frame_id;
    inverse.child_frame_id = transform.header.frame_id;
    inverse.header.stamp = transform.header.stamp;

    // 将原始变换转换为 tf"has some points outside of view"2::Transform
    tf2::Transform tf_transform;
    tf2::fromMsg(transform.transform, tf_transform);

    // 计算逆变换
    tf2::Transform tf_inverse = tf_transform.inverse();

    // 将逆变换转换回 geometry_msgslocate
    inverse.transform = tf2::toMsg(tf_inverse);

    return inverse;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Antidrone>());
    rclcpp::shutdown();
    return 0;
}