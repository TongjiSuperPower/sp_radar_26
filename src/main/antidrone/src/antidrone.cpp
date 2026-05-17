#include "antidrone.hpp"

Antidrone::Antidrone() : Node("anti_drone"), gimbal_("/home/radar/Desktop/radar26/sp_radar_26/src/main/antidrone/io/configs/camera.yaml")
{
    RCLCPP_INFO(this->get_logger(), "antidrone node start");

    std::string config_file = "./src/main/antidrone/config/antidrone.yaml";
    const auto yaml_config = YAML::LoadFile(config_file);

    std::vector<double> TUM_L2C = yaml_config["TUM_lidar_camera"].as<std::vector<double>>(); 
    transform_C2L_ = tum_to_transform_stamped(TUM_L2C);
    transform_L2C_ = inverse_transform(transform_C2L_); 
    std::vector<std::vector<double>> T_cam2gimbal_vec = yaml_config["T_camera2gimbal"].as<std::vector<std::vector<double>>>();
    T_camera2gimbal_ = cv::Mat::eye(4, 4, CV_64F);
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            T_camera2gimbal_.at<double>(i, j) = T_cam2gimbal_vec[i][j];
        }
    }
    
    std::vector<double> cam_matrix_vec = yaml_config["camera_matrix"].as<std::vector<double>>();
    camera_matrix_ = (cv::Mat_<double>(3, 3) << 
        cam_matrix_vec[0], cam_matrix_vec[1], cam_matrix_vec[2],
        cam_matrix_vec[3], cam_matrix_vec[4], cam_matrix_vec[5],
        cam_matrix_vec[6], cam_matrix_vec[7], cam_matrix_vec[8]);

    dist_coeffs_ = yaml_config["distort_coeffs"].as<std::vector<double>>();
        
    RCLCPP_INFO(this->get_logger(), "All transforms loaded successfully");
    
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/livox/clustered_lidar_drone", 10, 
        std::bind(&Antidrone::Callback, this, std::placeholders::_1));
}

void Antidrone::Callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    pcl::PointCloud<pcl::PointXYZ> camera_cloud;
    transform_point_cloud(msg, transform_L2C_, camera_cloud);
    rclcpp::Time ros_time = msg->header.stamp;
    
    rclcpp::Time now_ros = this->now();
    auto now_steady = std::chrono::steady_clock::now();
    
    auto ros_offset = (ros_time - now_ros).nanoseconds();
    auto steady_timestamp = now_steady + std::chrono::nanoseconds(ros_offset);
    std::chrono::steady_clock::time_point timestamp = steady_timestamp;



    for (auto &point : camera_cloud.points)
    {
        cv::Mat p_camera = (cv::Mat_<double>(4, 1) << point.x, point.y, point.z, 1.0);
        cv::Mat p_gimbal_h = T_camera2gimbal_ * p_camera;
        cv::Point3d rel_gim(p_gimbal_h.at<double>(0), p_gimbal_h.at<double>(1), p_gimbal_h.at<double>(2));

        tools::Solver solver;
        auto q = gimbal_.q(timestamp);
        solver.set_R_gimbal2world(q);
        Eigen::Vector3d p_gimbal(rel_gim.x, rel_gim.y, rel_gim.z);
        Eigen::Vector3d p_world = solver.R_gimbal2world() * p_gimbal;
        
        // Calculate world coordinates and angles
        double world_x = p_world.x();
        double world_y = p_world.y();
        double world_z = p_world.z();
        
        double target_yaw = -std::atan2(world_y, world_x);
        double target_pitch = -std::atan2(world_z, sqrt(world_x * world_x + world_y * world_y));
        double distance = sqrt(world_x*world_x + world_y*world_y + world_z*world_z);

        
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
    transform.child_frame_id = "camera_frame";

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