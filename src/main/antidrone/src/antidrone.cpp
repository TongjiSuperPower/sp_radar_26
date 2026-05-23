#include "antidrone.hpp"


Antidrone::Antidrone() : Node("anti_drone"), gimbal_("./src/main/antidrone/io/configs/camera.yaml")
{
    RCLCPP_INFO(this->get_logger(), "antidrone node start");

    std::string config_file = "./src/main/antidrone/config/antidrone.yaml";
    const auto yaml_config = YAML::LoadFile(config_file);

    std::vector<double> TUM_L2C = yaml_config["TUM_lidar_camera"].as<std::vector<double>>(); 
    transform_C2L_ = tum_to_transform_stamped(TUM_L2C);
    transform_L2C_ = inverse_transform(transform_C2L_); 

    std::vector<double> TUM_G2W = yaml_config["TUM_gimbal_world"].as<std::vector<double>>(); 
    transform_G2W_ = tum_to_transform_stamped(TUM_G2W);
    transform_W2G_ = inverse_transform(transform_G2W_); 

    std::vector<std::vector<double>> T_cam2gimbal_vec = yaml_config["T_camera2gimbal"].as<std::vector<std::vector<double>>>();
    T_camera2gimbal_ = cv::Mat::eye(4, 4, CV_64F);
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            T_camera2gimbal_.at<double>(i, j) = T_cam2gimbal_vec[i][j];
        }
    }

    std::vector<std::vector<double>> T_gimbal2world_vec = yaml_config["T_gimbal_world"].as<std::vector<std::vector<double>>>();
    T_gimbal2world_ = cv::Mat::eye(4, 4, CV_64F);
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            T_gimbal2world_.at<double>(i, j) = T_gimbal2world_vec[i][j];
        }
    }

    // >>> 新增：把 TUM 向量转换为 4x4 齐次矩阵并相乘得到 lidar->world
    auto tum_to_mat4 = [](const std::vector<double> &TUM) -> cv::Mat {
        cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
        // 平移
        T.at<double>(0, 3) = TUM[0];
        T.at<double>(1, 3) = TUM[1];
        T.at<double>(2, 3) = TUM[2];
        // 四元数 (qx,qy,qz,qw) -> 3x3 旋转矩阵
        double qx = TUM[3], qy = TUM[4], qz = TUM[5], qw = TUM[6];
        double r00 = 1 - 2 * (qy*qy + qz*qz);
        double r01 = 2 * (qx*qy - qz*qw);
        double r02 = 2 * (qx*qz + qy*qw);
        double r10 = 2 * (qx*qy + qz*qw);
        double r11 = 1 - 2 * (qx*qx + qz*qz);
        double r12 = 2 * (qy*qz - qx*qw);
        double r20 = 2 * (qx*qz - qy*qw);
        double r21 = 2 * (qy*qz + qx*qw);
        double r22 = 1 - 2 * (qx*qx + qy*qy);
        T.at<double>(0,0) = r00; T.at<double>(0,1) = r01; T.at<double>(0,2) = r02;
        T.at<double>(1,0) = r10; T.at<double>(1,1) = r11; T.at<double>(1,2) = r12;
        T.at<double>(2,0) = r20; T.at<double>(2,1) = r21; T.at<double>(2,2) = r22;
        return T;
    };

    // 使用原先读取的 TUM 向量（与上面相同的 yaml 变量）
    cv::Mat T_lidar2camera = tum_to_mat4(TUM_L2C).inv();   // lidar -> camera
    // cv::Mat T_camera2lidar = tum_to_mat4(TUM_L2C).inv(); // 取反得到 camera -> lidar
    cv::Mat T_gimbal2world = tum_to_mat4(TUM_G2W);  // gimbal -> world
    

    // 组合：p_world = T_gimbal2world * T_camera2gimbal_ * T_lidar2camera * p_lidar
    T_lidar2world_ = T_gimbal2world * T_camera2gimbal_ * T_lidar2camera;

    RCLCPP_INFO(this->get_logger(), "Computed T_lidar->world (4x4):");
    std::cout << "T_gimbal->world:\n" << T_gimbal2world << std::endl;
    std::cout << "T_camera->gimbal:\n" << T_camera2gimbal_ << std::endl;
    std::cout << "T_lidar->camera:\n" << T_lidar2camera << std::endl;
    std::cout << "T_lidar->world:\n" << T_lidar2world_ << std::endl;


    // std::vector<double> cam_matrix_vec = yaml_config["camera_matrix"].as<std::vector<double>>();
    // camera_matrix_ = (cv::Mat_<double>(3, 3) << 
    //     cam_matrix_vec[0], cam_matrix_vec[1], cam_matrix_vec[2],
    //     cam_matrix_vec[3], cam_matrix_vec[4], cam_matrix_vec[5],
    //     cam_matrix_vec[6], cam_matrix_vec[7], cam_matrix_vec[8]);

    // dist_coeffs_ = yaml_config["distort_coeffs"].as<std::vector<double>>();
        
    // RCLCPP_INFO(this->get_logger(), "All transforms loaded successfully");
    
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/livox/clustered_lidar", 10, 
        std::bind(&Antidrone::Callback, this, std::placeholders::_1));
}

void Antidrone::Callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    pcl::PointCloud<pcl::PointXYZ> lidar_cloud;
    std::cout  << "Received point cloud with " << msg->width * msg->height << " points" << std::endl;
    // pcl::fromROSMsg(*msg, camera_cloud); // 直接转换为 pcl 点云
    pcl::fromROSMsg(*msg, lidar_cloud); // 直接转换为 pcl 点云
    std::cout << "Converted to PCL point cloud with " << lidar_cloud.points.size() << " points" << std::endl;
    auto g_s = gimbal_.state();
    std::cout << "Current gimbal state: yaw=" << g_s.yaw* 180.0 / M_PI << "°, pitch=" << g_s.pitch* 180.0 / M_PI << "°" << std::endl;
    
    auto now_steady = std::chrono::steady_clock::now();
    auto g_q = gimbal_.q(now_steady);
    std::cout << "gimbal orientation (quaternion) - x: " << g_q.x() << ", y: " << g_q.y() << ", z: " << g_q.z() << ", w: " << g_q.w() << std::endl;
        
    tools::Solver solver;
    // auto q = gimbal_.q(timestamp);
    solver.set_R_gimbal2world(g_q);
    auto gimbal_world = solver.R_gimbal2world();
    std::cout << "gimbal to world rotation matrix:\n" << gimbal_world << std::endl;
    // Eigen::Vector3d p_gimbal(rel_gim.x, rel_gim.y, rel_gim.z);
    // Eigen::Vector3d p_world = solver.R_gimbal2world() * p_gimbal;
    
    // get the first point
    if (lidar_cloud.points.empty()) {
        RCLCPP_WARN(this->get_logger(), "Point cloud is empty!");
        return;
    }
    cv::Mat first_point_mat = (cv::Mat_<double>(4, 1) << lidar_cloud.points[0].x, lidar_cloud.points[0].y, lidar_cloud.points[0].z, 1.0);
    cv::Mat first_point_in_world = T_lidar2world_ * first_point_mat;

    double world_x = first_point_in_world.at<double>(0);
    double world_y = first_point_in_world.at<double>(1);
    double world_z = first_point_in_world.at<double>(2);

    double target_yaw = std::atan2(world_y, world_x);
    double target_pitch = -std::atan2(world_z, sqrt(world_x * world_x + world_y * world_y));
    std::cout << "First point in world coordinates: (" << world_x << ", " << world_y << ", " << world_z << ")" << std::endl;
    std::cout << "yaw: " << target_yaw * 180.0 / M_PI << "°, pitch: " << target_pitch * 180.0 / M_PI << "°" << std::endl;
    gimbal_.send(1, 0, target_yaw, 0, 0, target_pitch, 0, 0);

    // transform_point_cloud(msg, transform_L2C_, camera_cloud);
    // rclcpp::Time ros_time = msg->header.stamp;
    
    // rclcpp::Time now_ros = this->now();
    // auto now_steady = std::chrono::steady_clock::now();
    
    // auto ros_offset = (ros_time - now_ros).nanoseconds();
    // auto steady_timestamp = now_steady + std::chrono::nanoseconds(ros_offset);
    // std::chrono::steady_clock::time_point timestamp = steady_timestamp;

    // for (auto &point : camera_cloud.points)
    // {
    //     cv::Mat p_camera = (cv::Mat_<double>(4, 1) << point.x, point.y, point.z, 1.0);
    //     cv::Mat p_gimbal_h = T_camera2gimbal_ * p_camera;
    //     cv::Point3d rel_gim(p_gimbal_h.at<double>(0), p_gimbal_h.at<double>(1), p_gimbal_h.at<double>(2));

    //     tools::Solver solver;
    //     auto q = gimbal_.q(timestamp);
    //     solver.set_R_gimbal2world(q);
    //     Eigen::Vector3d p_gimbal(rel_gim.x, rel_gim.y, rel_gim.z);
    //     Eigen::Vector3d p_world = solver.R_gimbal2world() * p_gimbal;
        
    //     // Calculate world coordinates and angles
    //     double world_x = p_world.x();
    //     double world_y = p_world.y();
    //     double world_z = p_world.z();
        
    //     double target_yaw = -std::atan2(world_y, world_x);
    //     double target_pitch = -std::atan2(world_z, sqrt(world_x * world_x + world_y * world_y));
    //     double distance = sqrt(world_x*world_x + world_y*world_y + world_z*world_z);

        
    // }
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