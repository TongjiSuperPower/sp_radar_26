#include "../include/rosbag_pointcloud_projection.hpp"


PointCloudSubscriber::PointCloudSubscriber() : Node("point_cloud_subscriber")
{
    std::string config_file;
    this->declare_parameter<std::string>("config_file", "");
    this->get_parameter("config_file", config_file);
    const auto config = YAML::LoadFile(config_file);

    img_subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "/radar/hik_6mm/image/compressed", 10, std::bind(&PointCloudSubscriber::image_callback, this, std::placeholders::_1));
    std::vector<double> camera_matrix = config["camera_matrix"].as<std::vector<double>>();
    camera_matrix_ = cv::Mat(3, 3, CV_64F, camera_matrix.data()).clone();
    std::vector<double> distort_coeffs = config["distort_coeffs"].as<std::vector<double>>();
    distort_coeffs_ = cv::Mat(distort_coeffs.size(), 1, CV_64F, distort_coeffs.data()).clone();

    point_queue_ = std::queue<std::vector<cv::Point2f>>();
    features_queue_ = std::queue<std::vector<double>>();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    pc_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/radar/lidar_hap/pc_raw", 10, std::bind(&PointCloudSubscriber::point_cloud_callback, this, std::placeholders::_1));
}

void PointCloudSubscriber::image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    try
    {
        // 将CompressedImage消息转换为OpenCV图像
        cv::Mat img = cv::imdecode(msg->data, cv::IMREAD_COLOR);
        img_ = img.clone();  // 更新图像变量
        RCLCPP_INFO(this->get_logger(), "Received CompressedImage");
    }
    catch (std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "img decode exception: %s", e.what());
    }
}

void PointCloudSubscriber::point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    cv::namedWindow("img", 0);
    cv::resizeWindow("img", cv::Size(1280, 960));
    try
    {
        geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
            "camera_frame",
            "lidar_frame",
            rclcpp::Time(0)
        );
        pcl::PointCloud<pcl::PointXYZI> transformed_cloud;
        
        // lidar_frame -> camera_frame
        transform_point_cloud(msg, transform, transformed_cloud);
        
        if(!img_.empty())
        {
            pointclouds_to_image(transformed_cloud, img_);
            cv::imshow("img", img_);
            cv::waitKey(1);
        }
        else
            RCLCPP_INFO(this->get_logger(), "Received PointCloud but image is not available yet");
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_ERROR(this->get_logger(), "Could not transform: %s", ex.what());
    }
}

void PointCloudSubscriber::transform_point_cloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr &msg,
    const geometry_msgs::msg::TransformStamped &transform,
    pcl::PointCloud<pcl::PointXYZI> &transformed_cloud)
{
    // 使用 PCL 库进行点云转换
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(*msg, cloud); // 将 ROS PointCloud2 转换为 PCL 点云

    transformed_cloud.header = cloud.header;

    // 将每个点从 lidar_frame 转换到 camera_frame
    for (auto &point : cloud.points)
    {
        // 创建点
        geometry_msgs::msg::PointStamped point_in_lidar;
        point_in_lidar.header.frame_id = "lidar_frame";
        point_in_lidar.point.x = point.x;
        point_in_lidar.point.y = point.y;
        point_in_lidar.point.z = point.z;
        
        // 使用 tf 进行转换
        geometry_msgs::msg::PointStamped point_in_camera;
        try
        {
            tf2::doTransform(point_in_lidar, point_in_camera, transform);
            // 转换后的点坐标
            pcl::PointXYZI transformed_point(point_in_camera.point.x, point_in_camera.point.y, point_in_camera.point.z, point.intensity);
            transformed_cloud.points.push_back(transformed_point);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
        }
    }

    // 打印转换后的点云数量
    RCLCPP_INFO(this->get_logger(), "Transformed %zu points", transformed_cloud.points.size());
}

void PointCloudSubscriber::pointclouds_to_image(const pcl::PointCloud<pcl::PointXYZI> &cloud, cv::Mat &img)
{
    std::vector<cv::Point3f> obj_points;
    std::vector<double> obj_points_features;
    for (auto &point : cloud.points)
    {
        cv::Point3f obj_point(point.y, point.z, point.x);
        obj_points.push_back(obj_point);
        if (obj_point.z > max_distance_)
        {
            max_distance_ = obj_point.z;
        }
        obj_points_features.push_back(point.intensity);
    }

    std::vector<cv::Point2f> reprojected_points;
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);

    cv::projectPoints(obj_points, rvec, tvec, camera_matrix_, distort_coeffs_, reprojected_points);
    // 将计算后的2D点存入队列
    if (point_queue_.size() >= 20)
    {
        features_queue_.pop(); // 弹出第一个元素
        point_queue_.pop();    // 弹出第一个元素
    }
    features_queue_.push(obj_points_features); // 存入新值
    point_queue_.push(reprojected_points);     // 存入新值

    // 画点
    std::queue<std::vector<cv::Point2f>> temp_queue = point_queue_;       // 创建临时队列
    std::queue<std::vector<double>> temp_feature_queue = features_queue_; // 创建临时队列

    while (!temp_queue.empty())
    {
        std::vector<cv::Point2f> points = temp_queue.front();
        std::vector<double> features = temp_feature_queue.front();
        temp_feature_queue.pop();
        temp_queue.pop();
        for (unsigned int i = 0; i < points.size(); i++)
        {
            int colorValue = cv::saturate_cast<int>(features[i] * 255); // 从 0 到 255; // 从 0 到 255
            // std::cout<<"colorValue:"<<colorValue<<std::endl;
            cv::circle(img, points[i], 1, cv::Scalar((255 - colorValue*10)%256, (colorValue*10)%256, (128 + colorValue*10)%256 ), -1);
            // std::cout << "reprojected_points[" << i << "]:" << reprojected_points[i] << std::endl;
        }
    }
}

int main(int argc, char **argv)
{
    // 初始化 ROS 2 系统
    rclcpp::init(argc, argv);

    // 创建并运行点云订阅节点
    rclcpp::spin(std::make_shared<PointCloudSubscriber>());

    // 关闭 ROS 2
    rclcpp::shutdown();
    return 0;
}
