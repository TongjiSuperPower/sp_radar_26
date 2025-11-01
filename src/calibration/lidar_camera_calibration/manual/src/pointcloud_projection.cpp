#include "../include/pointcloud_projection.hpp"

PointCloudSubscriber::PointCloudSubscriber() : Node("point_cloud_subscriber")
{
    std::string config_file;
    this->declare_parameter<std::string>("config_file", "");
    this->get_parameter("config_file", config_file);
    const auto config = YAML::LoadFile(config_file);
    auto config_path = config["config_path"].as<std::string>();

    use_camera = config["use_camera"].as<bool>();
    if (use_camera)
        camera_ = std::make_shared<io::Camera>(config_path);

    std::vector<double> camera_matrix = config["camera_matrix"].as<std::vector<double>>();
    camera_matrix_ = cv::Mat(3, 3, CV_64F, camera_matrix.data()).clone();
    std::vector<double> distort_coeffs = config["distort_coeffs"].as<std::vector<double>>();
    distort_coeffs_ = cv::Mat(distort_coeffs.size(), 1, CV_64F, distort_coeffs.data()).clone();

    point_queue_ = std::queue<std::vector<cv::Point2f>>();
    features_queue_ = std::queue<std::vector<double>>();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/livox/lidar", 10, std::bind(&PointCloudSubscriber::point_cloud_callback, this, std::placeholders::_1));
}

void PointCloudSubscriber::point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    try
    {
        std::chrono::steady_clock::time_point timestamp;

        if (use_camera)
            camera_->read(img_, timestamp);

        geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
            "camera_frame",
            "lidar_frame",
            rclcpp::Time(0));
        pcl::PointCloud<pcl::PointXYZI> transformed_cloud;

        // lidar_frame -> camera_frame
        transform_point_cloud(msg, transform, transformed_cloud);
        pointclouds_to_image(transformed_cloud, img_);
        cv::imshow("img", img_);
        cv::waitKey(1);
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
        // std::cout << "point:" << point << std::endl;
        cv::Point3f obj_point(-point.y, -point.z, point.x); // 由于坐标系不同，需要调整坐标
        obj_points.push_back(obj_point);
        if (obj_point.z > max_distance_)
        {
            max_distance_ = obj_point.z;
        }
        obj_points_features.push_back(point.intensity); // 使用x作为特征点
    }

    std::vector<cv::Point2f> reprojected_points; // 计算后的2D点
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);

    cv::projectPoints(obj_points, rvec, tvec, camera_matrix_, distort_coeffs_, reprojected_points);
    // 将计算后的2D点存入队列
    if (point_queue_.size() >= 8)
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
            cv::circle(img, points[i], 1, cv::Scalar((255 - colorValue * 10) % 256, (colorValue * 10) % 256, (128 + colorValue * 10) % 256), -1);
            // std::cout << "reprojected_points[" << i << "]:" << reprojected_points[i] << std::endl;
        }
    }
}

void PointCloudSubscriber::read(const cv::Mat &img)
{
    img_ = img.clone();
}

int main(int argc, char **argv)
{
    // 初始化 ROS 2 系统
    rclcpp::init(argc, argv);

    std::shared_ptr pc_sub = std::make_shared<PointCloudSubscriber>();

    // 创建并运行点云订阅节点
    if (pc_sub->use_camera)
        rclcpp::spin(pc_sub);
    else
    {
        std::shared_ptr<cv::VideoCapture> cap_;
        std::string config_file;
        pc_sub->get_parameter("config_file", config_file);
        const auto config = YAML::LoadFile(config_file);
        std::string video_path = config["video_path"].as<std::string>();
        cap_ = std::make_shared<cv::VideoCapture>(video_path);
        while (true)
        {
            cv::Mat img;
            cap_->read(img);
            if (img.empty())
                break;
            pc_sub->read(img);
            rclcpp::spin_some(pc_sub);
            // std::this_thread::sleep_for(std::chrono::milliseconds(15));
            std::this_thread::sleep_for(std::chrono::microseconds(14650));
            // 13700视频快
            // 13850视频快
            // 14550视频快一点
            // 14580视频快一点
        }
    }

    // 关闭 ROS 2
    rclcpp::shutdown();
    return 0;
}
