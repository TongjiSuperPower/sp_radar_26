
#include "include/locate.hpp"

PointcloudLocater::PointcloudLocater()
    : Node("point_cloud_subscriber"), point_queue_(record_point_cloud_frame)
{
    camera_time_ = 0;
    features_queue_ = std::queue<std::vector<double>>();
    std::string config_file;
    config_file = "src/main/locate/configs/locate.yaml";
    const auto config = YAML::LoadFile(config_file);
    auto config_path = config["config_path"].as<std::string>();
    debug_flag_ = config["debug"].as<bool>();
    std::vector<double> camera_matrix = config["camera_matrix"].as<std::vector<double>>();
    camera_matrix_ = cv::Mat(3, 3, CV_64F, camera_matrix.data()).clone();
    std::vector<double> distort_coeffs = config["distort_coeffs"].as<std::vector<double>>();
    distort_coeffs_ = cv::Mat(distort_coeffs.size(), 1, CV_64F, distort_coeffs.data()).clone();

    enemy_colour = config["enemy"].as<std::string>();

    std::vector<double> TUM_C2L = config["TUM_camera2lidar"].as<std::vector<double>>(); // TUM: from camera to lidar
    transform_C2L_ = tum_to_transform_stamped(TUM_C2L);
    transform_L2C_ = inverse_transform(transform_C2L_); // inverse it, now from lidar to camera
    transform_L2M_ = tum_to_transform_stamped(config["TUM_lidar2map"].as<std::vector<double>>());    
    
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 订阅 "lidar" 话题，处理点云数据
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/livox/clustered_lidar", 10, std::bind(&PointcloudLocater::point_cloud_callback, this, std::placeholders::_1));
    bbox_subscription_ = this->create_subscription<radar_msgs::msg::CarBbox>(
        "/car_bbox", 10, std::bind(&PointcloudLocater::bbox_callback, this, std::placeholders::_1));
    drone_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/livox/clustered_lidar_drone", 10, std::bind(&PointcloudLocater::drone_point_cloud_callback, this, std::placeholders::_1));
    car_publisher_ = this->create_publisher<radar_msgs::msg::Cars>("/map_robot_data_pc", 10);
    drone_publisher_ = this->create_publisher<radar_msgs::msg::Cars>("/map_drone_data_pc", 10);
    both_publisher_ = this->create_publisher<radar_msgs::msg::Cars>("/map_both_data_pc", 10);
    for (int i = 0; i < 12; i++)
    {
        std::vector<cv::Point3f> car;
        car_points_.push_back(car);
    }
}

void PointcloudLocater::point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    try
    {
        lidar_time_ = (1.0 * msg->header.stamp.sec * 1000 + msg->header.stamp.nanosec / 1e6);
        pcl::PointCloud<pcl::PointXYZ> transformed_cloud; // points in camera
        transform_point_cloud(msg, transform_L2C_, transformed_cloud);
        auto points_with_id = pointclouds_to_image(transformed_cloud, pointcloud_img_);
        locate(points_with_id);

        if (camera_time_ != 0) {
            static bool if_show_img = true;
            char key;
            if(if_show_img)
            {
                cv::resize(pointcloud_img_, pointcloud_img_, cv::Size(1080, 720));
                cv::imshow("pointcloud", pointcloud_img_);
                key = cv::waitKey(1);
            }
            if(key == 'q' || key == 'Q') {
                if_show_img = false;
                // cv::destroyWindow("pointcloud");
                cv::destroyAllWindows();
            }
        }
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_ERROR(this->get_logger(), "Could not transform: %s", ex.what());
    }
}

void PointcloudLocater::bbox_callback(const radar_msgs::msg::CarBbox::SharedPtr msg)
{
    // 上锁
     std::lock_guard<std::mutex> lock(mtx_);
    
    // 将 bboxes 按 y_max 升序排序
    for (unsigned long int i = 0; i < msg->bboxs.size(); i++) {
        for (unsigned long int j = 0; j < msg->bboxs.size() - i - 1; j++) {
            if (msg->bboxs[j].y_max > msg->bboxs[j + 1].y_max) {
                std::swap(msg->bboxs[j], msg->bboxs[j + 1]);
            }
        }
    }

    bbox_msg_ = msg;
    camera_time_ = (1.0 * bbox_msg_->header.stamp.sec * 1000 + bbox_msg_->header.stamp.nanosec / 1e6);
    // 创建一个空白图像
    pointcloud_img_ = cv::Mat::zeros(msg->img_height, msg->img_width, CV_8UC3);

    // 在图像中画框
    for (const auto &bbox : msg->bboxs)
    {
        cv::Scalar color = (bbox.class_id >= 6 ? cv::Scalar(50, 50, 255) : cv::Scalar(255, 50, 50));
        cv::rectangle(pointcloud_img_, cv::Point(bbox.x_min, bbox.y_min), cv::Point(bbox.x_max, bbox.y_max), color, 2);
        // 在框的左上角加上文字
        cv::putText(pointcloud_img_, std::to_string(bbox.class_id % 6 + 1), cv::Point(bbox.x_min, bbox.y_min - 60),
                    cv::FONT_HERSHEY_SIMPLEX, 1, color, 2);
        cv::putText(pointcloud_img_, std::to_string(bbox.class_confidence), cv::Point(bbox.x_min, bbox.y_min - 20),
                    cv::FONT_HERSHEY_SIMPLEX, 1, color, 2);
    }

    // 显示图像
    // cv::resize(pointcloud_img_, pointcloud_img_, cv::Size(pointcloud_img_.cols / 2, pointcloud_img_.rows / 2));
    // cv::imshow("BBox Image", pointcloud_img_);
    // cv::waitKey(1);
}

void PointcloudLocater::drone_point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // 将 ROS 点云转换为 PCL 格式
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    // 准备需要发布的消息
    radar_msgs::msg::Cars drone_cars_msg;
    drone_cars_msg.header.frame_id = "map";
    drone_cars_msg.header.stamp = msg->header.stamp;   // 使用点云原始时间戳
   
    std::vector<radar_msgs::msg::Car> new_drone_cars;
   
    try {
        // 获取从雷达坐标系到地图坐标系的变换
        auto transform_L2M = tf_buffer_->lookupTransform(
            "map", "lidar_frame", tf2::TimePointZero);

        for (const auto& point : cloud.points) {
            geometry_msgs::msg::PointStamped pt_lidar, pt_map;
            pt_lidar.header.frame_id = "lidar_frame";
            pt_lidar.point.x = point.x;
            pt_lidar.point.y = point.y;
            pt_lidar.point.z = point.z;

            tf2::doTransform(pt_lidar, pt_map, transform_L2M);

            radar_msgs::msg::Car drone;
            drone.x = pt_map.point.x;
            drone.y = pt_map.point.y;
            if (enemy_colour == "red") {
                drone.class_id = 10;   // 红方无人机 ID
            } else {
                drone.class_id = 4;   // 无人机固定 ID
            }
            drone_cars_msg.cars.push_back(drone);
            new_drone_cars.push_back(drone);
        }
    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "Drone transform error: %s", ex.what());
        return;
    }
    {
        std::lock_guard<std::mutex> lock(mtx_);
        latest_drone_cars_ = std::move(new_drone_cars);
    }

    // 发布无人机目标
    drone_publisher_->publish(drone_cars_msg);
    publish_combined();
}

void PointcloudLocater::transform_point_cloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr &msg,
    const geometry_msgs::msg::TransformStamped &transform,
    pcl::PointCloud<pcl::PointXYZ> &transformed_cloud)
{
    // 使用 PCL 库进行点云转换
    pcl::PointCloud<pcl::PointXYZ> cloud; // 在雷达坐标系
    pcl::fromROSMsg(*msg, cloud);         // 将 ROS PointCloud2 转换为 PCL 点云

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
            pcl::PointXYZ transformed_point(point_in_camera.point.x, point_in_camera.point.y, point_in_camera.point.z);
            transformed_cloud.points.push_back(transformed_point);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
        }

    }
    // tf2::doTransform(cloud)

    // 打印转换后的点云数量
    RCLCPP_INFO(this->get_logger(), "Transformed %zu points", transformed_cloud.points.size());
}

std::vector<std::pair<pcl::PointXYZ, int>> PointcloudLocater::pointclouds_to_image(const pcl::PointCloud<pcl::PointXYZ> &cloud, cv::Mat &img)
{
    // 上锁
    std::lock_guard<std::mutex> lock(mtx_);
    std::vector<std::pair<pcl::PointXYZ, int>> car_points;
    if(camera_time_ == 0) return car_points;
    std::vector<cv::Point3f> obj_points; // 相机坐标系
    std::vector<double> obj_points_features;
    for (auto &point : cloud.points)
    {
        cv::Point3f obj_point(point.x, point.y, point.z);
        obj_points.push_back(obj_point);
        if (obj_point.x > max_distance_)
        {
            max_distance_ = obj_point.x;
        }
        obj_points_features.push_back(obj_point.x); // 使用x作为特征点
    }
    
    std::vector<cv::Point2f> reprojected_points; // 投影至像素平面的点
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
    if (obj_points.size() > 0)
    {
        cv::projectPoints(obj_points, rvec, tvec, camera_matrix_, distort_coeffs_, reprojected_points);
        double delta_time = abs(lidar_time_ - camera_time_);
        RCLCPP_INFO(this->get_logger(), "delta_time: %f", delta_time);
        // 时间差<0.2s
        if (delta_time < 200 || debug_flag_) {   // debug用的rosbag，所以雷达和相机时间不一致
            for (unsigned long int i = 0; i < reprojected_points.size(); i++) {
                int id = -1;
                for (const auto &bbox : bbox_msg_->bboxs) {                    
                    if (reprojected_points[i].x > bbox.x_min && reprojected_points[i].x < bbox.x_max && 
                        reprojected_points[i].y > bbox.y_min && reprojected_points[i].y < bbox.y_max) {                 
                        // car_points_[cout].push_back(obj_points[i]);
                        id = bbox.class_id;
                    }
                }
                car_points.push_back(std::make_pair(cloud.points[i], id));  // TODO 在目标相遇的时候，只会取最后一个投影到的框
                std::cout << "point tryouts for locate: " << reprojected_points[i].x << "," << reprojected_points[i].y << std::endl;
                int colorValue = cv::saturate_cast<int>(obj_points_features[i] / max_distance_ * 255); // 从 0 到 255; // 从 0 到 255
                try {
                    cv::circle(img, reprojected_points[i], 10, cv::Scalar(255 - colorValue / 2, 127 + colorValue / 2, 127 + colorValue / 2), -1);
                }
                catch (cv::Exception) {
                    throw "cv over range when draw circle";
                }
            }
        } 
    }  
    return car_points;                 
}

void PointcloudLocater::locate(std::vector<std::pair<pcl::PointXYZ, int>> car_centers)
{
    //std::lock_guard<std::mutex> lock(mtx_);

    std::vector<radar_msgs::msg::Car> new_ground_cars;
    
    radar_msgs::msg::Cars map_robot_center;
    map_robot_center.header.frame_id = "map";
    map_robot_center.header.stamp = this->now();
    try
    {
        transform_L2M_ = tf_buffer_->lookupTransform(
            "map",
            "lidar_frame",
            rclcpp::Time(0));
        RCLCPP_INFO(this->get_logger(), "tvec: %f, %f, %f, q: %f, %f, %f: %f", 
            transform_L2M_.transform.translation.x, 
            transform_L2M_.transform.translation.y,
            transform_L2M_.transform.translation.z,
            transform_L2M_.transform.rotation.x,
            transform_L2M_.transform.rotation.y,
            transform_L2M_.transform.rotation.z,
            transform_L2M_.transform.rotation.w
        );
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_ERROR(this->get_logger(), "Could not transform: %s", ex.what());
    }

    for (auto &point_id : car_centers) {
        geometry_msgs::msg::PointStamped car_center_in_camera;
        car_center_in_camera.point.x = point_id.first.x;
        car_center_in_camera.point.y = point_id.first.y;
        car_center_in_camera.point.z = point_id.first.z;
    
        geometry_msgs::msg::PointStamped car_center_in_map, car_center_in_lidar;
        tf2::doTransform(car_center_in_camera, car_center_in_lidar, transform_C2L_);
        tf2::doTransform(car_center_in_lidar, car_center_in_map, transform_L2M_);
        
        radar_msgs::msg::Car car_center;
        car_center.x = car_center_in_map.point.x;
        car_center.y = car_center_in_map.point.y;
        car_center.class_id = point_id.second;
        new_ground_cars.push_back(car_center);
        map_robot_center.cars.push_back(car_center);
        // map_robot_center.cars.emplace_back(car_center_in_map.point.x, car_center_in_map.point.y, point_id.second);
        // std::string text = fmt::format("x = {0:.2}, y = {1:.2}, z = {2:.2}", car_center_in_map.point.x, car_center_in_map.point.y, car_center_in_map.point.z);
        // std::string text = fmt::format("x = {0:.2}, y = {1:.2}, z = {2:.2}", car_center_in_map.point.x, car_center_in_map.point.y, car_center_in_map.point.z);
        // cv::putText(pointcloud_img_, text, cv::Point(bbox_msg_->bboxs[i].x_max, bbox_msg_->bboxs[i].y_min), 0, 1.5, cv::Scalar(255,255,255), 2);
    }
    {
        std::lock_guard<std::mutex> lock(mtx_);
        latest_ground_cars_ = std::move(new_ground_cars);
    }

    car_publisher_->publish(map_robot_center);
    publish_combined();
}

geometry_msgs::msg::TransformStamped PointcloudLocater::tum_to_transform_stamped(std::vector<double> TUM)
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
    transform.header.frame_id = "camera_frame";
    transform.child_frame_id = "lidar_frame";

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

geometry_msgs::msg::TransformStamped PointcloudLocater::inverse_transform(
    const geometry_msgs::msg::TransformStamped &transform)
{
    // 创建逆变换对象
    geometry_msgs::msg::TransformStamped inverse;

    // 交换源坐标系和目标坐标系
    inverse.header.frame_id = transform.child_frame_id;
    inverse.child_frame_id = transform.header.frame_id;
    inverse.header.stamp = transform.header.stamp;

    // 将原始变换转换为 tf2::Transform
    tf2::Transform tf_transform;
    tf2::fromMsg(transform.transform, tf_transform);

    // 计算逆变换
    tf2::Transform tf_inverse = tf_transform.inverse();

    // 将逆变换转换回 geometry_msgs
    inverse.transform = tf2::toMsg(tf_inverse);

    return inverse;
}

void PointcloudLocater::publish_combined()
{
    radar_msgs::msg::Cars combined_msg;
    combined_msg.header.frame_id = "map";
    combined_msg.header.stamp = this->now();
    {
        std::lock_guard<std::mutex> lock(mtx_);
        combined_msg.cars.insert(combined_msg.cars.end(),
                                 latest_ground_cars_.begin(), latest_ground_cars_.end());                       
        combined_msg.cars.insert(combined_msg.cars.end(),
                                 latest_drone_cars_.begin(), latest_drone_cars_.end());
    }

    both_publisher_->publish(combined_msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointcloudLocater>());
    rclcpp::shutdown();
    return 0;
}
